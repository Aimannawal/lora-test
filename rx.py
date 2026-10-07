# rx.py — LoRa Receiver + TCP Bridge (dengan AUTH) untuk GCS Flutter
# Upload ke ESP receiver sebagai main.py (bersama sx127x.py).
#
# Radio: paket binary 30 byte (lihat layout di tx.py).
# TCP ke GCS tetap teks, jadi format lama tidak berubah:
#   1. GCS connect, lalu kirim baris pertama:  AUTH <token>\n
#   2. Kalau token cocok, ESP kirim header CSV lalu tiap paket LoRa sebagai
#      CSV 14 field + checksum, diikuti satu baris  "#LINK <rssi> <snr>".
#   3. Kalau salah / timeout, koneksi ditutup.
# Mendukung sampai MAX_CLIENTS GCS sekaligus (mis. laptop + Raspberry Pi).
#
# CSV: MISSION_TIME,PACKET_ID,TEAM_ID,PRESSURE,ALTITUDE,VOLTAGE,CURRENT,
#      STATE,LAT,LON,ROLL,PITCH,YAW,CHECKSUM

from machine import Pin, SPI
import time
import socket
import network
import struct
from sx127x import SX127x

# ===== KONFIGURASI WIFI =====
WIFI_SSID     = "Iwin"
WIFI_PASSWORD = "12345678"
TCP_PORT      = 9999
# ============================

# ===== AUTH =====
AUTH_TOKEN      = "eepisat-gcs-2026"   # harus sama dengan kAuthToken di main.dart
AUTH_TIMEOUT_MS = 3000
MAX_CLIENTS     = 3
# ================

# ===== SESUAIKAN PIN & RADIO =====
PIN_SCK, PIN_MOSI, PIN_MISO = 18, 23, 19
PIN_CS, PIN_RST = 5, 14
FREQ = 433E6
SF = 10                # HARUS sama dengan tx.py
BW = 7                 # 7 = 125 kHz
PREAMBLE = 12
SPI_BAUD = 1000000
EXPECT_TEAM = 8        # paket dengan team id lain dibuang (anti paket asing)
SHOW_RSSI = True
DEBUG = True           # cetak [dbg] tiap 2 detik; set False kalau sudah stabil
# =================================

FMT = "<IHBHhHhBiihhh"
SIZE = struct.calcsize(FMT)   # 30
N_U16 = 0xFFFF
N_I16 = -32768
N_I32 = 0x7FFFFFFF

CSV_HEADER = (b"MISSION_TIME,PACKET_ID,TEAM_ID,PRESSURE,ALTITUDE,VOLTAGE,"
              b"CURRENT,STATE,GPS_LAT,GPS_LON,ROLL,PITCH,YAW,CHECKSUM\r\n")

AUTH_LINE = ("AUTH " + AUTH_TOKEN).encode()


def checksum(s):
    c = 0
    for b in s.encode():
        c ^= b
    return "%02X" % c


def fx(v, none, d):
    """Integer berskala -> teks desimal tanpa float. none -> 'nan'."""
    if v == none:
        return "nan"
    sign = "-" if v < 0 else ""
    a = str(abs(v))
    if len(a) <= d:
        a = "0" * (d + 1 - len(a)) + a
    return sign + a[:-d] + "." + a[-d:]


def decode_packet(data):
    """bytes (30) -> baris CSV 14 field + checksum, atau None kalau bukan paket kita."""
    if len(data) != SIZE:
        return None
    (sec, pid, team, p, alt, volt, cur, st,
     lat, lon, ro, pi, ya) = struct.unpack(FMT, data)
    if team != EXPECT_TEAM:
        return None
    body = "%02d:%02d:%02d,%04d,%02d,%s,%s,%s,%s,%d,%s,%s,%s,%s,%s" % (
        sec // 3600, (sec // 60) % 60, sec % 60, pid, team,
        fx(p, N_U16, 1), fx(alt, N_I16, 1),
        fx(volt, N_U16, 2), fx(cur, N_I16, 2), st,
        fx(lat, N_I32, 5), fx(lon, N_I32, 5),
        fx(ro, N_I16, 1), fx(pi, N_I16, 1), fx(ya, N_I16, 1))
    return body + "," + checksum(body)


def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    try:
        wlan.config(pm=network.WLAN.PM_NONE)   # matikan WiFi power-save (latensi lebih rendah)
    except Exception:
        pass
    if not wlan.isconnected():
        print("[WiFi] Connecting to", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        t0 = time.time()
        while not wlan.isconnected() and time.time() - t0 < 20:
            time.sleep(0.5)
            print(".", end="")
        print()
    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print("[WiFi] IP Address :", ip)
        print("[GCS]  Set kReceiverHost di main.dart ke:", ip)
        return wlan, ip
    print("[WiFi] GAGAL connect! Cek SSID/password.")
    return wlan, None


def start_server(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    s.listen(4)
    s.setblocking(False)
    return s


print("=== CanSat Ground Station - LoRa to TCP Bridge (binary) ===")
wlan, ip = wifi_connect()
if ip is None:
    raise SystemExit("WiFi gagal - restart")

spi = SPI(1, baudrate=SPI_BAUD, polarity=0, phase=0,
          sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))

while True:
    try:
        lora = SX127x(spi, cs=PIN_CS, rst=PIN_RST, freq=FREQ, sf=SF, bw=BW,
                      preamble=PREAMBLE)
        break
    except Exception as e:
        print("[LoRa] init gagal:", e)
        time.sleep(1)
lora.rx_start()
print("[LoRa] Ready, freq", FREQ, "SF", SF, "paket", SIZE, "byte")

server = start_server(TCP_PORT)
clients = []   # socket yang sudah lolos AUTH
pending = []   # [sock, buf, t0, ip] menunggu AUTH
rx_count = 0
bad_count = 0
last_crc = 0
last_status = time.time()
last_wifi = time.time()
_dbg = time.ticks_ms()

print("Menunggu GCS connect ke", ip, ":", TCP_PORT)


def safe_close(c):
    try:
        c.close()
    except Exception:
        pass


def drop_client(c):
    safe_close(c)
    if c in clients:
        clients.remove(c)


def accept_new():
    try:
        c, addr = server.accept()
    except OSError:
        return
    c.setblocking(False)
    pending.append([c, b"", time.ticks_ms(), addr[0]])


def check_pending():
    for p in pending[:]:
        c, buf, t0, peer = p
        try:
            d = c.recv(64)
            if d == b"":                       # peer menutup koneksi
                pending.remove(p)
                safe_close(c)
                continue
            if d:
                buf += d
                p[1] = buf
        except OSError:
            pass                               # belum ada data

        if b"\n" in buf:
            pending.remove(p)
            first = buf.split(b"\n")[0].strip()
            if first == AUTH_LINE:
                c.settimeout(0.5)
                try:
                    c.send(CSV_HEADER)
                except OSError as e:
                    print("[TCP] Gagal kirim header:", e)
                    safe_close(c)
                    continue
                clients.append(c)
                if len(clients) > MAX_CLIENTS:
                    drop_client(clients[0])
                print("[TCP] GCS auth OK from", peer, "| total", len(clients))
            else:
                print("[TCP] AUTH gagal dari", peer)
                safe_close(c)
        elif len(buf) > 64 or time.ticks_diff(time.ticks_ms(), t0) > AUTH_TIMEOUT_MS:
            pending.remove(p)
            print("[TCP] AUTH timeout/invalid dari", peer)
            safe_close(c)


def forward(line, rssi, snr):
    payload = (line + "\r\n#LINK %d %.1f\r\n" % (rssi, snr)).encode()
    for c in clients[:]:
        try:
            c.send(payload)
        except OSError as e:
            print("[TCP] Gagal forward, client dilepas:", e)
            drop_client(c)


while True:
    # -------- koneksi baru + cek AUTH --------
    accept_new()
    check_pending()

    # -------- poll LoRa --------
    try:
        data = lora.poll()
    except Exception as e:
        print("[LoRa] poll error:", e)
        data = None

    if lora.crc_errors != last_crc:
        last_crc = lora.crc_errors
        print("[LoRa] CRC error #%d rssi=%d" % (last_crc, lora.rssi_now()))

    if data is not None:
        line = decode_packet(data)
        if line is not None:
            rx_count += 1
            print("[LoRa] RX #%d: %s" % (rx_count, line))
            if SHOW_RSSI:
                print("        # rssi=%d snr=%.1f" % (lora.rssi, lora.snr))
            if clients:
                forward(line, lora.rssi, lora.snr)
            else:
                print("[TCP] Tidak ada GCS - paket tidak diteruskan")
        else:
            bad_count += 1
            print("! Paket asing / panjang salah (%d byte), total %d" % (len(data), bad_count))

    # -------- status --------
    if not clients and time.time() - last_status > 5:
        last_status = time.time()
        print("[status] WiFi OK, menunggu GCS...")

    # -------- WiFi putus -> sambung lagi --------
    if time.time() - last_wifi > 5:
        last_wifi = time.time()
        try:
            if not wlan.isconnected():
                print("[WiFi] putus, reconnect...")
                wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        except Exception as e:
            print("[WiFi] reconnect error:", e)

    # -------- watchdog radio (tiap 2 detik) --------
    if time.ticks_diff(time.ticks_ms(), _dbg) > 2000:
        _dbg = time.ticks_ms()
        try:
            if lora.rx_watchdog():
                print("[LoRa] chip jatuh, di-reinit (total %d)" % lora.reinit_count)
            elif DEBUG:
                print("[dbg] mode=0x%02X rssi=%d modem=0x%02X irq=0x%02X" % (
                    lora._r(0x01), lora.rssi_now(), lora._r(0x18), lora._r(0x12)))
        except Exception as e:
            print("[LoRa] watchdog error:", e)

    time.sleep_ms(5)