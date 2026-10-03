# rx.py — LoRa Receiver + TCP Bridge (dengan AUTH) untuk GCS Flutter
# Upload ke ESP receiver sebagai main.py
# Butuh sx127x.py (versi baru) di ESP yang sama.
#
# Protokol TCP:
#   1. GCS connect, lalu kirim baris pertama:  AUTH <token>\n
#   2. Kalau token cocok, ESP kirim header CSV lalu forward tiap paket LoRa.
#   3. Kalau salah / timeout, koneksi ditutup (klien sah tidak ikut tertendang).
# Mendukung sampai MAX_CLIENTS GCS sekaligus (mis. laptop + Raspberry Pi).

from machine import Pin, SPI
import time
import socket
import network
from sx127x import SX127x

# ===== KONFIGURASI WIFI =====
WIFI_SSID     = "Nash"
WIFI_PASSWORD = "123456789"
TCP_PORT      = 9999
# ============================

# ===== AUTH =====
AUTH_TOKEN      = "eepisat-gcs-2026"   # harus sama dengan kAuthToken di main.dart
AUTH_TIMEOUT_MS = 3000
MAX_CLIENTS     = 3
# ================

# ===== SESUAIKAN PIN & FREKUENSI =====
PIN_SCK, PIN_MOSI, PIN_MISO = 18, 23, 19
PIN_CS, PIN_RST = 5, 14
FREQ = 433E6
SPI_BAUD = 1000000     # 1 MHz: lebih tahan kabel jumper di breadboard
SHOW_RSSI = True
DEBUG = True           # cetak [dbg] tiap 2 detik; set False kalau sudah stabil
# =====================================

CSV_HEADER = (b"TEAM_ID,MISSION_TIME,PACKET_COUNT,ALTITUDE,PRESSURE,"
              b"TEMPERATURE,VOLTAGE,ROLL,PITCH,YAW,GPS_LAT,GPS_LON,"
              b"GPS_ALT,STATE\r\n")

AUTH_LINE = ("AUTH " + AUTH_TOKEN).encode()


def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
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
        return ip
    print("[WiFi] GAGAL connect! Cek SSID/password.")
    return None


def start_server(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    s.listen(4)
    s.setblocking(False)
    return s


print("=== CanSat Ground Station - LoRa to TCP Bridge ===")
ip = wifi_connect()
if ip is None:
    raise SystemExit("WiFi gagal - restart")

spi = SPI(1, baudrate=SPI_BAUD, polarity=0, phase=0,
          sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))

while True:
    try:
        lora = SX127x(spi, cs=PIN_CS, rst=PIN_RST, freq=FREQ)
        break
    except Exception as e:
        print("[LoRa] init gagal:", e)
        time.sleep(1)
lora.rx_start()
print("[LoRa] Ready, freq", FREQ)

server = start_server(TCP_PORT)
clients = []   # socket yang sudah lolos AUTH
pending = []   # [sock, buf, t0, ip] menunggu AUTH
rx_count = 0
last_status = time.time()
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
                c.settimeout(1)                # send tidak boleh menggantung lama
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


def forward(line):
    payload = (line + "\r\n").encode()
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

    # -------- diagnosa CRC --------
    try:
        if lora._r(0x12) & 0x20:
            crc_err += 1
            print("[LoRa] CRC error #%d rssi=%d" % (crc_err, lora.rssi_now()))
    except Exception:
        pass
    
    # -------- poll LoRa --------
    try:
        data = lora.poll()
    except Exception as e:
        print("[LoRa] poll error:", e)
        data = None

    if data is not None:
        try:
            line = data.decode().strip()
        except Exception:
            print("! <non-utf8>")
            line = None

        if line is not None and len(line.split(",")) == 14:
            rx_count += 1
            print("[LoRa] RX #%d: %s" % (rx_count, line))
            if SHOW_RSSI:
                print("        # rssi=%d snr=%.1f" % (lora.rssi, lora.snr))
            if clients:
                forward(line)
            else:
                print("[TCP] Tidak ada GCS - paket tidak diteruskan")
        elif line is not None:
            print("! Paket rusak:", line)

    # -------- status --------
    if not clients and time.time() - last_status > 5:
        last_status = time.time()
        print("[status] WiFi OK, menunggu GCS...")

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
