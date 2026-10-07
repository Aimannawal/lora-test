# tx.py — EEPISAT LoRa Transmitter (paket binary integer, 30 byte)
# Upload ke ESP pemancar sebagai main.py (bersama sx127x.py).
#
# Layout paket (little-endian, 30 byte) — HARUS sama dengan rx.py:
#   sec      uint32  detik sejak power on
#   packet   uint16  nomor paket (0801, 0802, ...)
#   team     uint8
#   pressure uint16  hPa x 10        (0xFFFF  = nan)
#   altitude int16   m x 10          (-32768  = nan)
#   voltage  uint16  V x 100         (0xFFFF  = nan)
#   current  int16   A x 100         (-32768  = nan)
#   state    uint8   0=LAUNCH_PAD 1=ASCENT 2=APOGEE 3=DESCENT 4=LANDED
#   lat      int32   derajat x 1e5   (0x7FFFFFFF = nan)
#   lon      int32   derajat x 1e5   (0x7FFFFFFF = nan)
#   roll     int16   derajat x 10    (-32768  = nan)
#   pitch    int16   derajat x 10    (-32768  = nan)
#   yaw      int16   derajat x 10    (-32768  = nan)
#
# Checksum tidak dikirim lewat radio (chip sudah punya CRC hardware).
# Receiver yang menyusun CSV 14 field + checksum untuk GCS.
#
# Hardware: SCK=18 MOSI=23 MISO=19 CS=5 RST=14, 433 MHz

from machine import Pin, SPI
import struct
import time

from sx127x import SX127x


# ============================================================
# CONFIG
# ============================================================
PIN_SCK = 18
PIN_MOSI = 23
PIN_MISO = 19
PIN_CS = 5
PIN_RST = 14

FREQ = 433E6
SPI_BAUD = 1000000
POWER_DBM = 20     # naikkan ke 20 hanya kalau catu daya kuat (+470 uF dekat modul)
SF = 10              # HARUS sama dengan rx.py (SF10 ~0,5 s, SF11 ~1,0 s untuk 30 byte)
BW = 7               # 7 = 125 kHz
PREAMBLE = 12

TEAM_ID = 8
PACKET_START = 801   # paket pertama = 0801

INTERVAL_MS = 1000
DEBUG = True


# ============================================================
# PACKET
# ============================================================
FMT = "<IHBHhHhBiihhh"
SIZE = struct.calcsize(FMT)   # 30
N_U16 = 0xFFFF
N_I16 = -32768
N_I32 = 0x7FFFFFFF


def q(v, scale, none, lo, hi):
    """Float -> integer berskala. None / NaN -> kode 'nan'."""
    if v is None or v != v:
        return none
    x = int(round(v * scale))
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def build_packet(pkt, sec, pressure, altitude, voltage, current,
                 state, lat, lon, roll, pitch, yaw):
    st = state if state is not None else 255
    return struct.pack(
        FMT,
        sec,
        (PACKET_START + pkt - 1) & 0xFFFF,
        TEAM_ID,
        q(pressure, 10, N_U16, 0, 65534),
        q(altitude, 10, N_I16, -32767, 32767),
        q(voltage, 100, N_U16, 0, 65534),
        q(current, 100, N_I16, -32767, 32767),
        st & 0xFF,
        q(lat, 100000, N_I32, -90 * 100000, 90 * 100000),
        q(lon, 100000, N_I32, -180 * 100000, 180 * 100000),
        q(roll, 10, N_I16, -32767, 32767),
        q(pitch, 10, N_I16, -32767, 32767),
        q(yaw, 10, N_I16, -32767, 32767),
    )


def read_sensors():
    # TODO: ganti dengan pembacaan sensor asli.
    # Kalau sensor gagal, isi dengan None.
    return {
        "pressure": 1013.2,
        "altitude": 0.0,
        "voltage": 7.42,
        "current": 0.35,
        "state": 0,
        "lat": -7.2762,
        "lon": 112.7942,
        "roll": 0.1,
        "pitch": 0.2,
        "yaw": 0.0,
    }


def airtime_ms(sf, bw_hz, n, cr=1, preamble=12):
    """Perkiraan airtime paket (explicit header, CRC on)."""
    tsym_us = ((1 << sf) * 1000000) // bw_hz
    de = 1 if tsym_us > 16000 else 0
    num = 8 * n - 4 * sf + 28 + 16
    den = 4 * (sf - 2 * de)
    nsym = 8 + ((num + den - 1) // den) * (cr + 4)
    total_us = (preamble * 4 + 17) * tsym_us // 4 + nsym * tsym_us
    return total_us // 1000


# ============================================================
# INIT
# ============================================================
print()
print("========================================")
print("       EEPISAT LORA TX (binary)")
print("========================================")

spi = SPI(
    1,
    baudrate=SPI_BAUD,
    polarity=0,
    phase=0,
    sck=Pin(PIN_SCK),
    mosi=Pin(PIN_MOSI),
    miso=Pin(PIN_MISO),
)

lora = SX127x(
    spi,
    cs=PIN_CS,
    rst=PIN_RST,
    freq=FREQ,
    sf=SF,
    bw=BW,
    power=POWER_DBM,
    preamble=PREAMBLE,
)

_bw_hz = (7800, 10400, 15600, 20800, 31250, 41700, 62500,
          125000, 250000, 500000)[BW]
AIR = airtime_ms(SF, _bw_hz, SIZE, preamble=PREAMBLE)
print("Freq:", FREQ, "| SF:", SF, "| BW:", _bw_hz, "| Power:", POWER_DBM, "dBm")
print("Paket:", SIZE, "byte | airtime ~", AIR, "ms | interval", INTERVAL_MS, "ms")
if AIR > INTERVAL_MS * 8 // 10:
    print("[WARN] airtime terlalu dekat dengan interval, 1 Hz tidak akan stabil")
print("[TX] SX127x INIT OK")


# ============================================================
# LOOP (tepat 1 Hz, jam misi dari waktu asli)
# ============================================================
t_start = time.ticks_ms()
next_t = time.ticks_add(t_start, INTERVAL_MS)
pkt = 0

while True:
    pkt += 1
    sec = time.ticks_diff(time.ticks_ms(), t_start) // 1000

    s = read_sensors()
    data = build_packet(
        pkt, sec,
        s["pressure"], s["altitude"], s["voltage"], s["current"],
        s["state"], s["lat"], s["lon"],
        s["roll"], s["pitch"], s["yaw"],
    )

    start = time.ticks_ms()
    try:
        ok = lora.send(data, timeout_ms=3000)
    except Exception as e:
        print("[TX] EXCEPTION:", repr(e))
        ok = False
    elapsed = time.ticks_diff(time.ticks_ms(), start)

    print("[TX] #%d t=%ds bytes=%d ok=%s %d ms reinit=%d" % (
        pkt, sec, len(data), ok, elapsed, lora.reinit_count))

    if DEBUG:
        try:
            print("     OP_MODE=0x%02X IRQ=0x%02X" % (lora._r(0x01), lora._r(0x12)))
        except Exception as e:
            print("     [DEBUG] read register error:", e)

    wait = time.ticks_diff(next_t, time.ticks_ms())
    if wait > 0:
        time.sleep_ms(wait)
        next_t = time.ticks_add(next_t, INTERVAL_MS)
    else:
        # telat (airtime > interval): jangan mengejar, mulai ulang jadwal
        next_t = time.ticks_add(time.ticks_ms(), INTERVAL_MS)