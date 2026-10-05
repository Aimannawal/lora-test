# tx.py — EEPISAT LoRa Transmitter (format paket ringkas + checksum)
#
# Format (14 field CSV):
# MISSION_TIME,PACKET_ID,TEAM_ID,PRESSURE,ALTITUDE,VOLTAGE,CURRENT,
# STATE,LAT,LON,ROLL,PITCH,YAW,CHECKSUM
#
# Contoh:
# 00:00:01,0801,08,1013.2,0.0,7.42,0.35,0,-7.27620,112.79420,0.1,0.2,0.0,5A
#
# STATE: 0=LAUNCH_PAD 1=ASCENT 2=APOGEE 3=DESCENT 4=LANDED
# Sensor error / belum ada data -> kirim None, otomatis jadi "nan"
# (di GCS muncul NaN merah).
#
# Hardware:
# SCK=GPIO18 MOSI=GPIO23 MISO=GPIO19 CS=GPIO5 RST=GPIO14, 433 MHz

from machine import Pin, SPI
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
POWER_DBM = 17
SF = 9              # HARUS sama dengan rx.py. SF7 = cepat, SF9 = lebih jauh

TEAM_ID = "08"
PACKET_START = 801  # paket pertama = 0801

INTERVAL_MS = 1000
DEBUG = True


# ============================================================
# PACKET
# ============================================================

def checksum(s):
    c = 0
    for b in s.encode():
        c ^= b
    return "%02X" % c


def num(v, digits):
    if v is None:
        return "nan"
    return "%.*f" % (digits, v)


def build_packet(pkt, mission_time, pressure, altitude, voltage, current,
                 state, lat, lon, roll, pitch, yaw):
    body = "%s,%04d,%s,%s,%s,%s,%s,%d,%s,%s,%s,%s,%s" % (
        mission_time,
        PACKET_START + pkt - 1,
        TEAM_ID,
        num(pressure, 1),
        num(altitude, 1),
        num(voltage, 2),
        num(current, 2),
        state,
        num(lat, 5),
        num(lon, 5),
        num(roll, 1),
        num(pitch, 1),
        num(yaw, 1),
    )
    return body + "," + checksum(body)


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


# ============================================================
# INIT
# ============================================================

print()
print("========================================")
print("       EEPISAT LORA TX")
print("========================================")
print("Freq:", FREQ, "| SF:", SF, "| Power:", POWER_DBM, "dBm")

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
    power=POWER_DBM,
)
print("[TX] SX127x INIT OK")


def read_reg(addr):
    try:
        return lora._r(addr)
    except Exception as e:
        print("[DEBUG] read register error:", e)
        return -1


# ============================================================
# LOOP
# ============================================================

pkt = 0

while True:
    pkt += 1

    seconds = pkt
    mission_time = "%02d:%02d:%02d" % (
        seconds // 3600,
        (seconds // 60) % 60,
        seconds % 60,
    )

    s = read_sensors()
    line = build_packet(
        pkt, mission_time,
        s["pressure"], s["altitude"], s["voltage"], s["current"],
        s["state"], s["lat"], s["lon"],
        s["roll"], s["pitch"], s["yaw"],
    )

    start = time.ticks_ms()
    try:
        ok = lora.send(line, timeout_ms=2000)
    except Exception as e:
        print("[TX] EXCEPTION:", repr(e))
        ok = False
    elapsed = time.ticks_diff(time.ticks_ms(), start)

    print("[TX] #%d %s | bytes=%d | ok=%s | %d ms" % (
        pkt, line, len(line), ok, elapsed))

    if DEBUG:
        print("     OP_MODE=0x%02X IRQ=0x%02X" % (read_reg(0x01), read_reg(0x12)))

    time.sleep_ms(INTERVAL_MS)
