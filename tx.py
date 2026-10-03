# tx.py — EEPISAT LoRa Transmitter Diagnostic
#
# Digunakan untuk debugging SX127x TX.
#
# Hardware:
# SCK  = GPIO18
# MOSI = GPIO23
# MISO = GPIO19
# CS   = GPIO5
# RST  = GPIO14
#
# Frequency = 433 MHz


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

INTERVAL_MS = 1000


# ============================================================
# START
# ============================================================

print()
print("========================================")
print("       EEPISAT LORA TX DIAGNOSTIC")
print("========================================")
print()

print("Frequency :", FREQ)
print("Power     :", POWER_DBM, "dBm")
print("SPI       :", SPI_BAUD)
print("SCK       :", PIN_SCK)
print("MOSI      :", PIN_MOSI)
print("MISO      :", PIN_MISO)
print("CS        :", PIN_CS)
print("RST       :", PIN_RST)

print()


# ============================================================
# SPI
# ============================================================

print("[TX] Initializing SPI...")

spi = SPI(
    1,
    baudrate=SPI_BAUD,
    polarity=0,
    phase=0,
    sck=Pin(PIN_SCK),
    mosi=Pin(PIN_MOSI),
    miso=Pin(PIN_MISO),
)

print("[TX] SPI OK")


# ============================================================
# SX127x
# ============================================================

print("[TX] Initializing SX127x...")

lora = SX127x(
    spi,
    cs=PIN_CS,
    rst=PIN_RST,
    freq=FREQ,
    power=POWER_DBM,
)

print("[TX] SX127x INIT OK")


# ============================================================
# REGISTER DEBUG
# ============================================================

def read_reg(addr):
    try:
        return lora._r(addr)
    except Exception as e:
        print("[DEBUG] read register error:", e)
        return -1


def dump_registers():

    print()
    print("---------- SX127x REGISTERS ----------")

    regs = {
        "VERSION       0x42": 0x42,
        "OP_MODE       0x01": 0x01,
        "FRF_MSB       0x06": 0x06,
        "FRF_MID       0x07": 0x07,
        "FRF_LSB       0x08": 0x08,
        "PA_CONFIG     0x09": 0x09,
        "FIFO_ADDR_PTR 0x0D": 0x0D,
        "FIFO_TX_BASE  0x0E": 0x0E,
        "FIFO_RX_BASE  0x0F": 0x0F,
        "IRQ_FLAGS     0x12": 0x12,
        "RX_NB_BYTES   0x13": 0x13,
        "MODEM_STAT    0x18": 0x18,
        "MODEM_CFG1    0x1D": 0x1D,
        "MODEM_CFG2    0x1E": 0x1E,
        "PREAMBLE_MSB  0x20": 0x20,
        "PREAMBLE_LSB  0x21": 0x21,
        "SYNC_WORD     0x39": 0x39,
    }

    for name, addr in regs.items():

        value = read_reg(addr)

        print(
            "%-25s = 0x%02X"
            % (name, value)
        )

    print("--------------------------------------")
    print()


dump_registers()


# ============================================================
# TEST PAYLOAD
# ============================================================

TEST_MESSAGE = (
    "EEPISAT,00:00:01,1,"
    "0.0,1013.2,29.5,7.42,"
    "0.1,0.2,0.0,"
    "-7.2762,112.7942,3.0,"
    "LAUNCH_PAD"
)

print("[TX] Test payload:")
print(TEST_MESSAGE)
print()

print(
    "[TX] Field count:",
    len(TEST_MESSAGE.split(","))
)

print()


# ============================================================
# MANUAL TX TEST
# ============================================================

print("========================================")
print("Starting TX test...")
print("========================================")
print()


pkt = 0

while True:

    pkt += 1

    # --------------------------------------------------------
    # Generate payload
    # --------------------------------------------------------

    seconds = pkt

    hh = seconds // 3600
    mm = (seconds // 60) % 60
    ss = seconds % 60

    mission_time = "%02d:%02d:%02d" % (
        hh,
        mm,
        ss,
    )

    line = (
        "EEPISAT,%s,%d,"
        "0.0,1013.2,29.5,7.42,"
        "0.1,0.2,0.0,"
        "-7.2762,112.7942,3.0,"
        "LAUNCH_PAD"
        % (
            mission_time,
            pkt,
        )
    )

    print()
    print("----------------------------------------")
    print("[TX] Packet:", pkt)
    print("[TX] Data:")
    print(line)
    print("----------------------------------------")

    # --------------------------------------------------------
    # Before TX
    # --------------------------------------------------------

    print(
        "[TX] OP_MODE before:",
        "0x%02X" % read_reg(0x01)
    )

    print(
        "[TX] IRQ before:",
        "0x%02X" % read_reg(0x12)
    )

    # --------------------------------------------------------
    # TX
    # --------------------------------------------------------

    start = time.ticks_ms()

    try:

        ok = lora.send(
            line,
            timeout_ms=1000,
        )

        elapsed = time.ticks_diff(
            time.ticks_ms(),
            start,
        )

        print(
            "[TX] Result:",
            ok,
        )

        print(
            "[TX] Time:",
            elapsed,
            "ms"
        )

    except Exception as e:

        print(
            "[TX] EXCEPTION:",
            repr(e),
        )

        ok = False

    # --------------------------------------------------------
    # After TX
    # --------------------------------------------------------

    print(
        "[TX] OP_MODE after:",
        "0x%02X" % read_reg(0x01)
    )

    print(
        "[TX] IRQ after:",
        "0x%02X" % read_reg(0x12)
    )

    print(
        "[TX] PA_CONFIG:",
        "0x%02X" % read_reg(0x09)
    )

    print(
        "[TX] FIFO_ADDR_PTR:",
        "0x%02X" % read_reg(0x0D)
    )

    # --------------------------------------------------------
    # Interpret result
    # --------------------------------------------------------

    irq = read_reg(0x12)
    mode = read_reg(0x01)

    print()

    print("[DEBUG] Interpretation:")

    if irq & 0x08:
        print(
            "[DEBUG] TxDone = YES"
        )
    else:
        print(
            "[DEBUG] TxDone = NO"
        )

    if mode & 0x80:
        print(
            "[DEBUG] LoRa mode = YES"
        )
    else:
        print(
            "[DEBUG] LoRa mode = NO"
        )

    mode_number = mode & 0x07

    if mode_number == 0:
        print(
            "[DEBUG] Mode = SLEEP"
        )
    elif mode_number == 1:
        print(
            "[DEBUG] Mode = STANDBY"
        )
    elif mode_number == 3:
        print(
            "[DEBUG] Mode = TX"
        )
    elif mode_number == 5:
        print(
            "[DEBUG] Mode = RX_CONTINUOUS"
        )
    else:
        print(
            "[DEBUG] Mode number =",
            mode_number,
        )

    # --------------------------------------------------------
    # Wait
    # --------------------------------------------------------

    time.sleep_ms(INTERVAL_MS)
