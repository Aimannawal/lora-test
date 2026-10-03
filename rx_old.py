from machine import Pin, SPI
import time
from sx127x import SX127x

# ===== SESUAIKAN PIN & FREKUENSI =====
PIN_SCK, PIN_MOSI, PIN_MISO = 18, 23, 19
PIN_CS, PIN_RST = 5, 14
FREQ = 433E6          # HARUS sama dengan TX
SHOW_RSSI = True      # info RSSI/SNR dicetak sebagai baris berawalan '#'
# =====================================

spi = SPI(1, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))
lora = SX127x(spi, cs=PIN_CS, rst=PIN_RST, freq=FREQ)
lora.rx_start()
print("# RX siap, freq", FREQ)

while True:
    data = lora.poll()
    if data is not None:
        try:
            line = data.decode().strip()
        except Exception:
            print("! <non-utf8>")
            continue
        if len(line.split(",")) == 14:
            print(line)  # CSV murni -> langsung bisa dibaca bridge/app
        else:
            print("! " + line)  # paket rusak
        if SHOW_RSSI:
            print("# rssi=%d snr=%.1f" % (lora.rssi, lora.snr))
    time.sleep_ms(5)
