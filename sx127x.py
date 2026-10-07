# sx127x.py — driver LoRa SX1276/78 untuk MicroPython (ESP32)
# Dipakai bersama oleh tx.py dan rx.py (upload ke kedua ESP).
#
# Fitur:
#   - reinit() / healthy() / ensure(): pemulihan otomatis kalau chip "jatuh"
#   - FIFO burst read/write (1 transaksi SPI)
#   - Low Data Rate Optimize (LDRO) otomatis untuk SF11/SF12 @125 kHz
#   - Preamble bisa diatur (default 12 simbol)
#   - Daya TX sampai 20 dBm (PA_DAC), default maks 17 dBm
#   - RSSI paket dengan koreksi SNR, penghitung CRC error
#   - rx_watchdog(): pastikan receiver tetap di mode RX continuous

from machine import Pin
import time

_BW_HZ = (7800, 10400, 15600, 20800, 31250, 41700, 62500,
          125000, 250000, 500000)


class SX127x:
    def __init__(self, spi, cs, rst, freq=433E6, sf=7, bw=7, cr=1,
                 power=17, sync=0x12, preamble=12):
        # bw: 6 = 62.5 kHz, 7 = 125 kHz, 8 = 250 kHz | cr: 1 = 4/5
        self.spi = spi
        self.cs = Pin(cs, Pin.OUT, value=1)
        self.rst = Pin(rst, Pin.OUT, value=1)
        self.freq = freq
        self.sf = min(max(sf, 7), 12)
        self.bw = bw
        self.cr = cr
        self.power = min(max(power, 2), 20)
        self.sync = sync
        self.preamble = preamble
        self.rssi = 0
        self.snr = 0
        self.reinit_count = 0
        self.crc_errors = 0
        self.reinit()

    # ---------- SPI dasar ----------
    def _r(self, a):
        self.cs(0)
        self.spi.write(bytes([a & 0x7F]))
        v = self.spi.read(1)
        self.cs(1)
        return v[0]

    def _w(self, a, v):
        self.cs(0)
        self.spi.write(bytes([a | 0x80, v]))
        self.cs(1)

    def _burst_write(self, a, data):
        self.cs(0)
        self.spi.write(bytes([a | 0x80]) + data)
        self.cs(1)

    def _burst_read(self, a, n):
        self.cs(0)
        self.spi.write(bytes([a & 0x7F]))
        d = self.spi.read(n)
        self.cs(1)
        return d

    # ---------- helper ----------
    def _ldro(self):
        # simbol > 16 ms -> LDRO wajib aktif
        bw_hz = _BW_HZ[self.bw]
        return 1 if ((1 << self.sf) * 1000) // bw_hz > 16 else 0

    def _rssi_offset(self):
        return 164 if self.freq < 525E6 else 157

    # ---------- init / recovery ----------
    def reinit(self):
        ver = 0
        ok = False
        for _ in range(5):
            self.rst(0)
            time.sleep_ms(10)
            self.rst(1)
            time.sleep_ms(20)
            ver = self._r(0x42)
            if ver == 0x12:
                ok = True
                break
            time.sleep_ms(50)
        if not ok:
            raise RuntimeError(
                "SX127x tidak terdeteksi (version=0x%02X). Cek wiring/CS/RST" % ver)

        # mode LoRa hanya bisa diganti dari sleep
        self._w(0x01, 0x00)
        time.sleep_ms(10)
        self._w(0x01, 0x80)  # LoRa + sleep
        time.sleep_ms(10)

        frf = int((int(self.freq) << 19) / 32000000)
        self._w(0x06, (frf >> 16) & 0xFF)
        self._w(0x07, (frf >> 8) & 0xFF)
        self._w(0x08, frf & 0xFF)

        self._w(0x0E, 0x00)  # FIFO TX base
        self._w(0x0F, 0x00)  # FIFO RX base
        self._w(0x0C, 0x23)  # LNA gain G1 (maks) + boost HF

        # PA_BOOST
        if self.power >= 20:
            self._w(0x4D, 0x87)   # PA_DAC: +20 dBm
            self._w(0x0B, 0x31)   # OCP ~140 mA
            self._w(0x09, 0x80 | 0x0F)
        else:
            self._w(0x4D, 0x84)   # PA_DAC normal
            self._w(0x0B, 0x2B)   # OCP 100 mA
            self._w(0x09, 0x80 | (self.power - 2))

        self._w(0x1D, (self.bw << 4) | (self.cr << 1))  # explicit header
        self._w(0x1E, (self.sf << 4) | 0x04)            # SF + CRC on
        self._w(0x26, 0x04 | (self._ldro() << 3))       # AGC auto + LDRO
        self._w(0x20, (self.preamble >> 8) & 0xFF)
        self._w(0x21, self.preamble & 0xFF)
        self._w(0x31, (self._r(0x31) & 0xF8) | 0x03)    # detection optimize SF7-12
        self._w(0x37, 0x0A)                             # detection threshold SF7-12
        self._w(0x39, self.sync)
        self._w(0x01, 0x81)  # standby
        time.sleep_ms(5)

        if (self._r(0x39) != self.sync
                or self._r(0x1D) != ((self.bw << 4) | (self.cr << 1))):
            raise RuntimeError("Konfigurasi tidak tersimpan di chip (SPI/daya tidak stabil)")

    def healthy(self):
        try:
            return (self._r(0x42) == 0x12
                    and bool(self._r(0x01) & 0x80)
                    and self._r(0x39) == self.sync)
        except Exception:
            return False

    def ensure(self):
        """Reinit kalau chip tidak waras. True kalau tadi di-reinit."""
        if not self.healthy():
            self.reinit_count += 1
            self.reinit()
            return True
        return False

    # ---------- TX ----------
    def send(self, data, timeout_ms=3000):
        if isinstance(data, str):
            data = data.encode()
        self.ensure()
        self._w(0x01, 0x81)      # standby
        self._w(0x12, 0xFF)      # bersihkan flag IRQ
        self._w(0x0D, 0x00)      # FIFO pointer = TX base
        self._burst_write(0x00, data)
        self._w(0x22, len(data))
        self._w(0x01, 0x83)      # TX
        t0 = time.ticks_ms()
        while not (self._r(0x12) & 0x08):
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                self.reinit_count += 1
                self.reinit()
                return False
            time.sleep_ms(1)
        self._w(0x12, 0xFF)
        self._w(0x01, 0x81)
        return True

    # ---------- RX ----------
    def rx_start(self):
        self.ensure()
        self._w(0x01, 0x81)
        self._w(0x12, 0xFF)
        self._w(0x0F, 0x00)
        self._w(0x0D, 0x00)
        self._w(0x01, 0x85)  # RX continuous

    def rx_watchdog(self):
        """Panggil berkala di receiver. True kalau chip barusan di-reinit."""
        if (not self.healthy()) or (self._r(0x01) & 0x07) != 5:
            self.reinit_count += 1
            self.reinit()
            self.rx_start()
            return True
        return False

    def rssi_now(self):
        return self._r(0x1B) - self._rssi_offset()

    def poll(self):
        """Return bytes kalau ada paket valid, selain itu None."""
        f = self._r(0x12)
        if f == 0xFF or not (f & 0x40):
            return None
        self._w(0x12, 0xFF)
        if f & 0x20:  # CRC error
            self.crc_errors += 1
            return None
        n = self._r(0x13)
        if n == 0:
            return None
        self._w(0x0D, self._r(0x10))
        data = self._burst_read(0x00, n)
        s = self._r(0x19)
        self.snr = (s - 256 if s > 127 else s) / 4
        rssi = self._r(0x1A) - self._rssi_offset()
        if self.snr < 0:
            rssi += self.snr   # koreksi RSSI saat sinyal di bawah noise floor
        self.rssi = int(rssi)
        return data