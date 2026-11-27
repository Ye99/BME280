"""A register-level fake of a BME280 (or BMP280) on an I2C bus, for host tests."""
import struct

BME280_ID = 0x60
BMP280_ID = 0x58

# Calibration from the Bosch BMP280 datasheet worked example (section 8.2);
# the T and P formulas are identical on the BME280.
EXAMPLE_TP = dict(T1=27504, T2=26435, T3=-1000,
                  P1=36477, P2=-10685, P3=3024, P4=2855, P5=140,
                  P6=-7, P7=15500, P8=-14600, P9=6000)
# Typical humidity calibration, with negative H4/H5 variants tested separately.
EXAMPLE_H = dict(H1=75, H2=362, H3=0, H4=313, H5=50, H6=30)

RESET_TP = 0x80000
RESET_H = 0x8000


class FakeBME280:
    def __init__(self, chip_id=BME280_ID, address=0x76, tp=None, h=None,
                 im_update_reads=0, measuring_reads=0):
        self.address = address
        self.mem = bytearray(256)
        self.mem[0xD0] = chip_id
        self.chip_id = chip_id
        self.writes = []
        self.im_update_reads = im_update_reads
        self.im_update_pending = 0
        self.measuring_pending = 0
        self.measuring_reads = measuring_reads
        self.raw = None  # (adc_T, adc_P, adc_H) produced by the next conversion
        self.deferred_ctrl_meas = None  # mode change held until conversion ends
        self.set_calibration(tp or EXAMPLE_TP, h or EXAMPLE_H)
        self._load_reset_data()

    # -- helpers for tests --------------------------------------------------
    def set_calibration(self, tp, h):
        struct.pack_into("<HhhHhhhhhhhh", self.mem, 0x88,
                         tp["T1"], tp["T2"], tp["T3"], tp["P1"], tp["P2"], tp["P3"],
                         tp["P4"], tp["P5"], tp["P6"], tp["P7"], tp["P8"], tp["P9"])
        if self.chip_id != BME280_ID:
            return
        self.mem[0xA1] = h["H1"]
        struct.pack_into("<hB", self.mem, 0xE1, h["H2"], h["H3"])
        h4, h5 = h["H4"] & 0xFFF, h["H5"] & 0xFFF
        self.mem[0xE4] = (h4 >> 4) & 0xFF
        self.mem[0xE5] = (h4 & 0x0F) | ((h5 & 0x0F) << 4)
        self.mem[0xE6] = (h5 >> 4) & 0xFF
        struct.pack_into("<b", self.mem, 0xE7, h["H6"])

    def _load_reset_data(self):
        self._store(RESET_TP, RESET_TP, RESET_H)

    def _store(self, adc_t, adc_p, adc_h):
        self.mem[0xF7] = (adc_p >> 12) & 0xFF
        self.mem[0xF8] = (adc_p >> 4) & 0xFF
        self.mem[0xF9] = (adc_p << 4) & 0xF0
        self.mem[0xFA] = (adc_t >> 12) & 0xFF
        self.mem[0xFB] = (adc_t >> 4) & 0xFF
        self.mem[0xFC] = (adc_t << 4) & 0xF0
        self.mem[0xFD] = (adc_h >> 8) & 0xFF
        self.mem[0xFE] = adc_h & 0xFF

    def mode(self):
        return self.mem[0xF4] & 0x03

    def _convert(self):
        if self.raw is None:
            return
        adc_t, adc_p, adc_h = self.raw
        osr_t = (self.mem[0xF4] >> 5) & 7
        osr_p = (self.mem[0xF4] >> 2) & 7
        osr_h = self.mem[0xF2] & 7
        self._store(adc_t if osr_t else RESET_TP,
                    adc_p if osr_p else RESET_TP,
                    adc_h if osr_h else RESET_H)

    # -- the machine.I2C surface the driver uses ----------------------------
    def readfrom_mem(self, addr, reg, n):
        if addr != self.address:
            raise OSError(19)  # ENODEV
        if self.im_update_pending and 0x88 <= reg <= 0xE7 and reg != 0xD0:
            return bytes(n)  # NVM still being copied: calibration reads as zeros
        if reg == 0xF3:
            status = 0
            if self.im_update_pending:
                self.im_update_pending -= 1
                status |= 0x01
            if self.measuring_pending:
                self.measuring_pending -= 1
                status |= 0x08
                if not self.measuring_pending:
                    self._convert()
                    if self.deferred_ctrl_meas is not None:
                        self.mem[0xF4] = self.deferred_ctrl_meas
                        self.deferred_ctrl_meas = None
                    if self.mode() in (0x01, 0x02):
                        self.mem[0xF4] &= 0xFC  # forced mode drops back to sleep
            self.mem[0xF3] = status
        return bytes(self.mem[reg:reg + n])

    def writeto_mem(self, addr, reg, buf):
        if addr != self.address:
            raise OSError(19)
        buf = bytes(buf)
        self.writes.append((reg, buf))
        if reg == 0xE0:
            if buf[0] == 0xB6:
                for r in (0xF2, 0xF4, 0xF5):
                    self.mem[r] = 0
                self._load_reset_data()
                self.im_update_pending = self.im_update_reads
            return
        if self.measuring_pending and reg in (0xF2, 0xF4, 0xF5):
            # Datasheet 3.3: a mode change during a conversion is held until
            # the conversion ends; other control writes meanwhile are ignored.
            if reg == 0xF4 and self.deferred_ctrl_meas is None:
                self.deferred_ctrl_meas = buf[0]
            return
        self.mem[reg:reg + len(buf)] = buf
        if reg == 0xF4 and (buf[0] & 0x03) in (0x01, 0x02, 0x03):
            # A measurement starts as soon as forced or normal mode is entered.
            if self.measuring_reads:
                self.measuring_pending = self.measuring_reads
            else:
                self._convert()
            if (buf[0] & 0x03) in (0x01, 0x02) and not self.measuring_reads:
                self.mem[0xF4] &= 0xFC  # forced mode drops back to sleep
