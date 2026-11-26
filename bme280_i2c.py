# Author(s): Jonathan Hanson 2018
#
# This is more or less a straight read of the Bosch data sheet at:
# https://www.bosch-sensortec.com/bst/products/all_products/bme280
# and specifically:
# https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-12.pdf
#
# Modeled on the reference library Bosch Sensortec C library at:
# https://github.com/BoschSensortec/BME280_driver
#
# The development of this module was heavily guided by the prior work done
# by Peter Dahlberg et al at:
# https://github.com/catdog2/mpy_bme280_esp8266

try:
    from micropython import const
except ImportError:  # desktop Python, for host-side tests
    def const(value):
        return value

try:
    from ustruct import unpack, unpack_from
except ImportError:
    from struct import unpack, unpack_from

try:
    from utime import sleep_ms
except ImportError:
    from time import sleep as _sleep

    def sleep_ms(ms):
        _sleep(ms / 1000)

# BME280 default address
BME280_I2C_ADDR_PRIM = const(0x76)
BME280_I2C_ADDR_SEC = const(0x77)

# Sensor Power Mode Options
BME280_SLEEP_MODE = const(0x00)
BME280_FORCED_MODE = const(0x01)
BME280_NORMAL_MODE = const(0x03)

# Oversampling Options
BME280_NO_OVERSAMPLING = const(0x00)
BME280_OVERSAMPLING_1X = const(0x01)
BME280_OVERSAMPLING_2X = const(0x02)
BME280_OVERSAMPLING_4X = const(0x03)
BME280_OVERSAMPLING_8X = const(0x04)
BME280_OVERSAMPLING_16X = const(0x05)

# Standby Duration Options
BME280_STANDBY_TIME_500_US = const(0x00)  # Note this is microseconds, so 0.5 ms
BME280_STANDBY_TIME_62_5_MS = const(0x01)
BME280_STANDBY_TIME_125_MS = const(0x02)
BME280_STANDBY_TIME_250_MS = const(0x03)
BME280_STANDBY_TIME_500_MS = const(0x04)
BME280_STANDBY_TIME_1000_MS = const(0x05)
BME280_STANDBY_TIME_10_MS = const(0x06)
BME280_STANDBY_TIME_20_MS = const(0x07)

# Filter Coefficient Options
BME280_FILTER_COEFF_OFF = const(0x00)
BME280_FILTER_COEFF_2 = const(0x01)
BME280_FILTER_COEFF_4 = const(0x02)
BME280_FILTER_COEFF_8 = const(0x03)
BME280_FILTER_COEFF_16 = const(0x04)

# Chip IDs. The BMP280 shares the temperature and pressure blocks but has no
# humidity sensor.
_BME280_CHIP_ID = const(0x60)
_BMP280_CHIP_IDS = (0x56, 0x57, 0x58)

# Register Addresses
_BME280_CHIP_ID_ADDR = const(0xD0)
_BME280_RESET_ADDR = const(0xE0)
_BME280_TEMP_PRESS_CALIB_DATA_ADDR = const(0x88)
_BME280_HUMIDITY_CALIB_DATA_ADDR = const(0xE1)
_BME280_CTRL_HUM_ADDR = const(0xF2)
_BME280_STATUS_ADDR = const(0xF3)
_BME280_CTRL_MEAS_ADDR = const(0xF4)
_BME280_PWR_CTRL_ADDR = _BME280_CTRL_MEAS_ADDR  # mode bits live in ctrl_meas
_BME280_CONFIG_ADDR = const(0xF5)
_BME280_DATA_ADDR = const(0xF7)

# Status register bits
_STATUS_MEASURING = const(0x08)
_STATUS_IM_UPDATE = const(0x01)

# Register range sizes
_BME280_TEMP_PRESS_CALIB_DATA_LEN = const(26)
_BME280_HUMIDITY_CALIB_DATA_LEN = const(7)
_BME280_P_T_H_DATA_LEN = const(8)

# Data register contents when a measurement was skipped, or none has run yet
# since reset (datasheet section 5.4.7 to 5.4.9).
_SKIPPED_TP = const(0x80000)
_SKIPPED_H = const(0x8000)

# Polling limits, in 1 ms steps. The longest conversion (16x on all three
# channels) takes about 113 ms (datasheet section 9.1).
_NVM_COPY_TIMEOUT_MS = const(20)
_MEASUREMENT_TIMEOUT_MS = const(150)

_NO_READING = {"temperature": None, "pressure": None, "humidity": None}


class BME280_I2C:
    def __init__(self, address: int = BME280_I2C_ADDR_PRIM, i2c=None):
        """
        Check the chip ID, reset the sensor, and load its calibration data.

        Works with the BME280 and the BMP280; on a BMP280 has_humidity is
        False and humidity readings are None. Raises OSError if the device
        at this address is neither.
        """
        self.address = address

        if i2c is None:
            raise ValueError('A configured I2C object is required.')
        self.i2c = i2c

        self.chip_id = self._read_chip_id()
        self.has_humidity = self.chip_id == _BME280_CHIP_ID
        self._soft_reset()
        self._load_calibration_data()

    def _read_chip_id(self) -> int:
        """
        Read the chip ID and check it belongs to a BME280 or BMP280.
        If it doesn't, wait 1ms and try again; after 5 tries raise OSError,
        before anything has been written to the device.
        """
        chip_id = None
        for _ in range(5):
            chip_id = self.i2c.readfrom_mem(self.address, _BME280_CHIP_ID_ADDR, 1)[0]
            if chip_id == _BME280_CHIP_ID or chip_id in _BMP280_CHIP_IDS:
                return chip_id
            sleep_ms(1)
        raise OSError("No BME280/BMP280 at 0x{:02x} (chip ID 0x{:02x})".format(
            self.address, chip_id))

    def _read_status(self) -> int:
        return self.i2c.readfrom_mem(self.address, _BME280_STATUS_ADDR, 1)[0]

    def _soft_reset(self):
        """
        Write the reset command, then wait until the sensor has copied its
        calibration from NVM (status im_update clears), as the reference
        library does.
        """
        self.i2c.writeto_mem(self.address, _BME280_RESET_ADDR, bytearray([0xB6]))
        sleep_ms(2)
        for _ in range(_NVM_COPY_TIMEOUT_MS):
            if not self._read_status() & _STATUS_IM_UPDATE:
                return
            sleep_ms(1)
        raise OSError("BME280 NVM copy did not finish after reset")

    def _load_calibration_data(self):
        """
        Load the read-only calibration values out of the sensor's memory, to be
        used later in calibrating the raw reads.  These get stored in various
        self.cal_dig_* object properties.

        See https://github.com/BoschSensortec/BME280_driver/blob/bme280_v3.3.4/bme280.c#L1192
        See https://github.com/BoschSensortec/BME280_driver/blob/bme280_v3.3.4/bme280.c#L1216
        See https://github.com/catdog2/mpy_bme280_esp8266/blob/master/bme280.py#L73
        """
        # Load the temperature and pressure calibration data
        # (note that the first value of the humidity data is stuffed in here)
        tp_cal_mem = self.i2c.readfrom_mem(self.address,
                                           _BME280_TEMP_PRESS_CALIB_DATA_ADDR,
                                           _BME280_TEMP_PRESS_CALIB_DATA_LEN)

        (self.cal_dig_T1, self.cal_dig_T2, self.cal_dig_T3,
         self.cal_dig_P1, self.cal_dig_P2, self.cal_dig_P3,
         self.cal_dig_P4, self.cal_dig_P5, self.cal_dig_P6,
         self.cal_dig_P7, self.cal_dig_P8, self.cal_dig_P9,
         _,
         self.cal_dig_H1) = unpack("<HhhHhhhhhhhhBB", tp_cal_mem)

        # Initialize the cal_t_fine carry-over value used during compensation
        self.cal_t_fine = 0

        if not self.has_humidity:
            self.cal_dig_H1 = self.cal_dig_H2 = self.cal_dig_H3 = 0
            self.cal_dig_H4 = self.cal_dig_H5 = self.cal_dig_H6 = 0
            return

        # Load the rest of the humidity calibration data
        hum_cal_mem = self.i2c.readfrom_mem(self.address,
                                            _BME280_HUMIDITY_CALIB_DATA_ADDR,
                                            _BME280_HUMIDITY_CALIB_DATA_LEN)

        self.cal_dig_H2, self.cal_dig_H3 = unpack_from("<hB", hum_cal_mem)

        # H4 and H5 are signed 12-bit values that share register 0xE5.
        e4_sign = unpack_from("<b", hum_cal_mem, 3)[0]
        self.cal_dig_H4 = (e4_sign << 4) | (hum_cal_mem[4] & 0b00001111)

        e6_sign = unpack_from("<b", hum_cal_mem, 5)[0]
        self.cal_dig_H5 = (e6_sign << 4) | (hum_cal_mem[4] >> 4)

        self.cal_dig_H6 = unpack_from("<b", hum_cal_mem, 6)[0]

    def get_measurement_settings(self):
        """
        Return a parsed set of the sensor's measurement settings as a dict
        These values include oversampling settings for each measurement,
        the IIR filter coefficient, and the standby duration for normal
        power mode.
        See the data sheet, section 3 and 5
        """
        mem = self.i2c.readfrom_mem(self.address, _BME280_CTRL_HUM_ADDR, 4)
        ctrl_hum, _, ctrl_meas, config = unpack("<BBBB", mem)

        return {
            "osr_h": (ctrl_hum & 0b00000111),
            "osr_p": (ctrl_meas >> 2) & 0b00000111,
            "osr_t": (ctrl_meas >> 5) & 0b00000111,
            "filter": (config >> 2) & 0b00000111,
            "standby_time": (config >> 5) & 0b00000111,
        }

    def set_measurement_settings(self, settings: dict):
        """
        Set the sensor's settings for each measurement's oversampling,
        the pressure IIR filter coefficient, and standby duration
        during normal power mode.

        The settings dict can have keys osr_h, osr_p, osr_t, filter, and
        standby_time.  All values are optional, and omitting any will retain
        the pre-existing value.

        The sensor is put to sleep while the registers are written (the
        config register is ignored in normal mode), and normal mode is
        restored afterwards if it was running.

        See the data sheet, section 3 and 5
        """
        self._validate_settings(settings)
        was_running = self.get_power_mode() == BME280_NORMAL_MODE
        self._enter_sleep_mode()
        self._write_measurement_settings(settings)
        if was_running:
            self._write_power_mode(BME280_NORMAL_MODE)

    def _validate_settings(self, settings: dict):
        oversampling_options = [
            BME280_NO_OVERSAMPLING, BME280_OVERSAMPLING_1X,
            BME280_OVERSAMPLING_2X, BME280_OVERSAMPLING_4X,
            BME280_OVERSAMPLING_8X, BME280_OVERSAMPLING_16X]

        filter_options = [
            BME280_FILTER_COEFF_OFF, BME280_FILTER_COEFF_2,
            BME280_FILTER_COEFF_4, BME280_FILTER_COEFF_8,
            BME280_FILTER_COEFF_16]

        standby_time_options = [
            BME280_STANDBY_TIME_500_US,
            BME280_STANDBY_TIME_62_5_MS, BME280_STANDBY_TIME_125_MS,
            BME280_STANDBY_TIME_250_MS, BME280_STANDBY_TIME_500_MS,
            BME280_STANDBY_TIME_1000_MS, BME280_STANDBY_TIME_10_MS,
            BME280_STANDBY_TIME_20_MS]

        for key in ('osr_h', 'osr_p', 'osr_t'):
            if key in settings and settings[key] not in oversampling_options:
                raise ValueError("{} must be one of the oversampling defines".format(key))
        if 'filter' in settings:
            if settings['filter'] not in filter_options:
                raise ValueError("filter must be one of the filter coefficient defines")
        if 'standby_time' in settings:
            if settings['standby_time'] not in standby_time_options:
                raise ValueError("standby_time must be one of the standby time duration defines")

    def _write_measurement_settings(self, settings: dict):
        # Read in the existing configuration, to modify
        mem = self.i2c.readfrom_mem(self.address, _BME280_CTRL_HUM_ADDR, 4)
        ctrl_hum, _, ctrl_meas, config = unpack("<BBBB", mem)

        # Make any changes necessary to the ctrl_hum register (BME280 only)
        if "osr_h" in settings and self.has_humidity:
            newval = (ctrl_hum & 0b11111000) | (settings['osr_h'] & 0b00000111)
            self.i2c.writeto_mem(self.address, _BME280_CTRL_HUM_ADDR, bytearray([newval]))

            # according to the data sheet, ctrl_hum needs a write to
            # ctrl_meas in order to take effect
            self.i2c.writeto_mem(self.address, _BME280_CTRL_MEAS_ADDR, bytearray([ctrl_meas]))

        # Make any changes necessary to the ctrl_meas register
        if "osr_p" in settings or "osr_t" in settings:
            newval = ctrl_meas
            if "osr_p" in settings:
                newval = (newval & 0b11100011) | ((settings['osr_p'] << 2) & 0b00011100)
            if "osr_t" in settings:
                newval = (newval & 0b00011111) | ((settings['osr_t'] << 5) & 0b11100000)
            self.i2c.writeto_mem(self.address, _BME280_CTRL_MEAS_ADDR, bytearray([newval]))

        # Make any changes necessary to the config register
        if "filter" in settings or "standby_time" in settings:
            newval = config
            if "filter" in settings:
                newval = (newval & 0b11100011) | ((settings['filter'] << 2) & 0b00011100)
            if "standby_time" in settings:
                newval = (newval & 0b00011111) | ((settings['standby_time'] << 5) & 0b11100000)
            self.i2c.writeto_mem(self.address, _BME280_CONFIG_ADDR, bytearray([newval]))

    def get_power_mode(self):
        """
        Result will be one of BME280_SLEEP_MODE, BME280_FORCED_MODE, or
        BME280_NORMAL_MODE.
        See the data sheet, section 3.3
        """
        mem = self.i2c.readfrom_mem(self.address, _BME280_PWR_CTRL_ADDR, 1)
        return (mem[0] & 0b00000011)

    def set_power_mode(self, new_power_mode: int):
        """
        Configure the sensor's power mode (BME280_SLEEP_MODE,
        BME280_FORCED_MODE, or BME280_NORMAL_MODE)

        Note that setting to forced mode will immediately set the sensor back
        to sleep mode after taking a measurement.

        See the data sheet, section 3.3
        """
        if new_power_mode not in [BME280_SLEEP_MODE, BME280_FORCED_MODE, BME280_NORMAL_MODE]:
            raise ValueError("New power mode must be sleep, forced, or normal constant")

        self._enter_sleep_mode()
        self._write_power_mode(new_power_mode)

    def _write_power_mode(self, power_mode: int):
        # Read the current register, mask out and set the new power mode,
        # and write the register back to the device.
        mem = self.i2c.readfrom_mem(self.address, _BME280_PWR_CTRL_ADDR, 1)
        newval = (mem[0] & 0b11111100) | (power_mode & 0b00000011)
        self.i2c.writeto_mem(self.address, _BME280_PWR_CTRL_ADDR, bytearray([newval]))

    def _enter_sleep_mode(self):
        """
        If the sensor mode isn't already "sleep", put it to sleep by clearing
        the mode bits in ctrl_meas. Oversampling, filter and standby settings
        are kept.
        """
        if self.get_power_mode() != BME280_SLEEP_MODE:
            self._write_power_mode(BME280_SLEEP_MODE)

    def _wait_for_measurement(self):
        """
        Wait until no conversion is running (status measuring bit clear), so
        the data registers hold a finished measurement. Raises OSError if a
        conversion runs longer than the longest one the datasheet allows.
        """
        for _ in range(_MEASUREMENT_TIMEOUT_MS):
            if not self._read_status() & _STATUS_MEASURING:
                return
            sleep_ms(1)
        raise OSError("BME280 measurement did not finish")

    def get_measurement(self):
        """
        Return a dict of temperature (DegC), pressure (Pa) and humidity
        (%RH), compensated with the sensor's stored calibration data.

        Waits for any running conversion to finish first. A value is None
        when the sensor has no measurement for it: nothing measured yet
        since reset, that channel's oversampling set to skipped, or no
        humidity sensor (BMP280). Pressure and humidity need the
        temperature, so they are None whenever temperature is.
        """
        self._wait_for_measurement()
        raw = self._read_uncompensated_data()

        if raw['temperature'] == _SKIPPED_TP:
            return dict(_NO_READING)

        # Temperature first: it sets cal_t_fine, used by the other two.
        temperature = self._compensate_temperature(raw['temperature'])

        pressure = None
        if raw['pressure'] != _SKIPPED_TP:
            pressure = self._compensate_pressure(raw['pressure'])

        humidity = None
        if self.has_humidity and raw['humidity'] != _SKIPPED_H:
            humidity = self._compensate_humidity(raw['humidity'])

        return {
            "temperature": temperature,
            "pressure": pressure,
            "humidity": humidity,
        }

    def _read_uncompensated_data(self):
        # Read the uncompensated temperature, pressure, and humidity data in
        # one burst, so all three come from the same measurement.
        mem = self.i2c.readfrom_mem(self.address, _BME280_DATA_ADDR, _BME280_P_T_H_DATA_LEN)
        (press_msb, press_lsb, press_xlsb,
         temp_msb, temp_lsb, temp_xlsb,
         hum_msb, hum_lsb) = unpack("<BBBBBBBB", mem)

        return {
            "temperature": (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4),
            "pressure": (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4),
            "humidity": (hum_msb << 8) | (hum_lsb),
        }

    ##
    # Float Implementations
    ##

    # def _compensate_temperature(self, adc_T: int) -> float:
    #     """
    #     Output value of “25.0” equals 25.0 DegC.
    #
    #     See the floating-point implementation in the reference library:
    #     https://github.com/BoschSensortec/BME280_driver/blob/bme280_v3.3.4/bme280.c#L884
    #     """
    #     temperature_min = -40
    #     temperature_max = 85
    #
    #     var1 = (adc_T / 16384.0) - (self.cal_dig_T1 / 1024.0)
    #     var1 = var1 * self.cal_dig_T2
    #
    #     var2 = (adc_T / 131072.0) - (self.cal_dig_T1 / 8192.0)
    #     var2 = var2 * var2 * self.cal_dig_T3
    #
    #     self.cal_t_fine = int(var1 + var2)
    #
    #     temperature = (var1 + var2) / 5120.0
    #
    #     if temperature < temperature_min:
    #         temperature = temperature_min
    #     elif temperature > temperature_max:
    #         temperature = temperature_max
    #
    #     return temperature

    # def _compensate_pressure(self, adc_P: int) -> float:
    #     """
    #     Output value of “96386.0” equals 96386 Pa = 963.86 hPa
    #
    #     See the floating-point implementation in the reference library:
    #     https://github.com/BoschSensortec/BME280_driver/blob/bme280_v3.3.4/bme280.c#L912
    #     """
    #     pressure_min = 30000.0
    #     pressure_max = 110000.0
    #
    #     var1 = (self.cal_t_fine / 2.0) - 64000.0
    #
    #     var2 = var1 * var1 * self.cal_dig_P6 / 32768.0
    #     var2 = var2 + (var1 * self.cal_dig_P5 * 2.0)
    #     var2 = (var2 / 4.0) + (self.cal_dig_P4 * 65536.0)
    #
    #     var3 = self.cal_dig_P3 * var1 * var1 / 524288.0
    #
    #     var1 = (var3 + self.cal_dig_P2 * var1) / 524288.0
    #     var1 = (1.0 + var1 / 32768.0) * self.cal_dig_P1
    #
    #     # avoid exception caused by division by zero
    #     if var1:
    #         pressure = 1048576.0 - adc_P
    #         pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1
    #         var1 = self.cal_dig_P9 * pressure * pressure / 2147483648.0
    #         var2 = pressure * self.cal_dig_P8 / 32768.0
    #         pressure = pressure + (var1 + var2 + self.cal_dig_P7) / 16.0
    #
    #         if pressure < pressure_min:
    #             pressure = pressure_min
    #         elif pressure > pressure_max:
    #             pressure = pressure_max
    #
    #     else:
    #         # Invalid case
    #         pressure = pressure_min
    #
    #     return pressure

    # def _compensate_humidity(self, adc_H: int) -> float:
    #     """
    #     Output value between 0.0 and 100.0, where 100.0 is 100%RH
    #
    #     See the floating-point implementation in the reference library:
    #     https://github.com/BoschSensortec/BME280_driver/blob/bme280_v3.3.4/bme280.c#L952
    #     """
    #     humidity_min = 0.0
    #     humidity_max = 100.0
    #
    #     var1 = self.cal_t_fine - 76800.0
    #
    #     var2 = self.cal_dig_H4 * 64.0 + (self.cal_dig_H5 / 16384.0) * var1
    #
    #     var3 = adc_H - var2
    #
    #     var4 = self.cal_dig_H2 / 65536.0
    #
    #     var5 = 1.0 + (self.cal_dig_H3 / 67108864.0) * var1
    #
    #     var6 = 1.0 + (self.cal_dig_H6 / 67108864.0) * var1 * var5
    #     var6 = var3 * var4 * (var5 * var6)
    #
    #     humidity = var6 * (1.0 - self.cal_dig_H1 * var6 / 524288.0)
    #
    #     if humidity > humidity_max:
    #         humidity = humidity_max
    #     elif humidity < humidity_min:
    #         humidity = humidity_min
    #
    #     return humidity

    ##
    # Integer Implementations (datasheet section 4.2.3 / 8.1)
    #
    # Python integers don't overflow, so these follow the reference C code
    # without its 32-bit wraparound. Floor division (//) on negative numbers
    # matches C's arithmetic right shift.
    ##

    def _compensate_temperature(self, adc_T: int) -> float:
        """
        Output value of "25.0" equals 25.0 DegC. Not clamped: a value outside
        the -40..85 DegC operating range is returned as computed.

        See the integer implementation in the data sheet, section 4.2.3
        And the reference library:
        https://github.com/BoschSensortec/BME280_driver/blob/bme280_v3.3.4/bme280.c#L987
        """
        var1 = (((adc_T // 8) - (self.cal_dig_T1 * 2)) * self.cal_dig_T2) // 2048

        var2 = (((((adc_T // 16) - self.cal_dig_T1) * (
                (adc_T // 16) - self.cal_dig_T1)) // 4096) * self.cal_dig_T3) // 16384

        self.cal_t_fine = var1 + var2

        temperature = (self.cal_t_fine * 5 + 128) // 256

        return temperature / 100

    def _compensate_pressure(self, adc_P: int):
        """
        Output value of "96386.2" equals 96386.2 Pa = 963.862 hPa, with
        1/256 Pa resolution. Returns None if the calibration data is invalid
        (it would divide by zero). Not clamped.

        This is the 64-bit integer implementation from the data sheet,
        section 4.2.3, and the reference library:
        https://github.com/BoschSensortec/BME280_driver/blob/bme280_v3.3.4/bme280.c#L1016
        """
        var1 = self.cal_t_fine - 128000
        var2 = var1 * var1 * self.cal_dig_P6
        var2 = var2 + ((var1 * self.cal_dig_P5) << 17)
        var2 = var2 + (self.cal_dig_P4 << 35)
        var1 = ((var1 * var1 * self.cal_dig_P3) >> 8) + ((var1 * self.cal_dig_P2) << 12)
        var1 = (((1 << 47) + var1) * self.cal_dig_P1) >> 33

        # avoid exception caused by division by zero
        if var1 == 0:
            return None

        pressure = 1048576 - adc_P
        numerator = ((pressure << 31) - var2) * 3125
        # C division truncates toward zero; floor division would not for
        # negative values.
        pressure = abs(numerator) // abs(var1)
        if (numerator < 0) != (var1 < 0):
            pressure = -pressure

        var1 = (self.cal_dig_P9 * (pressure >> 13) * (pressure >> 13)) >> 25
        var2 = (self.cal_dig_P8 * pressure) >> 19

        pressure = ((pressure + var1 + var2) >> 8) + (self.cal_dig_P7 << 4)

        return pressure / 256

    def _compensate_humidity(self, adc_H: int) -> float:
        """
        Output value between 0.0 and 100.0, where 100.0 is 100%RH

        See the 32-bit integer implementation in the data sheet, section 4.2.3
        And the reference library:
        https://github.com/BoschSensortec/BME280_driver/blob/bme280_v3.3.4/bme280.c#L1108
        """
        humidity_max = 102400

        var1 = self.cal_t_fine - 76800

        var2 = adc_H * 16384

        var3 = self.cal_dig_H4 * 1048576

        var4 = self.cal_dig_H5 * var1

        var5 = (((var2 - var3) - var4) + 16384) // 32768

        var2 = (var1 * self.cal_dig_H6) // 1024

        var3 = (var1 * self.cal_dig_H3) // 2048

        var4 = ((var2 * (var3 + 32768)) // 1024) + 2097152

        var2 = ((var4 * self.cal_dig_H2) + 8192) // 16384

        var3 = var5 * var2

        var4 = ((var3 // 32768) * (var3 // 32768)) // 128

        var5 = var3 - ((var4 * self.cal_dig_H1) // 16)
        if var5 < 0:
            var5 = 0
        if var5 > 419430400:
            var5 = 419430400

        humidity = var5 // 4096

        if (humidity > humidity_max):
            humidity = humidity_max

        return humidity / 1024
