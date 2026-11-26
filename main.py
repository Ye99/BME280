# Thanks to https://github.com/triplepoint/micropython_bme280_i2c

import time

import machine

import bme280_i2c


def make_i2c(scl_pin=5, sda_pin=4):
    """
    Create the I2C bus on the sensor's pins. Newer MicroPython firmware
    wants SoftI2C for a bit-banged bus; older firmware (such as the v1.12
    image reflashMcu.sh installs) only has machine.I2C.
    """
    i2c_class = getattr(machine, "SoftI2C", None) or machine.I2C
    return i2c_class(scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin))


def _fmt(value, unit):
    return "n/a" if value is None else "{} {}".format(value, unit)


def run(sensor, iterations=None, sleep_ms=None, out=print):
    """
    Print one measurement per loop. All three values come from the same
    get_measurement() call, so they belong to the same sample.
    """
    if sleep_ms is None:
        sleep_ms = time.sleep_ms
    count = 0
    while iterations is None or count < iterations:
        reading = sensor.get_measurement()
        out('Temperature: {}'.format(_fmt(reading["temperature"], "C")))
        out('Humidity: {}'.format(_fmt(reading["humidity"], "%RH")))
        out('Pressure: {}'.format(_fmt(reading["pressure"], "Pa")))
        out('========================')
        sleep_ms(100)
        count += 1


def main():
    # Raises OSError if no BME280/BMP280 answers at the address.
    sensor = bme280_i2c.BME280_I2C(address=bme280_i2c.BME280_I2C_ADDR_PRIM, i2c=make_i2c())

    # Configure the sensor for the application in question.
    sensor.set_measurement_settings({
        'filter': bme280_i2c.BME280_FILTER_COEFF_16,
        'standby_time': bme280_i2c.BME280_STANDBY_TIME_500_US,
        'osr_h': bme280_i2c.BME280_OVERSAMPLING_1X,
        'osr_p': bme280_i2c.BME280_OVERSAMPLING_16X,
        'osr_t': bme280_i2c.BME280_OVERSAMPLING_2X})

    # Start the sensor automatically sensing
    sensor.set_power_mode(bme280_i2c.BME280_NORMAL_MODE)

    # Prints lines like:
    # Temperature: 27.86 C
    # Humidity: 39.5 %RH
    # Pressure: 101412.25 Pa
    run(sensor)


if __name__ == '__main__':
    main()
