# BME280
Micropython code reads BME280 sensor (also works with a BMP280, without humidity).

## Usage

```python
import machine
import bme280_i2c

i2c = machine.SoftI2C(scl=machine.Pin(5), sda=machine.Pin(4))  # machine.I2C on old firmware
sensor = bme280_i2c.BME280_I2C(address=bme280_i2c.BME280_I2C_ADDR_PRIM, i2c=i2c)
sensor.set_measurement_settings({'osr_t': bme280_i2c.BME280_OVERSAMPLING_2X,
                                 'osr_p': bme280_i2c.BME280_OVERSAMPLING_16X,
                                 'osr_h': bme280_i2c.BME280_OVERSAMPLING_1X})
sensor.set_power_mode(bme280_i2c.BME280_NORMAL_MODE)
print(sensor.get_measurement())
# {'temperature': 25.08, 'pressure': 100653.25, 'humidity': 38.27}
```

Temperature is in °C, pressure in Pa and humidity in %RH. A value is `None`
when the sensor has nothing measured for it yet, that channel is set to
`BME280_NO_OVERSAMPLING`, or the chip is a BMP280 (humidity).

## Tests

The driver runs on desktop Python against a fake I2C bus:

```
pip install pytest
python -m pytest tests
```
