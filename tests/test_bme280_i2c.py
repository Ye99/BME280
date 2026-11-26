import pytest

import bme280_i2c as m
from fake_bus import FakeBME280, BME280_ID, BMP280_ID, EXAMPLE_TP, EXAMPLE_H

# Raw readings from the Bosch BMP280 datasheet worked example.
ADC_T = 519888
ADC_P = 415148
ADC_H = 27000


def make(bus=None, **kw):
    bus = bus or FakeBME280(**kw)
    return m.BME280_I2C(address=bus.address, i2c=bus), bus


def float_humidity(t_fine, adc_h, h):
    """Bosch reference floating-point humidity formula."""
    var1 = t_fine - 76800.0
    var2 = h["H4"] * 64.0 + (h["H5"] / 16384.0) * var1
    var3 = adc_h - var2
    var4 = h["H2"] / 65536.0
    var5 = 1.0 + (h["H3"] / 67108864.0) * var1
    var6 = 1.0 + (h["H6"] / 67108864.0) * var1 * var5
    var6 = var3 * var4 * (var5 * var6)
    return max(0.0, min(100.0, var6 * (1.0 - h["H1"] * var6 / 524288.0)))


def float_temperature(adc_t, tp):
    var1 = (adc_t / 16384.0 - tp["T1"] / 1024.0) * tp["T2"]
    var2 = (adc_t / 131072.0 - tp["T1"] / 8192.0) ** 2 * tp["T3"]
    return (var1 + var2) / 5120.0


def start_normal(sensor, bus, raw=(ADC_T, ADC_P, ADC_H)):
    bus.raw = raw
    sensor.set_measurement_settings({"osr_h": 1, "osr_p": 5, "osr_t": 2})
    sensor.set_power_mode(m.BME280_NORMAL_MODE)


# --- Compensation math (regression guards against the datasheet) -----------

def test_temperature_matches_datasheet_example():
    sensor, _ = make()
    assert sensor._compensate_temperature(ADC_T) == 25.08
    assert sensor.cal_t_fine == 128422


def test_pressure_uses_64bit_formula_with_sub_pascal_resolution():
    sensor, _ = make()
    sensor._compensate_temperature(ADC_T)
    p = sensor._compensate_pressure(ADC_P)
    assert isinstance(p, float)
    # Datasheet: int64 result 25767236 / 256 = 100653.27 Pa (int32 gave 100656).
    assert p == pytest.approx(100653.27, abs=0.05)


def test_humidity_matches_bosch_float_formula():
    sensor, _ = make()
    sensor._compensate_temperature(ADC_T)
    for adc_h in (24000, 27000, 32000):
        assert sensor._compensate_humidity(adc_h) == pytest.approx(
            float_humidity(sensor.cal_t_fine, adc_h, EXAMPLE_H), abs=0.01)


def test_negative_h4_h5_calibration_is_sign_extended():
    h = dict(EXAMPLE_H, H4=-300, H5=-45)
    sensor, _ = make(h=h)
    assert (sensor.cal_dig_H4, sensor.cal_dig_H5) == (-300, -45)


def test_calibration_loads_on_desktop_python():
    sensor, _ = make()
    assert (sensor.cal_dig_T1, sensor.cal_dig_P9, sensor.cal_dig_H2,
            sensor.cal_dig_H6) == (27504, 6000, 362, 30)


# --- Finding 1: settings validation -------------------------------------------

@pytest.mark.parametrize("key", ["osr_p", "osr_t"])
def test_single_oversampling_setting_is_accepted(key):
    sensor, bus = make()
    sensor.set_measurement_settings({key: m.BME280_OVERSAMPLING_16X})
    assert sensor.get_measurement_settings()[key] == m.BME280_OVERSAMPLING_16X


@pytest.mark.parametrize("key", ["osr_h", "osr_p", "osr_t", "filter", "standby_time"])
def test_invalid_setting_is_rejected_and_not_written(key):
    sensor, bus = make()
    before = len(bus.writes)
    with pytest.raises(ValueError, match=key):
        sensor.set_measurement_settings({"osr_h": 1, key: 8})
    assert len(bus.writes) == before


# --- Finding 2: changing settings must not stop a running sensor ----------------

def test_settings_change_in_normal_mode_keeps_sensor_running():
    sensor, bus = make()
    start_normal(sensor, bus)
    sensor.set_measurement_settings({"filter": m.BME280_FILTER_COEFF_16})
    assert sensor.get_power_mode() == m.BME280_NORMAL_MODE
    assert sensor.get_measurement_settings()["filter"] == m.BME280_FILTER_COEFF_16
    assert sensor.get_measurement_settings()["osr_p"] == 5


def test_settings_change_sleeps_without_soft_reset_and_writes_config_while_asleep():
    sensor, bus = make()
    start_normal(sensor, bus)
    bus.writes.clear()
    sensor.set_measurement_settings({"filter": m.BME280_FILTER_COEFF_4})
    assert not any(reg == 0xE0 for reg, _ in bus.writes)
    regs = [(reg, val[0]) for reg, val in bus.writes]
    sleep_at = next(i for i, (r, v) in enumerate(regs) if r == 0xF4 and v & 3 == 0)
    config_at = next(i for i, (r, _) in enumerate(regs) if r == 0xF5)
    assert sleep_at < config_at


# --- Finding 3: no reading before a measurement exists -------------------------

def test_get_measurement_waits_for_first_conversion_in_normal_mode():
    sensor, bus = make(measuring_reads=3)
    start_normal(sensor, bus)
    reading = sensor.get_measurement()
    assert reading["temperature"] == 25.08
    assert reading["pressure"] == pytest.approx(100653.27, abs=0.05)
    assert reading["humidity"] is not None


def test_get_measurement_waits_for_forced_conversion():
    sensor, bus = make(measuring_reads=3)
    bus.raw = (ADC_T, ADC_P, ADC_H)
    sensor.set_measurement_settings({"osr_h": 1, "osr_p": 1, "osr_t": 1})
    sensor.set_power_mode(m.BME280_FORCED_MODE)
    assert sensor.get_measurement()["temperature"] == 25.08


def test_get_measurement_times_out_if_conversion_never_finishes():
    sensor, bus = make(measuring_reads=10 ** 9)
    start_normal(sensor, bus)
    with pytest.raises(OSError):
        sensor.get_measurement()


def test_no_measurement_yet_returns_none_not_plausible_numbers():
    sensor, _ = make()
    assert sensor.get_measurement() == {
        "temperature": None, "pressure": None, "humidity": None}


def test_skipped_temperature_invalidates_everything():
    sensor, bus = make()
    bus.raw = (ADC_T, ADC_P, ADC_H)
    sensor.set_measurement_settings({"osr_h": 1, "osr_p": 1, "osr_t": 0})
    sensor.set_power_mode(m.BME280_FORCED_MODE)
    assert sensor.get_measurement() == {
        "temperature": None, "pressure": None, "humidity": None}


def test_skipped_pressure_and_humidity_are_none():
    sensor, bus = make()
    bus.raw = (ADC_T, ADC_P, ADC_H)
    sensor.set_measurement_settings({"osr_h": 0, "osr_p": 0, "osr_t": 1})
    sensor.set_power_mode(m.BME280_FORCED_MODE)
    reading = sensor.get_measurement()
    assert reading["temperature"] == 25.08
    assert reading["pressure"] is None
    assert reading["humidity"] is None


# --- Finding 4: chip ID -------------------------------------------------------------

def test_unknown_chip_is_rejected_before_any_write():
    bus = FakeBME280(chip_id=0x42)
    with pytest.raises(OSError):
        m.BME280_I2C(address=bus.address, i2c=bus)
    assert bus.writes == []


def test_bmp280_is_supported_without_humidity():
    sensor, bus = make(chip_id=BMP280_ID)
    assert sensor.has_humidity is False
    start_normal(sensor, bus)
    reading = sensor.get_measurement()
    assert reading["temperature"] == 25.08
    assert reading["humidity"] is None


def test_bme280_reports_humidity():
    sensor, _ = make(chip_id=BME280_ID)
    assert sensor.has_humidity is True


# --- Finding 5: wait for NVM copy after reset ------------------------------------

def test_calibration_is_read_after_nvm_copy_finishes():
    sensor, _ = make(im_update_reads=3)
    assert sensor.cal_dig_T1 == EXAMPLE_TP["T1"]
    assert sensor.cal_dig_H2 == EXAMPLE_H["H2"]


def test_reset_times_out_if_nvm_copy_never_finishes():
    bus = FakeBME280(im_update_reads=10 ** 9)
    with pytest.raises(OSError):
        m.BME280_I2C(address=bus.address, i2c=bus)


# --- Cleanups: no silent clamping ---------------------------------------------------

def test_temperature_is_not_clamped_to_85():
    sensor, _ = make()
    adc_t = 750000
    expected = float_temperature(adc_t, EXAMPLE_TP)
    assert expected > 85
    assert sensor._compensate_temperature(adc_t) == pytest.approx(expected, abs=0.02)


def test_invalid_pressure_calibration_returns_none_not_300_hpa():
    sensor, _ = make(tp=dict(EXAMPLE_TP, P1=0))
    sensor._compensate_temperature(ADC_T)
    assert sensor._compensate_pressure(ADC_P) is None
