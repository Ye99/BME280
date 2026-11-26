import hashlib
import os
import shutil
import stat
import subprocess
import sys
import types

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# --- main.py ---------------------------------------------------------------------

class FakePin:
    def __init__(self, n):
        self.n = n


def fake_machine(with_soft_i2c):
    mod = types.ModuleType("machine")
    mod.Pin = FakePin
    mod.created = []

    class HardI2C:
        def __init__(self, *args, **kw):
            mod.created.append(("I2C", args, kw))

    mod.I2C = HardI2C
    if with_soft_i2c:
        class SoftI2C:
            def __init__(self, *args, **kw):
                mod.created.append(("SoftI2C", args, kw))
        mod.SoftI2C = SoftI2C
    return mod


@pytest.fixture
def main_module(monkeypatch):
    def load(with_soft_i2c=True):
        monkeypatch.setitem(sys.modules, "machine", fake_machine(with_soft_i2c))
        sys.modules.pop("main", None)
        import main
        return main
    yield load
    sys.modules.pop("main", None)


def test_importing_main_does_not_start_the_loop(main_module):
    main = main_module()
    assert sys.modules["machine"].created == []
    assert callable(main.run)


def test_uses_soft_i2c_when_firmware_has_it(main_module):
    main = main_module(with_soft_i2c=True)
    main.make_i2c()
    kind, args, kw = sys.modules["machine"].created[0]
    assert kind == "SoftI2C"
    assert (kw["scl"].n, kw["sda"].n) == (5, 4)


def test_falls_back_to_i2c_on_old_firmware(main_module):
    main = main_module(with_soft_i2c=False)
    main.make_i2c()
    assert sys.modules["machine"].created[0][0] == "I2C"


class CountingSensor:
    def __init__(self, readings):
        self.readings = list(readings)
        self.calls = 0

    def get_measurement(self):
        self.calls += 1
        return self.readings[self.calls - 1]


def test_one_measurement_per_loop_iteration(main_module):
    main = main_module()
    sensor = CountingSensor([
        {"temperature": 25.08, "pressure": 100653.27, "humidity": 38.27},
        {"temperature": 25.1, "pressure": 100653.5, "humidity": 38.3},
    ])
    lines = []
    main.run(sensor, iterations=2, sleep_ms=lambda ms: None, out=lines.append)
    assert sensor.calls == 2
    text = "\n".join(lines)
    assert "25.08" in text and "100653.27" in text and "38.27" in text


def test_missing_values_are_printed_as_not_available(main_module):
    main = main_module()
    sensor = CountingSensor([{"temperature": None, "pressure": None, "humidity": None}])
    lines = []
    main.run(sensor, iterations=1, sleep_ms=lambda ms: None, out=lines.append)
    assert "n/a" in "\n".join(lines)


# --- boot.py ---------------------------------------------------------------------

def test_boot_does_not_start_webrepl_by_default(monkeypatch):
    started = []
    webrepl = types.ModuleType("webrepl")
    webrepl.start = lambda *a, **k: started.append(True)
    for name in ("uos", "machine"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "webrepl", webrepl)
    with open(os.path.join(ROOT, "boot.py")) as f:
        exec(compile(f.read(), "boot.py", "exec"), {"__name__": "__main__"})
    assert started == []


# --- reflashMcu.sh ---------------------------------------------------------------

FIRMWARE = b"fake esp8266 firmware image"


def make_tools(bindir, log):
    def tool(name, body):
        path = os.path.join(bindir, name)
        with open(path, "w") as f:
            f.write("#!/bin/bash\n" + body)
        os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)

    # curl: record args, write FIRMWARE to the -o target
    tool("curl", 'echo "curl $*" >> "%s"\n'
                 'while [ $# -gt 0 ]; do if [ "$1" = "-o" ]; then out="$2"; fi; shift; done\n'
                 'printf "%%s" "%s" > "$out"\n' % (log, FIRMWARE.decode()))
    tool("esptool.py", 'echo "esptool.py $*" >> "%s"\n' % log)
    tool("rshell", 'echo "rshell $*" >> "%s"\n' % log)


def run_reflash(tmp_path, sha=None):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    log = tmp_path / "calls.log"
    log.write_text("")
    make_tools(str(bindir), str(log))
    work = tmp_path / "work"
    work.mkdir()
    shutil.copy(os.path.join(ROOT, "reflashMcu.sh"), work)
    env = dict(os.environ, PATH=str(bindir) + os.pathsep + os.environ["PATH"])
    env.pop("FIRMWARE_SHA256", None)
    if sha is not None:
        env["FIRMWARE_SHA256"] = sha
    proc = subprocess.run(["bash", "reflashMcu.sh"], cwd=work, env=env,
                          capture_output=True, text=True)
    return proc, log.read_text()


def test_reflash_downloads_over_https(tmp_path):
    _, calls = run_reflash(tmp_path)
    curl = [l for l in calls.splitlines() if l.startswith("curl")][0]
    assert "https://" in curl and "http://" not in curl


def test_reflash_refuses_without_a_pinned_checksum(tmp_path):
    proc, calls = run_reflash(tmp_path)
    assert proc.returncode != 0
    assert "esptool.py" not in calls
    assert hashlib.sha256(FIRMWARE).hexdigest() in proc.stdout + proc.stderr


def test_reflash_refuses_on_checksum_mismatch(tmp_path):
    proc, calls = run_reflash(tmp_path, sha="0" * 64)
    assert proc.returncode != 0
    assert "esptool.py" not in calls


def test_reflash_flashes_when_checksum_matches(tmp_path):
    proc, calls = run_reflash(tmp_path, sha=hashlib.sha256(FIRMWARE).hexdigest())
    assert proc.returncode == 0, proc.stderr
    assert "erase_flash" in calls and "write_flash" in calls
