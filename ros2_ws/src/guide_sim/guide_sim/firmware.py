"""The car's real Arduino firmware, compiled for the PC and stepped in virtual time.

``Firmware(ino)`` compiles ``guide_motor.ino`` against the host Arduino core in
``arduino/`` (g++, cached by source hash and defines) and loads it with
ctypes. The simulator then plays the board's world: it advances time, calls
``loop()``, feeds serial bytes in, reads serial bytes and pin states out, and
sets the button and ultrasonic echo. The firmware source is not modified.
"""

from __future__ import annotations

import ctypes
import hashlib
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent / "arduino"
CACHE = Path(os.environ.get("GUIDE_SIM_CACHE", Path.home() / ".cache" / "guide_sim"))


def default_ino() -> Path:
    """The firmware in the repository (run from its root, or set GUIDE_CAR_ROOT)."""
    root = Path(os.environ.get("GUIDE_CAR_ROOT", os.getcwd()))
    return root / "ros2_ws" / "src" / "guide_bridge" / "firmware" / "guide_motor" / "guide_motor.ino"


def build(ino: Path, defines: tuple[str, ...] = ()) -> Path:
    """Compile ``ino`` into a shared library; returns its path (cached)."""
    ino = Path(ino).resolve()
    sources = [HERE / "Arduino.h", HERE / "Servo.h", HERE / "arduino_host.cpp", HERE / "firmware_main.cpp", ino]
    h = hashlib.sha256()
    for s in sources:
        h.update(s.read_bytes())
    h.update(" ".join(defines).encode())
    out = CACHE / f"guide_motor_{h.hexdigest()[:16]}.so"
    if out.exists():
        return out
    CACHE.mkdir(parents=True, exist_ok=True)
    cmd = ["g++", "-std=c++17", "-O2", "-shared", "-fPIC", "-Wall", "-Wno-unused-function",
           f"-I{HERE}", f"-DFIRMWARE_INO={ino}"] + [f"-D{d}" for d in defines]
    tmp = out.with_suffix(f".{os.getpid()}.tmp")
    subprocess.run(cmd + ["-o", str(tmp), str(HERE / "arduino_host.cpp"), str(HERE / "firmware_main.cpp")],
                   check=True, capture_output=True, text=True)
    tmp.replace(out)
    return out


class Firmware:
    """One board. Each instance loads its own copy of the library (separate globals)."""

    def __init__(self, ino: Path | None = None, defines: tuple[str, ...] = ("HAVE_ULTRASONIC",)):
        lib = build(ino or default_ino(), defines)
        # dlopen returns the same handle for the same path; copy so boards are independent.
        self._dir = tempfile.mkdtemp(prefix="guide_fw_")
        own = Path(self._dir) / lib.name
        shutil.copy(lib, own)
        self.lib = ctypes.CDLL(str(own))
        L = self.lib
        L.host_set_micros.argtypes = [ctypes.c_uint64]
        L.host_micros.restype = ctypes.c_uint64
        L.host_rx.argtypes = [ctypes.c_char_p, ctypes.c_int]
        L.host_tx.argtypes = [ctypes.c_char_p, ctypes.c_int]
        L.host_set_echo_us.argtypes = [ctypes.c_uint32]
        L.host_reset()
        self._buf = ctypes.create_string_buffer(4096)
        self.started = False

    def start(self):
        self.lib.host_setup()
        self.started = True

    def run_until(self, t_s: float, loop_period_s: float = 0.001):
        """Call loop() until virtual time ``t_s`` (the board loops much faster; 1 ms is plenty here)."""
        if not self.started:
            self.start()
        target = int(t_s * 1e6)
        step = int(loop_period_s * 1e6)
        while self.lib.host_micros() < target:
            self.lib.host_set_micros(min(target, self.lib.host_micros() + step))
            self.lib.host_loop()

    @property
    def time(self) -> float:
        return self.lib.host_micros() * 1e-6

    def serial_in(self, data: bytes):
        if data:
            self.lib.host_rx(data, len(data))

    def serial_out(self) -> bytes:
        out = b""
        while True:
            n = self.lib.host_tx(self._buf, len(self._buf))
            if n <= 0:
                return out
            out += self._buf.raw[:n]

    def pwm(self, pin: int) -> int:
        return self.lib.host_pwm(pin)

    def servo_us(self, pin: int) -> int:
        """The pulse width the firmware sends a servo or ESC on ``pin``, 0 if none."""
        return self.lib.host_servo_us(pin)

    def digital(self, pin: int) -> int:
        return self.lib.host_digital(pin)

    def pin_mode(self, pin: int) -> int:
        return self.lib.host_pin_mode(pin)

    def set_input(self, pin: int, level: bool):
        self.lib.host_set_input(pin, 1 if level else 0)

    def set_echo_us(self, us: float):
        self.lib.host_set_echo_us(max(0, int(us)))

    def close(self):
        shutil.rmtree(self._dir, ignore_errors=True)
