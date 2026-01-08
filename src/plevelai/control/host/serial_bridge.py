"""Serial link helper for the Arduino stepper controller."""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Dict, Optional

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover - handled at runtime
    serial = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


@dataclass
class ArduinoBridge:
    port: Optional[str]
    baudrate: int = 115200
    timeout: float = 0.1
    dry_run: bool = False

    def __post_init__(self) -> None:
        if self.dry_run:
            self._ser = None
            return
        if not self.port:
            raise ValueError("Serial port required unless dry_run=True")
        if serial is None:
            raise RuntimeError(
                "pyserial not installed. Install with `pip install pyserial` or set dry_run=True"
            ) from _IMPORT_ERROR
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2.0)

    def close(self) -> None:
        if getattr(self, "_ser", None):
            self._ser.close()

    def send_move(self, joint_angles_deg: Dict[str, float], metadata: Optional[Dict] = None) -> None:
        payload = {"cmd": "move", "joints": joint_angles_deg}
        if metadata:
            payload.update(metadata)
        self._send(payload)

    def send_home(self) -> None:
        self._send({"cmd": "home"})

    def _send(self, payload: Dict) -> None:
        line = json.dumps(payload) + "\n"
        if self.dry_run:
            print(f"[dry-run] {line.strip()}")
            return
        assert self._ser is not None
        self._ser.write(line.encode("utf-8"))


__all__ = ["ArduinoBridge"]
