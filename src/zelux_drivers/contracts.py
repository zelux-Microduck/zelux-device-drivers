"""五类可更换设备的稳定驱动契约。"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import StrEnum
from typing import Any, Protocol, runtime_checkable


class DeviceKind(StrEnum):
    SERVO = "servo"
    TOF = "tof"
    CAMERA = "camera"
    FAN = "fan"
    IMU = "imu"


@dataclass(frozen=True, slots=True)
class DriverDescriptor:
    id: str
    kind: DeviceKind
    version: str
    vendor: str
    models: tuple[str, ...]
    capabilities: tuple[str, ...]
    protocol: str = ">=1.1.0 <2.0.0"

    def to_dict(self) -> dict[str, Any]:
        out = asdict(self)
        out["kind"] = self.kind.value
        return out


@runtime_checkable
class DeviceDriver(Protocol):
    async def start(self) -> None: ...
    async def stop(self) -> None: ...
    def info(self) -> dict[str, Any]: ...


@runtime_checkable
class ServoDriver(DeviceDriver, Protocol):
    async def take(self) -> dict: ...
    async def release(self) -> dict: ...
    async def servo_read(self, sid: int) -> dict: ...
    async def servo_write(self, sid: int, reg: int, value: int, eeprom: bool = False) -> None: ...
    async def servo_torque(self, on: bool, ids: list[int] | None = None) -> None: ...
    async def servo_sync_goal(self, goals: dict[int, int]) -> None: ...
    def snapshot_servos(self) -> dict: ...


@runtime_checkable
class TofDriver(DeviceDriver, Protocol):
    def snapshot_tof(self) -> dict: ...


@runtime_checkable
class CameraDriver(DeviceDriver, Protocol):
    async def snapshot(self, path: str) -> bool: ...
    def stream_url(self) -> str | None: ...


@runtime_checkable
class FanDriver(DeviceDriver, Protocol):
    async def set_pct(self, index: int, pct: Any) -> dict: ...
    async def create_instance(self, instance: dict[str, Any]) -> dict: ...
    async def update_instance(self, instance_id: str, instance: dict[str, Any]) -> dict: ...
    async def delete_instance(self, instance_id: str) -> dict: ...
    async def set_enabled(self, instance_id: str, enabled: bool) -> dict: ...
    def snapshot_fan(self) -> dict: ...


@runtime_checkable
class ImuDriver(DeviceDriver, Protocol):
    def snapshot_imu(self) -> dict: ...
