"""按设备类别拆分的稳定 Device API。

当前 SimHal/RadxaHal 仍是组合对象，但已经满足这些 Protocol。业务模块迁移时应依赖
对应的小接口，而不是依赖完整 Hal；后续换成独立驱动进程时无需再改业务签名。
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class ServoDevice(Protocol):
    async def servo_take(self): ...
    async def servo_release(self): ...
    async def servo_read(self, sid: int) -> dict: ...
    async def servo_write(self, sid: int, reg: int, value: int, eeprom: bool = False) -> None: ...
    async def servo_torque(self, on: bool, ids: list[int] | None = None) -> None: ...
    async def servo_sync_goal(self, goals: dict[int, int]) -> None: ...
    def snapshot_servos(self) -> dict: ...


@runtime_checkable
class TofDevice(Protocol):
    def snapshot_tof(self) -> dict: ...


@runtime_checkable
class CameraDevice(Protocol):
    async def camera_snapshot(self, path: str) -> bool: ...
    def camera_stream_url(self) -> str | None: ...


@runtime_checkable
class FanDevice(Protocol):
    def snapshot_fan(self) -> dict: ...


@runtime_checkable
class ImuDevice(Protocol):
    def snapshot_imu(self) -> dict: ...

