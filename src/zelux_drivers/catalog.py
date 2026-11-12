"""型号驱动目录：Agent 只按稳定 ID 创建实现。"""
from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .contracts import DeviceKind, DriverDescriptor

Factory = Callable[..., Any]
_entries: dict[str, tuple[DriverDescriptor, Factory]] = {}


def register(descriptor: DriverDescriptor, factory: Factory) -> None:
    if descriptor.id in _entries:
        raise ValueError(f"驱动 ID 重复：{descriptor.id}")
    _entries[descriptor.id] = descriptor, factory


def descriptors(kind: DeviceKind | str | None = None) -> list[DriverDescriptor]:
    wanted = DeviceKind(kind) if kind is not None else None
    return sorted((d for d, _ in _entries.values() if wanted is None or d.kind == wanted),
                  key=lambda d: d.id)


def descriptor(driver_id: str) -> DriverDescriptor:
    try:
        return _entries[driver_id][0]
    except KeyError:
        raise KeyError(f"未知驱动：{driver_id}") from None


def create(driver_id: str, *args: Any, **kwargs: Any) -> Any:
    try:
        factory = _entries[driver_id][1]
    except KeyError:
        raise KeyError(f"未知驱动：{driver_id}") from None
    return factory(*args, **kwargs)
