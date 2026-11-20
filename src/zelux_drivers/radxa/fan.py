"""Client adapter for the privileged zelux-fan service.

This module deliberately contains no PWM/sysfs or GPIO implementation. The root
fan service is the only process that owns those hardware resources.
"""
from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any

SOCKET_API_VERSION = 1
DEFAULT_PCT = 60
DEFAULT_CURVE: list[list[int]] = [[58, 70], [64, 85], [70, 100]]
FANS: tuple[dict[str, Any], ...] = ()  # inventory is discovered from fanctl


class FanUnavailable(RuntimeError):
    def __init__(self, message: str, hint: str = "") -> None:
        super().__init__(message)
        self.message = message
        self.hint = hint


class FanRequestError(FanUnavailable):
    """A reachable service rejected a request, not a transport outage."""
    def __init__(self, response: dict) -> None:
        super().__init__(str(response.get("error") or "风扇服务拒绝了请求"))
        self.code = str(response.get("code") or "invalid_request")
        self.response = response


class FanOutcomeUnknown(FanUnavailable):
    code = "outcome_unknown"


def clamp_pct(pct: Any, lo: int = 0, hi: int = 100) -> int:
    try:
        value = int(round(float(pct)))
    except (TypeError, ValueError):
        return DEFAULT_PCT
    return lo if value < lo else hi if value > hi else value


def normalize_curve(curve: Any) -> list[list[int]]:
    rows: list[list[int]] = []
    if isinstance(curve, (list, tuple)):
        for row in curve:
            try:
                temp, pct = float(row[0]), clamp_pct(row[1])
            except (TypeError, ValueError, IndexError):
                continue
            if -10 <= temp <= 120:
                rows.append([int(round(temp)), pct])
    if not rows:
        return [list(row) for row in DEFAULT_CURVE]
    rows.sort(key=lambda row: row[0])
    result: list[list[int]] = []
    for row in rows:
        if result and result[-1][0] == row[0]:
            result[-1] = row
        else:
            result.append(row)
    return result


def curve_pct(temp: float | None, curve: list[list[int]], default: int = DEFAULT_PCT) -> int:
    pct = clamp_pct(default)
    if temp is not None:
        for threshold, target in normalize_curve(curve):
            if float(temp) >= threshold:
                pct = target
    return pct


class FanHelper:
    """One-request-per-connection JSON-lines client for `/run/zelux/fan.sock`."""

    def __init__(self, path: str, timeout: float = 1.5) -> None:
        self.path = path
        self.timeout = timeout

    def exists(self) -> bool:
        try:
            return Path(self.path).exists()
        except OSError:
            return False

    async def call(self, payload: dict) -> dict:
        if not hasattr(asyncio, "open_unix_connection"):
            raise FanUnavailable("当前平台不支持 Unix socket")
        request = {"api_version": SOCKET_API_VERSION, **payload}
        # Normal status reads cached RPM; only explicit selftest samples inline.
        response_timeout = max(self.timeout, 30.0 if payload.get("op") == "selftest" else 10.0)
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_unix_connection(self.path), timeout=self.timeout)
        except (OSError, asyncio.TimeoutError) as exc:
            raise FanUnavailable(f"风扇服务不可用（{self.path}）：{exc}",
                                 "检查 zelux-fand.service 状态") from None
        try:
            writer.write((json.dumps(request, ensure_ascii=False) + "\n").encode("utf-8"))
            await asyncio.wait_for(writer.drain(), timeout=self.timeout)
            line = await asyncio.wait_for(reader.readline(), timeout=response_timeout)
        except (OSError, asyncio.TimeoutError) as exc:
            if payload.get("op") not in ("status", "ping", "get", "temp", "validate"):
                raise FanOutcomeUnknown("请求已发送，但未收到确认，操作结果未知；请刷新状态，不要重复创建", str(exc)) from None
            raise FanUnavailable(f"风扇服务请求失败：{exc}", "检查 zelux-fand 服务日志") from None
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except (OSError, AttributeError):
                pass
        try:
            response = json.loads(line.decode("utf-8", "replace"))
        except (ValueError, UnicodeDecodeError):
            if payload.get("op") not in ("status", "ping", "get", "temp", "validate"):
                raise FanOutcomeUnknown("服务确认响应缺失或无效，操作结果未知；请刷新核对，不要重放写入") from None
            raise FanUnavailable("风扇服务返回了无效 JSON；Agent 与风扇服务版本可能不兼容") from None
        if not isinstance(response, dict):
            raise FanUnavailable("风扇服务返回了非对象响应")
        version = response.get("api_version")
        if version != SOCKET_API_VERSION:
            raise FanUnavailable(f"风扇服务 socket API 不兼容（收到 {version!r}，需要 1）",
                                 "请成套更新 Device Agent 与 zelux-fan")
        if response.get("ok") is not True:
            if payload.get("op") == "selftest" and "ok" in response and response["ok"] is None:
                return response  # Unknown RPM is neither success nor transport failure.
            raise FanRequestError(response)
        return response


class RadxaFan:
    """HAL-facing adapter; all hardware operations are delegated to fanctl."""

    backend = "zelux-fan-service"
    source = "service"

    def __init__(self, ctx=None, *, helper_path: str | None = None, **_ignored: Any) -> None:
        cfg = getattr(ctx, "config", None)
        self.log = getattr(ctx, "log", None)
        self.helper = FanHelper(helper_path or getattr(cfg, "fan_socket", "/run/zelux/fan.sock"))
        self.fans: list[dict[str, Any]] = []
        self.curve = [list(row) for row in DEFAULT_CURVE]
        self.mode = "auto"
        self.available = False
        self.reason = "风扇服务尚未连接"
        self.hint = ""
        self.error = ""
        self._snapshot: dict[str, Any] = {"items": [], "inventory": None}
        self._task: asyncio.Task | None = None
        self._closing = False
        self._io_lock = asyncio.Lock()

    async def start(self) -> None:
        try:
            await self.refresh()
        except FanUnavailable:
            pass  # Still start recovery polling when fanctl is temporarily down.
        if self._task is None or self._task.done():
            self._closing = False
            self._task = asyncio.create_task(self._refresh_loop(), name="fan-service-status")

    async def stop(self) -> None:
        self._closing = True
        task, self._task = self._task, None
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    async def _refresh_loop(self) -> None:
        while not self._closing:
            await asyncio.sleep(2.0)
            try:
                await self.refresh()
            except FanUnavailable as exc:
                self.available = False
                self.reason, self.hint = exc.message, exc.hint

    async def refresh(self) -> dict:
        async with self._io_lock:
            return await self._refresh_unlocked()

    async def _refresh_unlocked(self) -> dict:
        try:
            status = await self.helper.call({"op": "status"})
        except FanUnavailable as exc:
            self.available = isinstance(exc, FanRequestError) and exc.code == "resource_busy"
            self._snapshot = {**self._snapshot, "status_stale": True}
            self.reason, self.hint = exc.message, exc.hint
            raise
        self._snapshot = {**status, "status_stale": False}
        inventory = status.get("inventory") if isinstance(status.get("inventory"), dict) else {}
        self.fans = [dict(item) for item in inventory.get("instances", []) if isinstance(item, dict)]
        self.curve = normalize_curve(status.get("curve"))
        self.mode = "manual" if status.get("mode") == "manual" else "auto"
        self.available = True
        self.reason = "" if self.fans else "风扇服务已连接，但尚未配置风扇实例"
        self.hint = "" if self.fans else "在 Studio 风扇页添加实例并配置 PWM 引脚"
        self.error = ""
        return status

    async def _command(self, payload: dict) -> dict:
        async with self._io_lock:
            result = await self.helper.call(payload)
            if result.get("ok") is not True:
                raise FanRequestError(result)
            inventory = result.get("inventory")
            if isinstance(inventory, dict):
                ids = {item["id"] for item in inventory.get("instances", []) if item.get("enabled")}
                self._snapshot = {**self._snapshot, "inventory": inventory,
                                  "items": [item for item in self._snapshot.get("items", []) if item.get("id") in ids]}
                self.fans = [dict(item) for item in inventory.get("instances", [])]
            try:
                await self._refresh_unlocked()
            except FanUnavailable as exc:
                # The write was acknowledged. Never turn a later read failure
                # into a failed write or invite replay of a successful create.
                self._snapshot = {**self._snapshot, "status_stale": True}
                return {**result, "status_stale": True,
                        "refresh_warning": "操作已确认成功，实时状态刷新失败：" + exc.message}
            return result

    def info(self) -> dict:
        return {"backend": self.backend, "source": self.source, "available": self.available,
                "socket": self.helper.path, "curve": self.curve, "mode": self.mode,
                "reason": self.reason, "hint": self.hint}

    def _instance_id(self, target: Any) -> str:
        if isinstance(target, str) and target:
            return target
        try:
            index = int(target)
            return str(self.fans[index]["id"])
        except (ValueError, TypeError, IndexError, KeyError):
            raise FanUnavailable(f"未知风扇实例：{target!r}") from None

    async def set_pct(self, target: Any, pct: Any) -> dict:
        async with self._io_lock:
            return await self._set_pct_unlocked(target, pct)

    async def _set_pct_unlocked(self, target: Any, pct: Any) -> dict:
        instance_id = self._instance_id(target)
        value = clamp_pct(pct)
        try:
            result = await self.helper.call({"op": "set", "instance_id": instance_id, "pct": value})
            self.mode = result.get("mode", "auto")
            for item in self.fans:
                if item.get("id") == instance_id:
                    item["pct"] = result.get("pct", value)
            return {**result, "available": True, "source": self.source}
        except FanUnavailable as exc:
            self.error = exc.message
            return {"ok": False, "available": self.available, "reason": exc.message,
                    "code": getattr(exc, "code", "hardware"),
                    "hint": exc.hint, "instance_id": instance_id, "pct": value}

    async def set_auto(self) -> dict:
        result = await self._command({"op": "auto"})
        self.mode = "auto"
        return result

    async def set_curve(self, curve: Any) -> dict:
        points = normalize_curve(curve)
        result = await self._command({"op": "curve", "points": points})
        self.curve = points
        return result

    async def read(self, target: Any) -> dict:
        instance_id = self._instance_id(target)
        result = await self.helper.call({"op": "get", "instance_id": instance_id})
        return {**result, "available": self.available, "source": self.source}

    async def apply_curve(self, _temp: float | None = None) -> dict:
        return await self.set_auto()

    async def self_test(self, target: Any = None) -> dict:
        payload = {"op": "selftest"}
        if target is not None:
            payload["instance_id"] = self._instance_id(target)
        return await self.helper.call(payload)

    async def create_instance(self, instance: dict) -> dict:
        return await self._command({"op": "create", "instance": instance})

    async def update_instance(self, instance_id: str, instance: dict) -> dict:
        return await self._command({"op": "update", "instance_id": instance_id, "instance": instance})

    async def delete_instance(self, instance_id: str) -> dict:
        return await self._command({"op": "delete", "instance_id": instance_id, "confirm": True})

    async def set_enabled(self, instance_id: str, enabled: bool) -> dict:
        return await self._command({"op": "enable" if enabled else "disable",
                                    "instance_id": instance_id})

    def snapshot_fan(self) -> dict:
        status = self._snapshot
        return {"mode": self.mode, "pct": status.get("pct", DEFAULT_PCT),
                "status_stale": status.get("status_stale", True),
                "temp": status.get("temp"), "curve": self.curve,
                "items": [dict(item) for item in status.get("items", []) if isinstance(item, dict)],
                "inventory": status.get("inventory"), "available": self.available,
                "reason": self.reason, "error": self.error, "hint": self.hint,
                "source": self.source, "period_ns": status.get("period_ns"),
                "instance_count": len(status.get("items", []))}
