"""VL53L5CX 8×8 分区测距（Radxa Zero 3W，I²C-3，丝印地址 0x52 / 7 位 0x29）。

为什么上电必须做"上传固件 → 8×8 → 15 Hz"三步：
VL53L5CX 出厂只有一段 bootloader，内核里没有能直接出数的驱动。必须由用户态把 ST 的
固件（约 84 KB）灌进去，再下发 8×8 分区与测距频率，最后 start_ranging 才出数据。
所以这里不存在"读寄存器就有距离"的捷径；没装驱动库就直接报 unsupported + 一条能跑通的
命令，而不是每 66 ms 报一次超时把日志刷满。

为什么优先 vl53l5cx_ctypes 而不是自己用 smbus2 手写：
手写要搬 84 KB 固件和一张带 CRC 的 2 KB 默认配置表（都在 ST 的 ULD 里）。表里错一位，
读回来的距离会"看着正常但整体偏一倍"，这种错比直接报错难查得多。因此 smbus2 这条路径
只做一件靠谱的事：**探测设备在不在**，然后把原因说清楚，让人去装库。

数据约定的两个坑（都在真机上咬过人）：
1. `status` 只有 5 和 9 是有效距离，255 是没目标；把 255 当 0 mm 会让避障一路急刹。
2. 弱反射/量程外时即使 status=5 也可能给出 8191 这类哨兵值，所以 `valid` 还要卡量程，
   但**原始数组照原样上报**，别偷偷改写成 0——否则前端画的热力图是假的。
"""
from __future__ import annotations

import asyncio
import importlib
import logging
import time
from typing import Any

log = logging.getLogger("zeluxd.tof")

# ---------------------------------------------------------------- 常量
ROWS = 8
COLS = 8
ZONES = ROWS * COLS
DEFAULT_HZ = 15
DEFAULT_ADDR = 0x52            # 硬件丝印/8 位写地址
ADDR_7BIT = DEFAULT_ADDR >> 1  # 0x29：smbus2 与 ctypes 都用 7 位
VALID_STATUS = (5, 9)
NO_TARGET = 255
STALE_S = 3.0                  # 掉帧超过 3 秒就不是"抖一下"，是真的没数据
GHOST_MAX_MM = 4000            # 8×8 长距模式额定量程；超过这个数当无效（哨兵值）
RETRY_S = 10.0                 # 传感器可能比 zeluxd 晚上电，别一次失败就永远放弃
SENSOR = "vl53l5cx"

# 依赖缺失时唯一正确的姿势：给一条能直接粘进终端跑的命令
INSTALL_HINT = "pip install vl53l5cx-ctypes 或跑 zelux-tof-install（镜像里有一键脚本）"
UNSUPPORTED_REASON = "没装 vl53l5cx 驱动，执行 zelux-tof-install"


class TofUnavailable(RuntimeError):
    """带人话原因与修复建议的不可用异常（内部用，HAL 层不往外抛）。"""

    def __init__(self, message: str, hint: str = "") -> None:
        super().__init__(message)
        self.message = message
        self.hint = hint or INSTALL_HINT


# ---------------------------------------------------------------- 纯函数
def is_valid(status: int, distance_mm: int = 0) -> bool:
    """这一格算不算一次可信的距离读数。"""
    return int(status) in VALID_STATUS and 0 < int(distance_mm) <= GHOST_MAX_MM


def zone_index(row: int, col: int) -> int:
    """行优先：0..7 是第一行。与 vl53l5cx_ctypes 的数组顺序一致。"""
    return int(row) * COLS + int(col)


def compose_frame(distance_mm: list[int] | None, status: list[int] | None, *,
                  seq: int = 0, hz: int = DEFAULT_HZ, ts: float | None = None) -> dict:
    """把两个裸数组拼成契约里的 tof 对象（纯函数，单测可以直接调）。

    短了补、长了截：传感器在 8×8 与 4×4 之间切过一次时数组长度会变，
    与其让避障在 16 格上算三扇区，不如在这里就把形状钉死。
    """
    dist = [int(d or 0) for d in (distance_mm or [])][:ZONES]
    stat = [int(s if s is not None else NO_TARGET) for s in (status or [])][:ZONES]
    dist += [0] * (ZONES - len(dist))
    stat += [NO_TARGET] * (ZONES - len(stat))

    valid = [d for d, s in zip(dist, stat) if is_valid(s, d)]
    ghosts = sum(1 for d, s in zip(dist, stat) if s in VALID_STATUS and not is_valid(s, d))
    return {
        "state": "ready",
        "sensor": SENSOR,
        "rows": ROWS,
        "cols": COLS,
        "hz": hz,
        "distance_mm": dist,
        "status": stat,
        "nearest_mm": min(valid) if valid else None,
        "valid_count": len(valid),
        "ghost_count": ghosts,
        "seq": int(seq),
        "age_ms": 0,
        "ts": ts if ts is not None else time.time(),
    }


def empty_reason(state: str, reason: str, **extra: Any) -> dict:
    """不可用/等待中的统一形状：前端只认 state，别的字段给日志页看。"""
    out = {"state": state, "sensor": SENSOR, "rows": ROWS, "cols": COLS,
           "hz": DEFAULT_HZ, "distance_mm": [], "status": [],
           "nearest_mm": None, "valid_count": 0, "seq": 0, "age_ms": 0,
           "reason": reason}
    out.update(extra)
    return out


# ---------------------------------------------------------------- 后端：ctypes 库
class CtypesBackend:
    """vl53l5cx-ctypes 后端。OKI/ST 的 ULD 都在库里，我们只负责三行配置。"""

    name = "vl53l5cx-ctypes"

    def __init__(self, bus: int, addr7: int, hz: int) -> None:
        self.bus = bus
        self.addr7 = addr7
        self.hz = hz
        self._sensor = None

    @staticmethod
    def available() -> bool:
        try:
            importlib.import_module("vl53l5cx_ctypes")
            return True
        except ImportError:
            return False

    def open(self) -> None:
        try:
            mod = importlib.import_module("vl53l5cx_ctypes")
        except ImportError:
            raise TofUnavailable(UNSUPPORTED_REASON) from None

        cls = getattr(mod, "VL53L5CX", None)
        if cls is None:
            raise TofUnavailable("vl53l5cx-ctypes 里没有 VL53L5CX 类：装成别的包了",
                                 "pip uninstall vl53l5cx-ctypes && pip install vl53l5cx-ctypes")

        # 不同版本构造函数签名不一样（有人用 i2c_bus/i2c_addr，有人只吃 kwargs、有人无参），
        # 与其钉死一个版本号，不如按可能性逐个试，全失败再用无参默认（走 1 号总线）。
        sensor = None
        for kwargs in ({"i2c_bus": self.bus, "i2c_addr": self.addr7},
                       {"bus": self.bus, "addr": self.addr7},
                       {"i2c_bus": self.bus},
                       {}):
            try:
                sensor = cls(**kwargs)
                break
            except TypeError:
                continue
            except OSError as exc:
                raise TofUnavailable(
                    f"I²C-{self.bus} 上打不开 0x{self.addr7:02X}：{exc}",
                    f"先跑 i2cdetect -y {self.bus} 看 0x{self.addr7:02X} 在不在；"
                    f"不在就查 4pin 线序与 3V3 供电") from None
            except RuntimeError as exc:
                # 库自己找不到设备时抛的是 RuntimeError，现场最常见的就是线插反/没上电
                raise TofUnavailable(
                    f"传感器没应答：{exc}",
                    f"跑 i2cdetect -y {self.bus} 确认 0x{self.addr7:02X}；"
                    f"查到地址却初始化失败，先给模块单独供 3V3 再试") from None
        if sensor is None:
            raise TofUnavailable("vl53l5cx-ctypes 初始化失败（构造函数签名对不上）")

        self._sensor = sensor
        try:
            # 分辨率：老版本只有整数 64，新版本给常量。8×8 是我们避障三扇区的前提。
            res = getattr(mod, "VL53L5CX_RESOLUTION_8X8", ZONES)
            sensor.set_resolution(res)
            sensor.set_ranging_frequency_hz(self.hz)
            sensor.start_ranging()
        except Exception as exc:  # 固件没灌进去、I²C 半路掉线都从这里冒出来
            self.close()
            raise TofUnavailable(f"传感器配置失败：{exc}",
                                 "确认 3V3 供电够（ToF 峰值电流不小），线别超过 20 cm") from None

    def read(self) -> tuple[list[int], list[int]]:
        data = self._sensor.get_ranging_data()
        return list(data.distance_mm), list(data.target_status)

    def close(self) -> None:
        sensor, self._sensor = self._sensor, None
        if sensor is None:
            return
        try:
            sensor.stop_ranging()
        except Exception:
            pass


# ---------------------------------------------------------------- 后端：只探测
class ProbeBackend:
    """只有 smbus2 时的降级路径：确认设备在不在，不假装能出数。

    为什么只做探测：见模块 docstring——搬固件需要 ST ULD 的二进制与配置表，
    手写一份等于给现场埋一个"距离偏一倍"的雷。
    """

    name = "smbus2-probe"

    def __init__(self, bus: int, addr7: int) -> None:
        self.bus = bus
        self.addr7 = addr7
        self._found = False
        self._why = ""

    @staticmethod
    def smbus_available() -> bool:
        try:
            importlib.import_module("smbus2")
            return True
        except ImportError:
            return False

    def probe(self) -> bool:
        """True = 总线上有设备应答。此时"缺库"才是真原因，而不是接线错。"""
        try:
            from smbus2 import SMBus
        except ImportError:
            self._why = "smbus2 也没装"
            return False
        try:
            with SMBus(self.bus) as bus:
                bus.read_byte(self.addr7)
            self._found = True
            return True
        except FileNotFoundError:
            self._why = f"没有 /dev/i2c-{self.bus}：I²C 总线没开（检查 /boot 的 overlays）"
        except PermissionError:
            self._why = f"没有 /dev/i2c-{self.bus} 的权限：把 zelux 用户加进 i2c 组后重登"
        except OSError as exc:
            self._why = f"0x{self.addr7:02X} 没应答（{getattr(exc, 'strerror', None) or exc}）"
        return False

    @property
    def why(self) -> str:
        return self._why


# ---------------------------------------------------------------- 外壳
class RadxaTof:
    """给 `hal/radxa.py` 用的 ToF 部分。方法名与 `hal/base.py` 的 `snapshot_tof` 对齐。"""

    backend = "radxa-tof"

    def __init__(self, ctx=None, *, bus: int | None = None, addr: int | None = None,
                 hz: int = DEFAULT_HZ) -> None:
        cfg = getattr(ctx, "config", None)
        self.log = getattr(ctx, "log", log) or log
        self.bus = int(bus if bus is not None else getattr(cfg, "i2c_bus", 3))
        raw_addr = int(addr if addr is not None else getattr(cfg, "tof_addr", DEFAULT_ADDR))
        # 配置里写的是丝印值 0x52（8 位写地址），smbus2/ctypes 要 7 位
        self.addr7 = raw_addr >> 1 if raw_addr > 0x3F else raw_addr
        self.hz = max(1, int(hz))

        self._dev: Any = None
        self._seq = 0
        self._last_ok = 0.0
        self._hz_real = 0.0
        self._fault: dict[str, str] = {}
        self._cache: dict | None = None
        self._task: asyncio.Task | None = None
        self._closing = False

    # ---------------- 生命周期 ----------------
    async def start(self) -> None:
        if self._task and not self._task.done():
            return
        self._closing = False
        self._task = asyncio.create_task(self._sample_loop(), name="tof-sample")

    async def stop(self) -> None:
        self._closing = True
        task, self._task = self._task, None
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception:
                pass
        dev, self._dev = self._dev, None
        if dev is not None:
            try:
                await asyncio.to_thread(dev.close)
            except Exception:
                pass

    def info(self) -> dict:
        return {"backend": self.backend, "bus": self.bus, "addr": f"0x{self.addr7:02X}",
                "hz": self.hz, "hz_real": round(self._hz_real, 1),
                "driver": getattr(self._dev, "name", None), "fault": dict(self._fault)}

    # ---------------- 采样 ----------------
    async def _sample_loop(self) -> None:
        period = 1.0 / self.hz
        while not self._closing:
            if self._dev is None:
                opened = await self._try_open()
                if not opened:
                    # 别 66 ms 重试一次把 CPU 与日志打满；传感器晚 10 秒上电还能自己回来
                    await asyncio.sleep(RETRY_S)
                    continue
            t0 = time.monotonic()
            try:
                dist, stat = await asyncio.to_thread(self._dev.read)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._lose(f"读 ToF 失败：{exc}")
                await asyncio.sleep(1.0)
                continue

            self._seq += 1
            self._cache = compose_frame(dist, stat, seq=self._seq, hz=self.hz)
            self._last_ok = time.time()
            self._fault = {}
            dt = time.monotonic() - t0
            if dt > 0.002:
                inst = 1.0 / dt
                self._hz_real = inst if not self._hz_real else self._hz_real * 0.9 + inst * 0.1
                self._cache["hz_real"] = round(self._hz_real, 1)
            await asyncio.sleep(max(0.0, period - dt))

    async def _try_open(self) -> bool:
        try:
            dev = await asyncio.to_thread(self._open_sync)
        except TofUnavailable as exc:
            if self._fault.get("reason") != exc.message:
                # 只在原因变化时记一条：每 10 秒刷同样的错误没人看
                self.log.warning("ToF 不可用：%s（%s）", exc.message, exc.hint)
            self._fault = {"state": "unsupported", "reason": exc.message, "detail": exc.hint}
            return False
        except Exception as exc:
            self._fault = {"state": "error", "reason": f"ToF 初始化失败：{exc}"}
            return False
        self._dev = dev
        self.log.info("ToF 就绪：%s · I²C-%d/0x%02X · %d×%d @ %d Hz",
                      getattr(dev, "name", "?"), self.bus, self.addr7, ROWS, COLS, self.hz)
        return True

    def _open_sync(self):
        """同步打开（在 to_thread 里跑）：ctypes 后端要搬固件，几百毫秒正常。"""
        if CtypesBackend.available():
            dev = CtypesBackend(self.bus, self.addr7, self.hz)
            dev.open()
            return dev

        # 没有 ctypes：能确认"是缺库"还是"是接线"，这两种情况给的命令完全不同
        probe = ProbeBackend(self.bus, self.addr7)
        if probe.probe():
            raise TofUnavailable(
                f"I²C-{self.bus} 上 0x{self.addr7:02X} 有应答，但缺固件驱动：{UNSUPPORTED_REASON}",
                f"{INSTALL_HINT}。只装 smbus2 是不够的——VL53L5CX 必须上传 84 KB 固件才能出数")
        raise TofUnavailable(f"I²C-{self.bus} 上没找到 0x{self.addr7:02X}：{probe.why}",
                             f"跑 i2cdetect -y {self.bus} 确认；4pin 线序 3V3/GND/SDA/SCL 别插反")

    def _lose(self, why: str) -> None:
        self.log.warning("%s；1 秒后重连", why)
        dev, self._dev = self._dev, None
        if dev is not None:
            try:
                dev.close()
            except Exception:
                pass
        self._fault = {"state": "error", "reason": why}

    # ---------------- 读缓存 ----------------
    def snapshot_tof(self) -> dict:
        if self._cache is None:
            if self._fault:
                return empty_reason(self._fault["state"], self._fault["reason"],
                                    detail=self._fault.get("detail", ""), bus=self.bus,
                                    addr=f"0x{self.addr7:02X}")
            return empty_reason("waiting", "正在上电初始化：上传固件 → 8×8 分区 → 15 Hz",
                                bus=self.bus, addr=f"0x{self.addr7:02X}")

        age = max(0.0, time.time() - self._last_ok)
        out = dict(self._cache)
        out["age_ms"] = int(age * 1000)
        out["bus"] = self.bus
        out["addr"] = f"0x{self.addr7:02X}"
        if age > STALE_S:
            out["state"] = "stale"
            out["reason"] = (f"{age:.1f} 秒没有新数据：传感器掉线、或 I²C-{self.bus} 被别的进程占了"
                             f"（跑 i2cdetect -y {self.bus} 看 0x{self.addr7:02X}）")
        return out

    async def tof_frame(self) -> dict | None:
        snap = self.snapshot_tof()
        return snap if snap.get("state") == "ready" else None

    def probe(self) -> dict:
        """诊断页用：不打开传感器，只回答"库在不在、线上有没有设备"。"""
        has_lib = CtypesBackend.available()
        probe = ProbeBackend(self.bus, self.addr7)
        found = probe.probe()
        return {"bus": self.bus, "addr": f"0x{self.addr7:02X}",
                "library": "vl53l5cx-ctypes" if has_lib else None,
                "device": found, "why": "" if found else probe.why,
                "hint": "" if has_lib else INSTALL_HINT}
