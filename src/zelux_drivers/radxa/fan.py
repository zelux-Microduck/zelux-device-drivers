"""双风扇：PWM 调速 + 4pin 测速（Radxa Zero 3W）。

硬件事实（实测，别按"常见接法"改）：
* 风扇 1：PWM14 / GPIO3_C4（全局 116），**没有测速线**，只能调速；
* 风扇 2：PWM12 / GPIO149 调速 + GPIO150 测速（4pin 带 tach），接口丝印 `GPIO4_C6/PIN_24`；
* 两颗都是 `period=40000ns / 25 kHz / normal polarity`，duty 0–40000。

为什么要走 root 助手（`/run/zelux/fan.sock`）：
zeluxd 以 zelux 用户跑，`/sys/class/pwm` 归 root，直接写会 EACCES。把"开通道、写 duty"
关在一个最小 root 服务里，网页进程被攻破也拿不到 root。socket 不在时退化为直接写 sysfs
（板子上若已 chmod/chown 给 zelux 组，这条路同样能用）——两条路都走不通就报
`available:false` + 人话原因，绝不抛异常把整个状态流带崩。

测速为什么是"1 秒窗口计数"而不是万用表式的瞬时值：
4pin 风扇的 tach 是开漏方波，每转 2 个脉冲（`pulses_per_rev` 可配，有的风扇是 1 或 4）。
频率低（几十到几百 Hz），用 1 秒窗口计数误差最小；RPM = hz × 60 / pulses_per_rev。
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from pathlib import Path
from typing import Any

log = logging.getLogger("zeluxd.fan")

# ---------------------------------------------------------------- 硬件参数
PWM_PERIOD_NS = 40000
PWM_FREQ_HZ = 25000
DUTY_MAX = PWM_PERIOD_NS
DEFAULT_PCT = 60
DEFAULT_CURVE: list[list[int]] = [[58, 70], [64, 85], [70, 100]]

FANS: tuple[dict[str, Any], ...] = (
    {"name": "顶部风扇", "pwm": 14, "gpio": 116, "pin": "GPIO3_C4", "tach": None},
    {"name": "主板散热", "pwm": 12, "gpio": 149, "pin": "GPIO4_C6/PIN_24", "tach": 150},
)

PULSES_PER_REV = 2
TACH_WINDOW_S = 1.0
TACH_MIN_RPM = 300            # 自检判定：低于这个数就是没转（或线没插）
RPM_REFRESH_S = 2.0


class FanUnavailable(RuntimeError):
    """内部使用：不可用时带人话原因，外层统一转成 available:false。"""

    def __init__(self, message: str, hint: str = "") -> None:
        super().__init__(message)
        self.message = message
        self.hint = hint


# ---------------------------------------------------------------- 纯函数
def clamp_pct(pct: Any, lo: int = 0, hi: int = 100) -> int:
    """面板输入什么都能进来（null/-1/1200），这里统一夹紧，别让 duty 溢出。"""
    try:
        v = int(round(float(pct)))
    except (TypeError, ValueError):
        return DEFAULT_PCT
    return lo if v < lo else hi if v > hi else v


def pct_to_duty(pct: Any, duty_max: int = DUTY_MAX) -> int:
    return int(round(clamp_pct(pct) / 100.0 * duty_max))


def duty_to_pct(duty: Any, duty_max: int = DUTY_MAX) -> int:
    try:
        d = int(duty)
    except (TypeError, ValueError):
        return 0
    d = max(0, min(duty_max, d))
    return int(round(d * 100.0 / duty_max))


def rpm_from_hz(hz: float, pulses_per_rev: int = PULSES_PER_REV) -> int:
    """4pin 风扇一般每转 2 个脉冲。ppr 配错时读数会整体偏 2 倍，先看这个参数。"""
    ppr = max(1, int(pulses_per_rev))
    return max(0, int(round(max(0.0, float(hz)) * 60.0 / ppr)))


def normalize_curve(curve: Any) -> list[list[int]]:
    """校验温控阶梯：[[温度, 百分比], ...]，按温度升序、夹紧、去重。

    错的曲线比没有曲线更糟（比如温度写反了会导致越热越慢），所以宁可回退默认值。
    """
    rows: list[list[int]] = []
    if isinstance(curve, (list, tuple)):
        for item in curve:
            try:
                t, p = float(item[0]), int(item[1])
            except (TypeError, ValueError, IndexError):
                continue
            if not (-10 <= t <= 120):
                continue
            rows.append([int(round(t)), clamp_pct(p)])
    if not rows:
        return [list(r) for r in DEFAULT_CURVE]
    rows.sort(key=lambda r: r[0])
    out: list[list[int]] = []
    for t, p in rows:                       # 同一温度只留最后一条，避免二义
        if out and out[-1][0] == t:
            out[-1] = [t, p]
        else:
            out.append([t, p])
    return out


def curve_pct(temp: float | None, curve: Any = None, default: int = DEFAULT_PCT) -> int:
    """阶梯温控：温度 >= 台阶温度就套那一档，都不满足用默认 60%。

    默认 60% 而不是 0%：停转会让 RK3566 在重负载下十几秒就上 80℃，
    而且风扇重启时的"咯"一下比常转更烦人。
    """
    if temp is None:
        return clamp_pct(default)
    ladder = normalize_curve(curve if curve is not None else DEFAULT_CURVE)
    pct = clamp_pct(default)
    for t, p in ladder:
        if float(temp) >= t:
            pct = p
    return pct


# ---------------------------------------------------------------- root 助手
class FanHelper:
    """一行一个 JSON 的 unix socket 客户端（helper 端见 image/overlay）。

    请求：`{"op":"set","index":0,"pwm":14,"duty":24000,"period":40000}`
    应答：`{"ok":true,"duty":24000}` / `{"ok":false,"error":"..."}`
    """

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
            raise FanUnavailable("这个平台没有 unix socket：风扇只能走 sysfs",
                                 "把 zelux 用户加进 gpio 组，或直接用 zelux-fand")
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_unix_connection(self.path), timeout=self.timeout)
        except (FileNotFoundError, ConnectionRefusedError) as exc:
            raise FanUnavailable(f"根助手没在跑（{self.path}）：{exc}",
                                 "sudo systemctl restart zelux-fand") from None
        except (OSError, asyncio.TimeoutError) as exc:
            raise FanUnavailable(f"连不上根助手（{self.path}）：{exc}",
                                 "sudo systemctl status zelux-fand 看它为什么起不来") from None
        try:
            writer.write((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))
            await asyncio.wait_for(writer.drain(), timeout=self.timeout)
            line = await asyncio.wait_for(reader.readline(), timeout=self.timeout)
        except asyncio.TimeoutError:
            raise FanUnavailable("根助手没回话（1.5 秒超时）",
                                 "sudo journalctl -u zelux-fand -n 50") from None
        except OSError as exc:
            raise FanUnavailable(f"和根助手通信断了：{exc}") from None
        finally:
            try:
                writer.close()
            except Exception:
                pass
        if not line:
            raise FanUnavailable("根助手直接关了连接", "sudo journalctl -u zelux-fand -n 50")
        try:
            res = json.loads(line.decode("utf-8", "replace"))
        except ValueError:
            raise FanUnavailable("根助手回的不是 JSON", "版本对不上：把 zelux-fand 一起升级") from None
        if not isinstance(res, dict):
            raise FanUnavailable("根助手回了奇怪的东西")
        if res.get("error"):
            raise FanUnavailable(f"根助手拒绝：{res['error']}")
        return res


# ---------------------------------------------------------------- sysfs PWM（降级）
class SysfsPwm:
    """`/sys/class/pwm/pwmchipN/pwmM/{period,duty_cycle,enable}` 的最薄一层封装。

    降级路径的已知不确定点：内核给通道编号是"全局号"，但通道挂在哪个 pwmchip 上
    取决于设备树。所以先找已导出的 `pwm<number>`，找不到才尝试 export，
    并在返回值里标 `source="sysfs"` 提醒现场核对是不是动对了风扇。
    """

    def __init__(self, number: int, root: str = "/sys/class/pwm") -> None:
        self.number = int(number)
        self.root = Path(root)
        self._path: Path | None = None
        self._chip: Path | None = None
        self._exported = False

    def resolve(self) -> Path | None:
        if self._path is not None and self._path.exists():
            return self._path
        if not self.root.exists():
            return None
        for chip in sorted(self.root.glob("pwmchip*")):
            direct = chip / f"pwm{self.number}"
            if direct.exists():
                self._path = direct
                return self._path
        # 只有 probe 阶段确认过 export 可写（self._chip）才去动它，
        # 否则可能把编号撞进错误的控制器：控制错风扇比控制不了更糟
        chips = [self._chip] if self._chip else []
        for chip in chips + [c for c in sorted(self.root.glob("pwmchip*")) if c != self._chip]:
            try:
                (chip / "export").write_text(str(self.number), "utf-8")
            except OSError:
                continue
            direct = chip / f"pwm{self.number}"
            if direct.exists():
                self._path, self._exported = direct, True
                return self._path
        return None

    def writable(self) -> bool:
        """只判断"能不能写"，**不 export**。

        导出通道的副作用等于接管：内核会把 period/duty 归零并关掉输出，
        正在转的风扇会当场停。所以探测阶段只找已存在的通道（helper 导出的）
        或可写的 export 入口。
        """
        if self._path is not None and self._path.exists():
            return os.access(self._path / "duty_cycle", os.W_OK)
        if not self.root.exists():
            return False
        for chip in sorted(self.root.glob("pwmchip*")):
            direct = chip / f"pwm{self.number}"
            if direct.exists():
                self._path = direct
                return os.access(direct / "duty_cycle", os.W_OK)
        for chip in sorted(self.root.glob("pwmchip*")):
            if os.access(chip / "export", os.W_OK):
                self._chip = chip        # 记下能写的 chip，真调速时再 export
                return True
        return False

    def _write(self, name: str, value: int) -> None:
        path = self.resolve()
        if path is None:
            raise FanUnavailable(f"系统里没有 pwm{self.number} 通道",
                                 "确认设备树开了 pwm14/pwm12 的 overlay")
        target = path / name
        try:
            target.write_text(str(int(value)), "utf-8")
        except PermissionError:
            raise FanUnavailable(f"没权限写 {target}",
                                 "把 zelux 加入 gpio 组，或启动 zelux-fand") from None
        except OSError as exc:
            raise FanUnavailable(f"写 {target} 失败：{exc}") from None

    def apply(self, duty: int, period: int = PWM_PERIOD_NS) -> None:
        # 先关再设：有的内核在 enable=1 时改 period 会直接 EBUSY
        try:
            self._write("enable", 0)
        except FanUnavailable:
            pass
        self._write("period", period)
        self._write("duty_cycle", max(0, min(period, int(duty))))
        self._write("enable", 1)


# ---------------------------------------------------------------- 测速
def _gpio_chip_for(global_no: int) -> tuple[str | None, int]:
    """全局 GPIO 号 → (/dev/gpiochipN, 片内 offset)。

    内核的 gpiochip 基址是动态的，别硬编码 gpiochip4：扫 sysfs 的 base/ngpio 才靠谱。
    """
    root = Path("/sys/class/gpio")
    offset = global_no % 32
    if root.exists():
        for chip in sorted(root.glob("gpiochip*")):
            try:
                base = int((chip / "base").read_text("utf-8").strip())
                ngpio = int((chip / "ngpio").read_text("utf-8").strip())
            except (OSError, ValueError):
                continue
            if base <= global_no < base + ngpio:
                return f"/dev/{chip.name}", global_no - base
    return None, offset


class Tach:
    """GPIO 上的转速计：优先 libgpiod 边沿中断，退化到 sysfs 轮询。"""

    def __init__(self, gpio: int, pulses_per_rev: int = PULSES_PER_REV) -> None:
        self.gpio = int(gpio)
        self.ppr = max(1, int(pulses_per_rev))

    # -- libgpiod（两条 API 都试，发行版里 v1/v2 都有人用） --
    def _count_gpiod(self, window: float) -> float | None:
        try:
            import gpiod  # 真机才有；本机 import 失败就走 sysfs
        except ImportError:
            return None
        device, offset = _gpio_chip_for(self.gpio)
        if device is None:
            return None
        for fn in (self._count_v2, self._count_v1):
            try:
                hz = fn(gpiod, device, offset, window)
            except Exception:
                continue
            if hz is not None:
                return hz
        return None

    @staticmethod
    def _count_v2(gpiod, device: str, offset: int, window: float) -> float | None:
        from gpiod.line import Direction, Edge  # v2 才有

        settings = gpiod.LineSettings(direction=Direction.INPUT, edge_detection=Edge.RISING)
        req = gpiod.request_lines(device, consumer="zelux-fan", config={offset: settings})
        n = 0
        end = time.monotonic() + window
        try:
            while time.monotonic() < end:
                left = max(0.0, end - time.monotonic())
                if req.wait_edge_events(timeout=left):
                    n += len(req.read_edge_events())
        finally:
            req.release()
        return n / window

    @staticmethod
    def _count_v1(gpiod, device: str, offset: int, window: float) -> float | None:
        chip = gpiod.Chip(device)          # v1：Chip(path) 可以，v2 会抛，正好分流
        line = chip.get_line(offset)
        line.request(consumer="zelux-fan", type=gpiod.LINE_REQ_EV_RISING_EDGE)
        n = 0
        end = time.monotonic() + window
        try:
            while time.monotonic() < end:
                left = max(0.0, end - time.monotonic())
                if line.event_wait(sec=left, nsec=0):
                    line.event_read()
                    n += 1
        finally:
            try:
                line.release()
            except Exception:
                pass
        return n / window

    # -- sysfs 轮询 --
    def _count_sysfs(self, window: float) -> float | None:
        base = Path("/sys/class/gpio")
        node = base / f"gpio{self.gpio}"
        try:
            base.mkdir(parents=True, exist_ok=True)
            if not node.exists():
                (base / "export").write_text(str(self.gpio), "utf-8")
                time.sleep(0.05)
            (node / "direction").write_text("in", "utf-8")
            try:
                (node / "edge").write_text("rising", "utf-8")
            except OSError:
                pass                        # 有的内核不让改 edge，照样能轮询
        except (OSError, PermissionError) as exc:
            log.debug("export gpio%d 失败：%s", self.gpio, exc)
            return None

        value = node / "value"
        try:
            prev = value.read_text("utf-8").strip()
        except OSError:
            return None
        n = 0
        end = time.monotonic() + window
        while time.monotonic() < end:
            try:
                cur = value.read_text("utf-8").strip()
            except OSError:
                return None
            if cur == "1" and prev == "0":
                n += 1
            prev = cur
        return n / window

    def measure(self, window: float = TACH_WINDOW_S) -> dict:
        """同步测量（在 to_thread 里跑）：返回 {ok, hz, rpm, why, backend}。"""
        for name, fn in (("libgpiod", self._count_gpiod), ("sysfs", self._count_sysfs)):
            try:
                hz = fn(window)
            except Exception as exc:
                log.debug("测速 %s 失败：%s", name, exc)
                continue
            if hz is None:
                continue
            return {"ok": True, "hz": round(hz, 1), "rpm": rpm_from_hz(hz, self.ppr),
                    "backend": name}
        return {"ok": False, "hz": None, "rpm": None,
                "why": (f"读不到 GPIO{self.gpio} 的脉冲：装 libgpiod（apt install gpiod python3-libgpiod）"
                        f"或把 zelux 加进 gpio 组"), "backend": None}


# ---------------------------------------------------------------- 外壳
class RadxaFan:
    """给 `hal/radxa.py` 用的风扇部分。方法名与 `hal/base.py` 的 `fan_*` 对齐。"""

    backend = "radxa-fan"

    def __init__(self, ctx=None, *, helper_path: str | None = None,
                 fans: tuple[dict[str, Any], ...] = FANS,
                 pulses_per_rev: int = PULSES_PER_REV) -> None:
        cfg = getattr(ctx, "config", None)
        self.log = getattr(ctx, "log", log) or log
        self.helper = FanHelper(helper_path or getattr(cfg, "fan_socket", "/run/zelux/fan.sock"))
        self.fans = [dict(f) for f in fans]
        self.pulses_per_rev = max(1, int(pulses_per_rev))

        self.curve = [list(r) for r in DEFAULT_CURVE]
        self.mode = "auto"
        self.available = False
        self.reason = "还没探测风扇控制器"
        self.hint = ""
        self.error = ""               # 最近一次操作失败的原因（区别于"没有风扇"）
        self.source = "none"          # helper | sysfs | none

        self._pct = {i: DEFAULT_PCT for i in range(len(self.fans))}
        self._rpm: dict[int, int | None] = {i: None for i in range(len(self.fans))}
        self._tach_hz: dict[int, float | None] = {i: None for i in range(len(self.fans))}
        self._target_pct = DEFAULT_PCT
        self._pwm = {i: SysfsPwm(f["pwm"]) for i, f in enumerate(self.fans)}
        self._tach = {i: Tach(f["tach"], self.pulses_per_rev)
                      for i, f in enumerate(self.fans) if f.get("tach") is not None}
        self._task: asyncio.Task | None = None
        self._closing = False

    # ---------------- 生命周期 ----------------
    async def start(self) -> None:
        # _probe 是同步方法，不能 await；否则抛 "object NoneType can't be used in
        # 'await' expression"，风扇这一块永远不可用
        self._probe()
        if self._task is None or self._task.done():
            self._closing = False
            self._task = asyncio.create_task(self._rpm_loop(), name="fan-rpm")

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
        # 不关风扇：zeluxd 重启时把风扇停掉，主控会在它启动的十几秒里过热

    def _probe(self) -> None:
        if self.helper.exists():
            self.available, self.source = True, "helper"
            self.reason = ""
            return
        writable = [i for i, pwm in self._pwm.items() if pwm.writable()]
        if writable:
            self.available, self.source = True, "sysfs"
            self.reason = ""
            self.log.warning("根助手不在，风扇走 sysfs 降级路径（编号可能对不上，注意听声音）")
            return
        self.available, self.source = False, "none"
        self.reason = ("没找到风扇控制器：/run/zelux/fan.sock 不存在，/sys/class/pwm 也写不进去")
        self.hint = "sudo systemctl restart zelux-fand；没有这个服务就把 zelux 用户加进 gpio 组"

    async def _rpm_loop(self) -> None:
        """每 2 秒测一次转速。1 秒窗口 + 2 秒周期 = 转速表是"最近的"而不是"瞬时的"。"""
        while not self._closing:
            for i in list(self._tach):
                if self._pct.get(i, 0) <= 0:
                    # 停转时不用占着 GPIO 数 1 秒，直接记 0
                    self._rpm[i], self._tach_hz[i] = 0, 0.0
                    continue
                res = await asyncio.to_thread(self._tach[i].measure, TACH_WINDOW_S)
                if res["ok"]:
                    self._rpm[i], self._tach_hz[i] = res["rpm"], res["hz"]
                else:
                    self._rpm[i], self._tach_hz[i] = None, None
                    self._reason_once(res.get("why", ""))
            await asyncio.sleep(RPM_REFRESH_S)

    def _reason_once(self, why: str) -> None:
        if why and why != getattr(self, "_last_tach_why", ""):
            self._last_tach_why = why
            self.log.warning("测速不可用：%s", why)

    def info(self) -> dict:
        return {"backend": self.backend, "source": self.source, "available": self.available,
                "socket": self.helper.path, "curve": self.curve, "mode": self.mode}

    # ---------------- 调速 ----------------
    def _fan(self, index: int) -> dict:
        try:
            return self.fans[int(index)]
        except (IndexError, ValueError, TypeError):
            raise FanUnavailable(f"没有第 {index} 号风扇（这台机器只有 {len(self.fans)} 颗）") from None

    async def set_pct(self, index: int, pct: Any) -> dict:
        """设转速。失败**不抛**：返回 {ok:false, available:false, reason}，页面照常渲染。"""
        try:
            fan = self._fan(index)
        except FanUnavailable as exc:
            return {"ok": False, "available": self.available, "reason": exc.message,
                    "index": index, "pct": clamp_pct(pct)}
        idx = int(index)
        value = clamp_pct(pct)
        duty = pct_to_duty(value)
        if not self.available:
            return {"ok": False, "available": False, "reason": self.reason,
                    "hint": self.hint, "index": idx, "pct": value}

        try:
            if self.source == "helper":
                await self.helper.call({"op": "set", "index": idx, "pwm": fan["pwm"],
                                        "duty": duty, "period": PWM_PERIOD_NS})
            else:
                await asyncio.to_thread(self._pwm[idx].apply, duty, PWM_PERIOD_NS)
        except FanUnavailable as exc:
            # 运行期失败（助手重启、sysfs 被抢）不改 available：那是"探测结论"，
            # 改了以后页面会一直显示"没有风扇"，连重试按钮都没了
            self.error = exc.message
            self.log.warning("风扇 %s 调速失败：%s", fan["name"], exc.message)
            return {"ok": False, "available": self.available, "reason": exc.message,
                    "hint": exc.hint, "index": idx, "pct": value}
        except Exception as exc:
            self.error = f"风扇 {fan['name']} 调速出错：{exc}"
            self.log.warning("%s", self.error)
            return {"ok": False, "available": self.available, "reason": self.error,
                    "index": idx, "pct": value}

        self.error = ""
        self._pct[idx] = value
        self._target_pct = value
        if value == 0:
            self._rpm[idx], self._tach_hz[idx] = 0, 0.0
        return {"ok": True, "available": True, "index": idx, "pct": value,
                "duty": duty, "period": PWM_PERIOD_NS, "source": self.source}

    async def apply_curve(self, temp: float | None) -> dict:
        pct = curve_pct(temp, self.curve)
        applied = [await self.set_pct(i, pct) for i in range(len(self.fans))]
        self.mode = "auto"
        self._target_pct = pct
        return {"ok": all(a.get("ok") for a in applied) or not self.available,
                "temp": temp, "pct": pct, "curve": [list(r) for r in self.curve],
                "applied": applied, "available": self.available, "reason": self.reason}

    # ---------------- 实例管理（fanctl 契约） ----------------
    async def create_instance(self, instance: dict[str, Any]) -> dict:
        """创建风扇实例：转发 {op:create, instance} 到 fanctl。"""
        if not self.available:
            return {"ok": False, "available": False, "reason": self.reason, "hint": self.hint}
        try:
            res = await self.helper.call({"op": "create", "instance": instance})
            return {"ok": res.get("ok", True), "available": True, **res}
        except FanUnavailable as exc:
            self.log.warning("创建风扇实例失败：%s", exc.message)
            return {"ok": False, "available": self.available, "reason": exc.message, "hint": exc.hint}

    async def update_instance(self, instance_id: str, instance: dict[str, Any]) -> dict:
        """更新风扇实例：转发 {op:update, instance_id, instance} 到 fanctl。"""
        if not self.available:
            return {"ok": False, "available": False, "reason": self.reason, "hint": self.hint}
        try:
            res = await self.helper.call({"op": "update", "instance_id": instance_id, "instance": instance})
            return {"ok": res.get("ok", True), "available": True, **res}
        except FanUnavailable as exc:
            self.log.warning("更新风扇实例失败：%s", exc.message)
            return {"ok": False, "available": self.available, "reason": exc.message, "hint": exc.hint}

    async def delete_instance(self, instance_id: str) -> dict:
        """删除风扇实例：转发 {op:delete, instance_id, confirm:true} 到 fanctl。"""
        if not self.available:
            return {"ok": False, "available": False, "reason": self.reason, "hint": self.hint}
        try:
            res = await self.helper.call({"op": "delete", "instance_id": instance_id, "confirm": True})
            return {"ok": res.get("ok", True), "available": True, **res}
        except FanUnavailable as exc:
            self.log.warning("删除风扇实例失败：%s", exc.message)
            return {"ok": False, "available": self.available, "reason": exc.message, "hint": exc.hint}

    async def set_enabled(self, instance_id: str, enabled: bool) -> dict:
        """启用/禁用风扇实例：转发 {op:enable/disable, instance_id} 到 fanctl。"""
        if not self.available:
            return {"ok": False, "available": False, "reason": self.reason, "hint": self.hint}
        try:
            res = await self.helper.call({"op": "enable" if enabled else "disable", "instance_id": instance_id})
            return {"ok": res.get("ok", True), "available": True, **res}
        except FanUnavailable as exc:
            self.log.warning("设置风扇实例状态失败：%s", exc.message)
            return {"ok": False, "available": self.available, "reason": exc.message, "hint": exc.hint}

    # ---------------- 读数 ----------------
    async def read(self, index: int) -> dict:
        """读一颗风扇。带 tach 的会真等 1 秒窗口（调用方负责 to_thread/协程等）。"""
        fan = self._fan(index)
        idx = int(index)
        rpm, hz, why = self._rpm.get(idx), self._tach_hz.get(idx), ""
        if idx in self._tach and self._pct.get(idx, 0) > 0:
            res = await asyncio.to_thread(self._tach[idx].measure, TACH_WINDOW_S)
            if res["ok"]:
                rpm, hz = res["rpm"], res["hz"]
                self._rpm[idx], self._tach_hz[idx] = rpm, hz
            else:
                why = res.get("why", "")
        return {"index": idx, "name": fan["name"], "pwm": fan["pwm"], "gpio": fan["gpio"],
                "pct": self._pct.get(idx, 0), "rpm": rpm, "tach_hz": hz,
                "source": self.source, "available": self.available,
                "reason": why or self.reason}

    def snapshot_fan(self) -> dict:
        items = []
        for i, fan in enumerate(self.fans):
            items.append({"name": fan["name"], "pwm": fan["pwm"], "gpio": fan["gpio"],
                          "pct": self._pct.get(i, 0), "rpm": self._rpm.get(i),
                          "tach_hz": self._tach_hz.get(i)})
        return {"mode": self.mode, "pct": self._target_pct, "temp": None,
                "curve": [list(r) for r in self.curve], "items": items,
                "available": self.available, "reason": self.reason, "error": self.error,
                "hint": self.hint, "source": self.source,
                "period_ns": PWM_PERIOD_NS, "freq_hz": PWM_FREQ_HZ,
                "pulses_per_rev": self.pulses_per_rev}

    # ---------------- 自检 ----------------
    async def self_test(self, index: int | None = None) -> dict:
        """全速 1 秒 → 看 rpm 是不是 > 300 → 回原值。

        这是"风扇到底转没转"唯一可靠的判据：只写 duty 而不测速，等于闭着眼修车。
        没有 tach 的那颗（风扇 1）只能给 ok:null + 人话，绝不猜。
        """
        indexes = [int(index)] if index is not None else list(range(len(self.fans)))
        results = []
        for idx in indexes:
            fan = self._fan(idx)
            prev = self._pct.get(idx, DEFAULT_PCT)
            if not self.available:
                results.append({"index": idx, "name": fan["name"], "ok": None,
                                "rpm": None, "reason": self.reason})
                continue
            try:
                await self.set_pct(idx, 100)
                await asyncio.sleep(1.0)      # 让风叶真的转起来，0.2 秒读到的还是惯性
                rec = await self.read(idx)
            finally:
                await self.set_pct(idx, prev)  # 不管成功失败都回到原来的档位
            rpm = rec.get("rpm")
            if rpm is None:
                results.append({"index": idx, "name": fan["name"], "ok": None, "rpm": None,
                                "reason": "这颗没有测速线，只能靠声音/手感判断（4pin 才有 tach）"})
            elif rpm > TACH_MIN_RPM:
                results.append({"index": idx, "name": fan["name"], "ok": True, "rpm": rpm,
                                "reason": f"全速 rpm={rpm} > {TACH_MIN_RPM}，风扇在转"})
            else:
                results.append({"index": idx, "name": fan["name"], "ok": False, "rpm": rpm,
                                "reason": (f"全速 rpm={rpm} ≤ {TACH_MIN_RPM}：风扇没转、卡住、"
                                           f"或测速线（GPIO{fan.get('tach')}）没插好")})
        ok = all(r["ok"] is not False for r in results)
        return {"ok": ok, "available": self.available, "reason": self.reason,
                "items": results}
