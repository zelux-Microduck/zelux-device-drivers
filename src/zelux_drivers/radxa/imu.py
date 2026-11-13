"""Radxa Zero 3W 的 IMU：板上没有，靠外部 I²C 模块（QMI8658 / MPU6050 / ICM42688 最常见）。

三件必须说清楚的事：

1) **探测顺序**是硬编码的（0x6B QMI8658 → 0x68/0x69 MPU6050 → ICM42688 → 0x6A → Linux IIO）。
   为什么按地址而不是按芯片：I²C 上只能先知道"某个地址有人应答"，只有读到 WHO_AM_I
   才敢确定是谁。所以每个候选都是"打开总线 → 读 ID → 对得上才初始化"。
2) **不依赖 smbus**：同样是精简系统少一个依赖。`/dev/i2c-N` 用 ioctl(0x0703) 选从机，
   读一次寄存器就是 write(reg) + read(n)，这是 i2c-dev 的标准用法。
3) **yaw 会漂**：没有磁力计，陀螺积分一小时飘几十度是物理决定的。这里如实标 `yaw_drift`，
   页面就该把它显示成"参考值"，不要拿它做定向往复。

滤波：互补滤波（陀螺积分 + 加速度计重力方向校正，α=0.98）。
静止时加速度计绝对准但有振动噪声，运动时陀螺准但会漂——互补滤波就是把两者的
"低频/高频"各取一半，代码只有三行但足够 200 Hz 环用。
"""
from __future__ import annotations

import asyncio
import collections
import json
import logging
import math
import os
import struct
import threading
import time
from pathlib import Path

log = logging.getLogger("zeluxd.imu")

ALPHA = 0.98                    # 互补滤波系数：越接近 1 越信陀螺
G = 9.80665
SAMPLE_HZ = 100
HISTORY = 200                   # 标定向导要回看最近几秒，200 帧 = 2 s

# 探测候选：(I²C 地址, 备注)。地址顺序就是探测顺序。
I2C_CANDIDATES = (0x6B, 0x68, 0x69, 0x6A)


# ---------------------------------------------------------------- 纯函数（可单测）
def wrap180(deg: float) -> float:
    """角度折到 (-180, 180]，避免 yaw 越滚越大。"""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def accel_to_roll_pitch(ax: float, ay: float, az: float) -> tuple[float, float]:
    """由重力方向算横滚/俯仰（度）。静止时这是绝对值，运动中会被线性加速度污染。"""
    if ax == 0.0 and ay == 0.0 and az == 0.0:
        return 0.0, 0.0
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return roll, pitch


def complementary(roll: float, pitch: float, acc_roll: float, acc_pitch: float,
                  gx: float, gy: float, dt: float, alpha: float = ALPHA) -> tuple[float, float]:
    """陀螺积分走高频、加速度计拉回低频；dt 单位秒，角度单位度。"""
    roll = alpha * (roll + gx * dt) + (1.0 - alpha) * acc_roll
    pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * acc_pitch
    pitch = max(-89.0, min(89.0, pitch))      # 万向节锁附近没有意义，直接夹住
    return roll, pitch


def yaw_step(yaw: float, gz: float, dt: float) -> float:
    return wrap180(yaw + gz * dt)


def control_orientation(roll: float, pitch: float, yaw: float,
                        gx: float, gy: float, gz: float) -> dict:
    """Convert UI degrees into robotd's rad/s, gravity and wxyz contract."""
    r, p, y = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(r / 2.0), math.sin(r / 2.0)
    cp, sp = math.cos(p / 2.0), math.sin(p / 2.0)
    cy, sy = math.cos(y / 2.0), math.sin(y / 2.0)
    quat = [cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr]
    gravity = [math.sin(p), -math.sin(r) * math.cos(p),
               -math.cos(r) * math.cos(p)]
    return {"gyro_rad_s": [math.radians(gx), math.radians(gy), math.radians(gz)],
            "gravity": gravity, "quaternion_wxyz": quat}


def spread(values: list[float]) -> float:
    """峰峰值。向导用它判断"到底动不动"，比标准差更直观。"""
    return (max(values) - min(values)) if values else 0.0


# ---------------------------------------------------------------- 芯片驱动
class I2C:
    """i2c-dev 裸访问：ioctl 选从机，然后 write(reg)+read(n)。"""

    I2C_SLAVE = 0x0703

    def __init__(self, bus: int, addr: int) -> None:
        import fcntl

        self.path = f"/dev/i2c-{bus}"
        self.addr = addr
        try:
            self.fd = os.open(self.path, os.O_RDWR)
        except OSError as exc:
            raise OSError(f"打不开 {self.path}：{exc.strerror or exc}") from None
        try:
            fcntl.ioctl(self.fd, self.I2C_SLAVE, addr)
        except OSError:
            os.close(self.fd)
            raise OSError(f"{self.path} 上没有 0x{addr:02X} 这个设备") from None

    def read(self, reg: int, n: int) -> bytes:
        os.write(self.fd, bytes((reg & 0xFF,)))
        data = os.read(self.fd, n)
        if len(data) != n:
            raise OSError(f"0x{self.addr:02X} 只回了 {len(data)}/{n} 字节")
        return data

    def write(self, reg: int, data: bytes) -> None:
        os.write(self.fd, bytes((reg & 0xFF,)) + bytes(data))

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass


class Chip:
    name = "unknown"
    i2c: I2C | None = None

    def read_raw(self) -> tuple[float, float, float, float, float, float]:
        """返回 (ax, ay, az) m/s² 与 (gx, gy, gz) °/s。"""
        raise NotImplementedError

    def close(self) -> None:
        # I²C 芯片都要放掉 fd，否则重启采样会攒一堆句柄
        if self.i2c is not None:
            self.i2c.close()
            self.i2c = None


class QMI8658(Chip):
    """0x6B。WHO_AM_I(0x00)=0x05，数据从 0x35 起 12 字节小端。"""

    name = "qmi8658"
    WHO = 0x05
    A_SCALE = 4.0 * G / 32768.0        # ±4 g
    G_SCALE = 2048.0 / 32768.0         # ±2048 °/s

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        who = i2c.read(0x00, 1)[0]
        if who != self.WHO:
            raise OSError(f"0x6B 上不是 QMI8658（WHO_AM_I=0x{who:02X}）")
        i2c.write(0x02, bytes((0x40,)))            # CTRL1：地址自增，小端
        i2c.write(0x03, bytes((0x17,)))            # CTRL2：±4 g，250 Hz
        i2c.write(0x04, bytes((0x47,)))            # CTRL3：±2048 °/s，250 Hz
        i2c.write(0x08, bytes((0x03,)))            # CTRL7：开加速度 + 开陀螺
        time.sleep(0.02)

    def read_raw(self):
        d = self.i2c.read(0x35, 12)
        ax, ay, az, gx, gy, gz = struct.unpack("<6h", d)
        return (ax * self.A_SCALE, ay * self.A_SCALE, az * self.A_SCALE,
                gx * self.G_SCALE, gy * self.G_SCALE, gz * self.G_SCALE)


class MPU6050(Chip):
    """0x68/0x69/0x6A。WHO_AM_I(0x75)=0x68，数据从 0x3B 起大端。"""

    name = "mpu6050"
    WHO = (0x68, 0x98)                  # 0x98 是常见的兼容克隆
    A_SCALE = 8.0 * G / 32768.0         # ±8 g
    G_SCALE = 500.0 / 32768.0           # ±500 °/s

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        who = i2c.read(0x75, 1)[0]
        if who not in self.WHO:
            raise OSError(f"0x{i2c.addr:02X} 上不是 MPU6050（WHO_AM_I=0x{who:02X}）")
        i2c.write(0x6B, bytes((0x80,)))            # 复位
        time.sleep(0.05)
        i2c.write(0x6B, bytes((0x01,)))            # 时钟用 PLL
        i2c.write(0x19, bytes((0x09,)))            # 采样分频 → 100 Hz
        i2c.write(0x1A, bytes((0x03,)))            # 低通 44 Hz
        i2c.write(0x1B, bytes((0x08,)))            # 陀螺 ±500 °/s
        i2c.write(0x1C, bytes((0x10,)))            # 加速度 ±8 g
        time.sleep(0.05)

    def read_raw(self):
        d = self.i2c.read(0x3B, 14)
        ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", d)
        return (ax * self.A_SCALE, ay * self.A_SCALE, az * self.A_SCALE,
                gx * self.G_SCALE, gy * self.G_SCALE, gz * self.G_SCALE)


class ICM42688(Chip):
    """0x68/0x69。WHO_AM_I(0x75)=0x47，加速度从 0x1F、陀螺从 0x25 起大端。"""

    name = "icm42688"
    WHO = 0x47
    A_SCALE = 16.0 * G / 32768.0        # ±16 g
    G_SCALE = 2000.0 / 32768.0          # ±2000 °/s

    def __init__(self, i2c: I2C) -> None:
        self.i2c = i2c
        who = i2c.read(0x75, 1)[0]
        if who != self.WHO:
            raise OSError(f"0x{i2c.addr:02X} 上不是 ICM42688（WHO_AM_I=0x{who:02X}）")
        i2c.write(0x11, bytes((0x01,)))            # DEVICE_CONFIG：软复位
        time.sleep(0.02)
        i2c.write(0x4E, bytes((0x0F,)))            # PWR_MGMT0：加速度 + 陀螺低噪
        i2c.write(0x4F, bytes((0x08,)))            # GYRO_CONFIG0：±2000 °/s，1 kHz
        i2c.write(0x50, bytes((0x08,)))            # ACCEL_CONFIG0：±16 g，1 kHz
        time.sleep(0.05)

    def read_raw(self):
        a = self.i2c.read(0x1F, 6)
        g = self.i2c.read(0x25, 6)
        ax, ay, az = struct.unpack(">3h", a)
        gx, gy, gz = struct.unpack(">3h", g)
        return (ax * self.A_SCALE, ay * self.A_SCALE, az * self.A_SCALE,
                gx * self.G_SCALE, gy * self.G_SCALE, gz * self.G_SCALE)


class IIOChip(Chip):
    """内核已经把 IMU 认成 IIO 设备时走 sysfs，最省事也最难出错。"""

    name = "iio"

    def __init__(self, root: Path) -> None:
        self.root = root
        try:
            self.name = (root / "name").read_text("utf-8").strip() or "iio"
        except OSError:
            pass
        self.a_scale = self._scale("in_accel_scale", 1.0)
        self.g_scale = self._scale("in_gyro_scale", 1.0)   # IIO 陀螺单位是 rad/s

    def _scale(self, fname: str, default: float) -> float:
        try:
            return float((self.root / fname).read_text("utf-8").strip())
        except (OSError, ValueError):
            return default

    def _int(self, fname: str) -> float:
        try:
            return float((self.root / fname).read_text("utf-8").strip())
        except (OSError, ValueError):
            return 0.0

    def read_raw(self):
        a = [self._int(f"in_accel_{ax}_raw") * self.a_scale for ax in ("x", "y", "z")]
        g = [math.degrees(self._int(f"in_gyro_{ax}_raw") * self.g_scale) for ax in ("x", "y", "z")]
        return (a[0], a[1], a[2], g[0], g[1], g[2])

    def close(self) -> None:
        pass


def detect_i2c(bus: int) -> Chip | None:
    """按地址挨个试；只有 WHO_AM_I 对得上才认。失败的 fd 一定关掉，别攒句柄。"""
    for addr in I2C_CANDIDATES:
        for cls in (QMI8658, MPU6050, ICM42688):
            handle = None
            try:
                handle = I2C(bus, addr)
                chip = cls(handle)
                log.info("IMU：%s @ i2c-%d 0x%02X", chip.name, bus, addr)
                return chip
            except Exception:
                if handle is not None:
                    handle.close()
                continue
    return None


def detect_iio() -> Chip | None:
    base = Path("/sys/bus/iio/devices")
    if not base.exists():
        return None
    for dev in sorted(base.glob("iio:device*")):
        try:
            if not (dev / "in_accel_x_raw").exists():
                continue
            if not (dev / "in_gyro_x_raw").exists():
                continue
            chip = IIOChip(dev)
            log.info("IMU：走 IIO（%s，%s）", chip.name, dev)
            return chip
        except Exception:
            continue
    return None


# ---------------------------------------------------------------- 异步外壳
class RadxaIMU:
    backend = "radxa-imu"

    def __init__(self, ctx) -> None:
        self.ctx = ctx
        self.log = getattr(ctx, "log", log) or log
        self.bus = int(getattr(ctx.config, "i2c_bus", 3))
        self._chip: Chip | None = None
        self._task: asyncio.Task | None = None
        self._io_lock = threading.Lock()       # 采样线程与 bias 标定可能同时读芯片
        self._frames: collections.deque = collections.deque(maxlen=HISTORY)
        self._bias = {"gx": 0.0, "gy": 0.0, "gz": 0.0}
        self._cal = {"roll_offset": 0.0, "pitch_offset": 0.0, "roll_sign": 1, "pitch_sign": 1}
        self._state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._gyro = [0.0, 0.0, 0.0]       # 去偏后的 °/s；对 robotd 发布时换成 rad/s
        self._hz = 0.0
        self._t_last = 0.0
        self._cache: dict = {"ok": False, "reason": "IMU 还没起来"}
        self._load_calib()

    # ---------------- 生命周期 ----------------
    async def start(self) -> None:
        chip = await asyncio.to_thread(self._detect)
        if chip is None:
            self._cache = {"ok": False, "reason": "没找到 IMU，检查 I²C 接线"}
            self.log.warning("没找到 IMU（i2c-%d 上没有 0x6B/0x68/0x69/0x6A 应答，IIO 也没有）", self.bus)
            return
        self._chip = chip
        self._task = asyncio.create_task(self._loop())

    async def stop(self) -> None:
        task, self._task = self._task, None
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        if self._chip:
            self._chip.close()
            self._chip = None

    def _detect(self) -> Chip | None:
        try:
            chip = detect_i2c(self.bus)
        except Exception as exc:
            self.log.warning("I²C 探测出错：%s", exc)
            chip = None
        if chip is None:
            chip = detect_iio()
        return chip

    def info(self) -> dict:
        return {"backend": self.backend, "chip": getattr(self._chip, "name", None),
                "bus": self.bus, "hz": round(self._hz, 1), "bias": dict(self._bias)}

    # ---------------- 采样 ----------------
    async def _loop(self) -> None:
        prev = time.monotonic()
        n, t_win = 0, prev
        while True:
            t0 = time.monotonic()
            try:
                raw = await asyncio.to_thread(self._read)
            except Exception as exc:
                self._fail(f"读 IMU 失败：{exc}")
                await asyncio.sleep(0.5)
                continue
            now = time.monotonic()
            dt = now - prev
            prev = now
            if dt <= 0 or dt > 0.5:            # 首次或卡顿：这一拍不积分
                dt = 0.01
            self._integrate(raw, dt)
            n += 1
            if now - t_win >= 1.0:             # 每秒汇报一次实际频率，不靠标称值糊弄
                self._hz = n / (now - t_win)
                n, t_win = 0, now
            await asyncio.sleep(max(0.0, 1.0 / SAMPLE_HZ - (time.monotonic() - t0)))

    def _read(self):
        with self._io_lock:
            if not self._chip:
                raise OSError("芯片已关闭")
            return self._chip.read_raw()

    def _integrate(self, raw, dt: float) -> None:
        ax, ay, az, gx, gy, gz = raw
        gx -= self._bias["gx"]
        gy -= self._bias["gy"]
        gz -= self._bias["gz"]
        acc_r, acc_p = accel_to_roll_pitch(ax, ay, az)
        r, p = complementary(self._state["roll"], self._state["pitch"], acc_r, acc_p, gx, gy, dt)
        yaw = yaw_step(self._state["yaw"], gz, dt)
        self._state = {"roll": r, "pitch": p, "yaw": yaw}
        self._gyro = [gx, gy, gz]
        self._t_last = time.time()
        self._frames.append({"t": self._t_last, "roll": round(r, 3), "pitch": round(p, 3),
                             "yaw": round(yaw, 3), "gz": round(gz, 3),
                             "ax": round(ax, 3), "ay": round(ay, 3), "az": round(az, 3)})
        self._cache = self._build()

    def _build(self) -> dict:
        st = self._state
        cal = self._cal
        roll = wrap180(st["roll"] * cal["roll_sign"] + cal["roll_offset"])
        pitch = wrap180(st["pitch"] * cal["pitch_sign"] + cal["pitch_offset"])
        control = control_orientation(roll, pitch, st["yaw"], *self._gyro)
        return {
            "ok": True,
            "roll": round(roll, 2),
            "pitch": round(pitch, 2),
            "yaw": round(st["yaw"], 2),
            "gz": round(self._gyro[2], 2),
            **control,
            "chip": getattr(self._chip, "name", "?"),
            "calibrated": bool(self.ctx.state.get("imu_calibrated")),
            "yaw_drift": True,               # 没有磁力计，这是物理事实，页面要标"参考"
            "hz": round(self._hz, 1),
            "age_ms": int((time.time() - self._t_last) * 1000) if self._t_last else None,
            "bias": {k: round(v, 3) for k, v in self._bias.items()},
            "install": {"roll_sign": cal["roll_sign"], "pitch_sign": cal["pitch_sign"]},
        }

    def _fail(self, msg: str) -> None:
        # 采样失败要留在快照里给页面看，但不要把上一帧的姿态抹掉
        self._cache = {**self._cache, "ok": False, "reason": msg}

    # ---------------- 对外 ----------------
    def snapshot_imu(self) -> dict:
        return self._cache

    def snapshot(self) -> dict:
        return self._cache

    def recent(self, n: int = 100) -> list[dict]:
        n = max(0, min(int(n), HISTORY))
        items = list(self._frames)
        return items[-n:] if n else []

    def clear_history(self) -> None:
        self._frames.clear()

    async def imu_sample(self) -> dict:
        """页面要"现在这一帧"，给的就是缓存里最新的（10 Hz 状态流本来也够看）。"""
        snap = dict(self._cache)
        snap["recent"] = len(self._frames)
        return snap

    async def estimate_bias(self, seconds: float = 3.0) -> dict:
        """静置估陀螺零偏。鸭子放在桌上、别碰，几秒钟的事。"""
        if not self._chip:
            return {"ok": False, "reason": "没有 IMU，先解决接线"}
        seconds = max(0.5, min(10.0, float(seconds)))

        def _do() -> dict:
            gx = gy = gz = 0.0
            xs: list[float] = []
            ys: list[float] = []
            zs: list[float] = []
            n = 0
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                with self._io_lock:
                    _ax, _ay, _az, a, b, c = self._chip.read_raw()
                gx += a
                gy += b
                gz += c
                xs.append(a)
                ys.append(b)
                zs.append(c)
                n += 1
                time.sleep(0.01)
            if not n:
                return {"ok": False, "reason": "一帧都没采到"}
            return {"ok": True, "n": n, "gx": gx / n, "gy": gy / n, "gz": gz / n,
                    "spread": round(max(spread(xs), spread(ys), spread(zs)), 2)}

        res = await asyncio.to_thread(_do)
        if not res.get("ok"):
            return res
        self._bias = {"gx": res["gx"], "gy": res["gy"], "gz": res["gz"]}
        self.log.info("陀螺零偏：gx=%.2f gy=%.2f gz=%.2f °/s（%d 帧，峰峰 %.2f）",
                      res["gx"], res["gy"], res["gz"], res["n"], res["spread"])
        if res["spread"] > 4.0:
            res["warning"] = "采的时候还在动，零偏不准，重新静置几秒"
        return {**res, "bias": {k: round(v, 3) for k, v in self._bias.items()}}

    # ---------------- 标定 ----------------
    def _calib_path(self) -> Path:
        d: Path = self.ctx.config.state_dir
        return d / "imu_calib.json"

    def _load_calib(self) -> None:
        try:
            data = json.loads(self._calib_path().read_text("utf-8"))
        except (OSError, ValueError):
            return
        for k in self._cal:
            if k in data:
                self._cal[k] = data[k]
        self.ctx.state["imu_calibrated"] = True
        log.info("已载入 IMU 标定：roll%+.1f° pitch%+.1f° 安装方向 sign=(%d,%d)",
                 self._cal["roll_offset"], self._cal["pitch_offset"],
                 self._cal["roll_sign"], self._cal["pitch_sign"])

    def apply_calibration(self, roll_offset: float = 0.0, pitch_offset: float = 0.0,
                          roll_sign: int = 1, pitch_sign: int = 1) -> None:
        """偏置 + 安装方向一起生效：输出 = 原始 × 方向 + 偏置。"""
        self._cal = {"roll_offset": float(roll_offset), "pitch_offset": float(pitch_offset),
                     "roll_sign": -1 if int(roll_sign) < 0 else 1,
                     "pitch_sign": -1 if int(pitch_sign) < 0 else 1}
        self._cache = self._build()

    @property
    def calibration(self) -> dict:
        return dict(self._cal)
