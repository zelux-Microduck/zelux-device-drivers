"""Radxa Zero 3W 的舵机总线：15 颗 Feetech STS/SCS，`/dev/ttyS2` @ 1 Mbps，TTL 半双工。

为什么不用 pyserial：
镜像里是精简 Debian，装机现场经常没网，少一个 pip 依赖就少一类"装不上"；
`termios` 是 libc 自带的老接口，1 Mbps 非标波特率也支持，这个协议够用。

为什么拆三层（也为了能本机跑测试）：
* `checksum()/frame()/sync_write_frame()` 纯函数 —— 不碰硬件就能验证帧格式；
* `ServoProto` 只依赖 `send/recv` 两个方法，真串口和内存假总线都能插进来，
  "发一帧、收一帧、超时重试、半双工回声"的脏活全在这里；
* `RadxaServo` 是给 HAL 用的异步外壳：一把 `asyncio.Lock` + `to_thread`，
  保证 1 Mbps 的总线 IO 不会堵住 10 Hz 的状态总线。

半双工为什么要挑帧而不是"先 echo 后回包"：
不同 TTL 转接板有的回显有的不回显，回显又可能和应答粘在一起（甚至断开在两段）。
所以这里不做假设：把读到的字节喂进缓冲区，按 `FF FF id len err ... chk` 逐字节找，
用"长度 + 校验和 + err==0"三重条件把回显和应答区分开。
"""
from __future__ import annotations

import asyncio
import logging
import math
import os
import select
import struct
import time
from typing import Iterable

from ..errors import hardware
from ..base import JOINT_IDS

log = logging.getLogger("zeluxd.servo")

# ---------------------------------------------------------------- 协议常量
PING, READ, WRITE, REG_WRITE, ACTION, RESET, SYNC_WRITE = 1, 2, 3, 4, 5, 6, 0x83
BROADCAST = 0xFE
CENTER = 2048
FULL_TURN = 4096
TICKS_PER_RAD = FULL_TURN / (2 * math.pi)      # 651.8986 步/弧度
MAX_SPEED = 3400                               # STS3215 @ 1 Mbps 的上限，步/秒

# 寄存器表：名字 → (起始地址, 字节数)。
#
# 说明两点，都是真机上会咬人的地方：
# 1) 现值区：位置 0–4095 必须占 2 字节，所以 56/57 一定是位置，速度/负载/电压/温度
#    只能顺延成 58/60/62/63。契约里写的 57/58/59/60 是凭记忆抄的表，自相矛盾（位置
#    会被挤成 1 字节）。接管后跑一次 `read_all()`，看电压是不是 7.8 V 左右即可确认。
# 2) 0 位偏移：这里用 31（与契约一致）；部分 STS 固件是 30。真机若出现"改 0 位没反应"，
#    只改这一行，别改逻辑。
REG: dict[str, tuple[int, int]] = {
    "id": (5, 1),
    "baud": (6, 1),
    "response": (8, 1),
    "min_angle": (9, 2),
    "max_angle": (11, 2),
    "phase": (18, 1),        # bit0=方向反转，bit2/3=速度单位（日志里 18=52 → 1 步/秒）
    "protect": (19, 1),      # 卸载条件（堵转保护开关）
    "p": (21, 1),
    "d": (22, 1),
    "i": (23, 1),
    "offset": (31, 2),
    "mode": (33, 1),
    "torque": (40, 1),
    "goal_pos": (41, 2),
    "goal_speed": (43, 2),
    "pos": (56, 2),
    "speed": (58, 2),
    "load": (60, 2),
    "volt": (62, 1),
    "temp": (63, 1),
}

# 地址 → 字节数（写单个寄存器时用它决定发 1 字节还是 2 字节）
ADDR_WIDTH: dict[int, int] = {}
for _name, (_addr, _w) in REG.items():
    ADDR_WIDTH[_addr] = _w

# 现值区块：一次 READ 拿回 pos/speed/load/volt/temp，省 4 次往返
PRESENT_ADDR, PRESENT_N = REG["pos"][0], 8

# 危险寄存器（改了要人确认）→ 后果。文案要能让人在按下确认前手一抖。
DANGER_TEXT: dict[int, str] = {
    REG["id"][0]: "改 ID 后这颗就再也找不到了，只能单独接上、用新 ID 重新扫",
    REG["baud"][0]: "改了波特率整条总线立刻失联，得单颗接到 1 Mbps 以外的口上救回来",
    REG["response"][0]: "应答级别改成不回复，之后所有读取都会超时（包括这次）",
    REG["min_angle"][0]: "限位设反或设太窄，关节会直接顶死在机构上，堵转几秒就烫手",
    REG["max_angle"][0]: "限位设反或设太窄，关节会直接顶死在机构上，堵转几秒就烫手",
    REG["phase"][0]: "方向反了姿势就是镜像的，两条腿会互相打架、当场翻倒",
    REG["protect"][0]: "关掉卸载保护后堵转不卸力，会烧线圈（也可能烧驱动板）",
    REG["mode"][0]: "运行模式不对时，位置指令会变成电机一直转，鸭子会自己走掉",
}


# ---------------------------------------------------------------- 帧（纯函数）
def checksum(body: bytes) -> int:
    """SCS 校验：`~(id + len + instr + params) & 0xFF`。"""
    return (~sum(body)) & 0xFF


def frame(sid: int, instr: int, params: bytes = b"") -> bytes:
    """组一帧：`FF FF id len instr params... chk`，len = 参数数 + 2（instr 与 chk）。"""
    body = bytes((sid & 0xFF, len(params) + 2, instr & 0xFF)) + bytes(params)
    return b"\xff\xff" + body + bytes((checksum(body),))


def sync_write_frame(addr: int, width: int, items: Iterable[tuple[int, bytes]]) -> bytes:
    """STS 的 SYNC_WRITE：2 字节地址 + 2 字节长度（不是 SCS 的 1+1）。

    一帧写满 15 颗：15 颗 × (1 字节 ID + width 字节数据)，总线只走一次，
    这是 200 Hz 控制环能做起来的前提——逐颗写的话光串口就要 40 ms。
    """
    params = struct.pack("<HH", addr, width)
    n = 0
    for sid, data in items:
        if len(data) != width:
            raise ValueError(f"SYNC_WRITE 第 {sid} 颗数据长度 {len(data)} ≠ {width}")
        params += bytes((sid & 0xFF,)) + bytes(data)
        n += 1
    if n == 0:
        raise ValueError("SYNC_WRITE 没有目标舵机")
    return frame(BROADCAST, SYNC_WRITE, params)


def rad_to_tick(rad: float) -> int:
    """弧度 → 刻度。中位 2048，一整圈 4096 步。"""
    return max(0, min(FULL_TURN - 1, int(round(CENTER + rad * TICKS_PER_RAD))))


def tick_to_rad(tick: float) -> float:
    return (float(tick) - CENTER) / TICKS_PER_RAD


def encode_speed(speed: int) -> bytes:
    """目标速度：bit15 是方向位，低 15 位是绝对值（步/秒）。

    0 表示"不限速"，所以默认给它一个限幅值——15 颗一起全速冲姿势会把鸭子甩飞。
    """
    v = int(speed)
    v = -MAX_SPEED if v < -MAX_SPEED else MAX_SPEED if v > MAX_SPEED else v
    raw = (0x8000 | -v) if v < 0 else v
    return struct.pack("<H", raw & 0xFFFF)


def decode_speed(raw: int) -> int:
    return -(raw & 0x7FFF) if raw & 0x8000 else (raw & 0x7FFF)


def decode_load(raw: int) -> float:
    """负载：bit10 是方向，低 10 位是 0.1% 单位；返回带符号的百分比。"""
    mag = raw & 0x03FF
    pct = mag / 10.0
    return -pct if raw & 0x0400 else pct


def _le16(buf: bytes, off: int = 0) -> int:
    return buf[off] | (buf[off + 1] << 8)


def encode_reg(value: int, width: int) -> bytes:
    """负值（0 位偏移、反向速度）按 16 位补码发。"""
    if width == 2:
        return struct.pack("<H", value & 0xFFFF)
    return bytes((value & 0xFF,))


# ---------------------------------------------------------------- 传输层
class SerialLink:
    """termios 裸串口。raw、8N1、VMIN=0/VTIME=1，读超时靠 select。"""

    def __init__(self, port: str, baud: int = 1_000_000) -> None:
        import termios  # 只在真机导入：本机（Windows/无硬件）也能 import 整个模块跑单测

        try:
            self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except FileNotFoundError:
            raise hardware(f"打不开 {port}：设备不存在，先确认 TTL 线插在金手指这一侧") from None
        except PermissionError:
            raise hardware(f"没有 {port} 的权限，把 zelux 用户加进 dialout 组后重登") from None
        except OSError as exc:
            raise hardware(f"打不开 {port}：{exc.strerror or exc}") from None

        self.port = port
        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            os.close(self.fd)
            raise hardware(f"内核不支持 {baud} 波特率，检查串口驱动是不是走了 16550")
        try:
            iflag, oflag, cflag, lflag, _is, _os, cc = termios.tcgetattr(self.fd)
            iflag = 0                                   # 不要软流控、不要 CR/LF 翻译
            oflag = 0
            lflag = 0
            cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.CRTSCTS)
            cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
            cc = list(cc)
            cc[termios.VMIN] = 0                        # 非阻塞读，等多久由 select 决定
            cc[termios.VTIME] = 1
            termios.tcsetattr(self.fd, termios.TCSANOW,
                              [iflag, oflag, cflag, lflag, speed, speed, cc])
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception as exc:
            os.close(self.fd)
            raise hardware(f"{port} 配置失败：{exc}") from None

    # -- 与 FakeLink 相同的三个方法 --
    def send(self, data: bytes) -> None:
        try:
            while data:
                n = os.write(self.fd, data)
                data = data[n:]
        except OSError as exc:
            raise hardware(f"写 {self.port} 失败：{exc.strerror or exc}") from None

    def recv(self, timeout: float) -> bytes:
        if timeout > 0:
            r, _, _ = select.select([self.fd], [], [], timeout)
            if not r:
                return b""
        try:
            return os.read(self.fd, 64)
        except BlockingIOError:
            return b""
        except OSError:
            return b""

    def flush_input(self) -> None:
        try:
            import termios

            termios.tcflush(self.fd, termios.TCIFLUSH)
        except Exception:
            pass

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass


class FakeLink:
    """内存假总线（`ZELUX_SERVO_SIM=1`）：接口和 SerialLink 一样，用来跑单测与本机演示。

    它真的会回帧、真的按寄存器表存值（含 2 字节位置、16 位补码偏移），
    所以协议层的 bug（长度算错、校验错、回显没滤掉）在这里就能暴露。
    """

    def __init__(self, ids: Iterable[int] | None = None, echo: bool = True) -> None:
        self.ids = list(ids if ids is not None else JOINT_IDS)
        self.echo = echo                     # 多数 TTL 转接板会回显，默认打开才贴近真机
        self.regs: dict[int, dict[int, int]] = {sid: self._defaults(sid) for sid in self.ids}
        self.frames: list[bytes] = []        # 收到的原始帧，测试断言用
        self.closed = False
        self._rx = bytearray()

    @staticmethod
    def _defaults(sid: int) -> dict[int, int]:
        return {5: sid, 6: 7, 8: 1, 9: 0, 11: 4095, 18: 52, 19: 52,
                21: 16, 22: 32, 23: 8, 31: 0, 33: 0, 40: 0,
                41: CENTER, 43: 0, 56: CENTER, 58: 0, 60: 0, 62: 78, 63: 32}

    # -- 传输接口 --
    def send(self, data: bytes) -> None:
        self.frames.append(bytes(data))
        if self.echo:
            self._rx += data
        buf = bytearray(data)
        i = 0
        while i + 4 <= len(buf):
            if buf[i] != 0xFF or buf[i + 1] != 0xFF:
                i += 1
                continue
            total = 4 + buf[i + 3]
            if len(buf) < i + total:
                break
            sid, instr = buf[i + 2], buf[i + 4]
            params = bytes(buf[i + 5:i + total - 1])
            i += total
            self._handle(sid, instr, params)

    def recv(self, timeout: float) -> bytes:
        if not self._rx and timeout > 0:
            time.sleep(min(timeout, 0.002))
        out = bytes(self._rx)
        self._rx.clear()
        return out

    def flush_input(self) -> None:
        self._rx.clear()

    def close(self) -> None:
        self.closed = True

    # -- 假固件 --
    def _mem(self, sid: int) -> dict[int, int]:
        mem: dict[int, int] = {}
        for addr, width in ADDR_WIDTH.items():
            v = self.regs[sid].get(addr, 0) & 0xFFFF
            for k in range(width):
                mem[addr + k] = (v >> (8 * k)) & 0xFF
        return mem

    def _poke(self, sid: int, addr: int, data: bytes) -> None:
        mem = self._mem(sid)
        for k, b in enumerate(data):
            mem[addr + k] = b
        for a, w in ADDR_WIDTH.items():
            if a >= addr + len(data) or a + w <= addr:
                continue
            v = 0
            for k in range(w):
                v |= mem.get(a + k, 0) << (8 * k)
            self.regs[sid][a] = v

    def _reply(self, sid: int, data: bytes, err: int = 0) -> None:
        if sid not in self.regs:
            return
        if self.regs[sid].get(8, 1) == 0:      # 应答级别 0 = 哑巴
            return
        body = bytes((sid, len(data) + 2, err)) + data
        self._rx += b"\xff\xff" + body + bytes((checksum(body),))

    def _handle(self, sid: int, instr: int, p: bytes) -> None:
        if instr == PING:
            self._reply(sid, b"")
        elif instr == READ and len(p) >= 2:
            mem = self._mem(sid) if sid in self.regs else {}
            self._reply(sid, bytes(mem.get(p[0] + k, 0) for k in range(p[1])))
        elif instr == WRITE and len(p) >= 1 and sid in self.regs:
            self._poke(sid, p[0], p[1:])
            self._reply(sid, b"")
        elif instr == SYNC_WRITE and len(p) >= 4:
            addr, width = _le16(p, 0), _le16(p, 2)
            body = p[4:]
            for k in range(0, len(body) - width, width + 1):
                target = body[k]
                if target in self.regs:
                    self._poke(target, addr, body[k + 1:k + 1 + width])
        elif instr == RESET and sid in self.regs:
            self.regs[sid] = self._defaults(sid)
        # ACTION 无应答，符合协议

    # -- 测试辅助 --
    def settle(self) -> None:
        """模拟"舵机走到目标位置"：让现值跟上目标，测姿势时可以看最终落点。"""
        for sid, regs in self.regs.items():
            regs[56] = regs.get(41, CENTER)


# ---------------------------------------------------------------- 协议客户端
class ServoProto:
    """同步的"发一帧收一帧"。所有超时/重试/回显逻辑都在这里，没有 asyncio。"""

    def __init__(self, link, retries: int = 2, timeout: float = 0.02) -> None:
        self.link = link
        self.retries = max(1, int(retries))     # 默认 2 次：一次是常态，第二次是噪声
        self.timeout = timeout
        self._buf = bytearray()
        self.errors = 0

    # -- 原始收发 --
    def _scan(self, sid: int, n_params: int) -> bytes | None:
        """在缓冲里挑一帧合法应答；返回 `err + params`，找不到返回 None。"""
        buf = self._buf
        i = 0
        need = n_params + 2                     # len = 参数数 + err + chk
        while i + 4 <= len(buf):
            if buf[i] != 0xFF or buf[i + 1] != 0xFF:
                i += 1
                continue
            if buf[i + 3] != need or buf[i + 2] != sid:
                i += 1
                continue
            total = 4 + need
            if len(buf) < i + total:
                return None                     # 还没收全，等下一轮
            body = bytes(buf[i + 2:i + total - 1])
            if checksum(body) != buf[i + total - 1] or body[2] != 0:
                # 校验不过，或 err≠0：典型就是被回显的那一帧请求（err 位其实是 instr）
                i += 1
                continue
            del buf[:i + total]
            return bytes(body[2:])
        del buf[:i]
        return None

    def _recv_frame(self, sid: int, n: int) -> bytes | None:
        deadline = time.monotonic() + self.timeout
        while True:
            got = self._scan(sid, n)
            if got is not None:
                return got
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            chunk = self.link.recv(min(left, 0.01))
            if chunk:
                self._buf += chunk
                if len(self._buf) > 512:
                    del self._buf[:-64]
            else:
                time.sleep(min(0.001, left))

    def transact(self, sid: int, instr: int, params: bytes = b"", n_params: int = 0) -> bytes | None:
        for _ in range(self.retries):
            self._buf.clear()                   # 上一次超时的残包没有价值
            self.link.send(frame(sid, instr, params))
            got = self._recv_frame(sid, n_params)
            if got is not None:
                return got
            self.errors += 1
        return None

    # -- 语义 --
    def ping(self, sid: int) -> bool:
        return self.transact(sid, PING, b"", 0) is not None

    def read(self, sid: int, addr: int, n: int) -> bytes | None:
        # 应答的 len = 数据字节数 + 2（err 与 chk），所以这里传 n 而不是 n+1
        got = self.transact(sid, READ, bytes((addr, n)), n)
        return got[1:] if got else None

    def write(self, sid: int, addr: int, data: bytes) -> bool:
        return self.transact(sid, WRITE, bytes((addr,)) + bytes(data), 0) is not None

    def sync_write(self, addr: int, width: int, items: list[tuple[int, bytes]]) -> None:
        """广播写没有应答，发出去就算成功（真丢包靠下一帧目标位置兜住）。"""
        self.link.send(sync_write_frame(addr, width, items))

    def read_servo(self, sid: int, full: bool = False) -> dict | None:
        block = self.read(sid, PRESENT_ADDR, PRESENT_N)
        if not block:
            return None
        torque = self.read(sid, REG["torque"][0], 1)
        rec = {
            "id": sid,
            "pos": _le16(block, 0),
            "speed": decode_speed(_le16(block, 2)),
            "load": round(decode_load(_le16(block, 4)), 1),
            "volt": round(block[6] / 10.0, 1),
            "temp": block[7],
            "torque": bool(torque[0]) if torque else False,
        }
        if full:
            # 页面上的单颗详情要看到 0 位与方向，轮询时就不读这两项，省总线
            off = self.read(sid, REG["offset"][0], 2)
            phase = self.read(sid, REG["phase"][0], 1)
            if off:
                raw = _le16(off, 0)
                rec["offset"] = raw - 0x10000 if raw > 0x7FFF else raw
            if phase:
                rec["phase"] = phase[0]
                rec["direction"] = -1 if phase[0] & 0x01 else 1
        return rec


# ---------------------------------------------------------------- 异步外壳
def _env_flag(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


class RadxaServo:
    """给 `hal/radxa.py` 用的舵机部分。方法名与 `hal/base.py` 的 `servo_*` 对齐。"""

    backend = "radxa-servo"

    def __init__(self, ctx) -> None:
        self.ctx = ctx
        self.log = getattr(ctx, "log", log) or log
        cfg = ctx.config
        self.port = getattr(cfg, "servo_port", "/dev/ttyS2")
        self.baud = int(getattr(cfg, "servo_baud", 1_000_000))
        self.sim = _env_flag("ZELUX_SERVO_SIM")
        self.hold_path = os.environ.get("ZELUX_MOTIOND_HOLD", "/run/zelux/motiond.hold")

        self._lock = asyncio.Lock()
        self._proto: ServoProto | None = None
        self._taken = False
        self._took_ms = 0
        self._missing: list[int] = []
        self._speed_limit = int(os.environ.get("ZELUX_SERVO_SPEED", "900"))
        self._poll: asyncio.Task | None = None
        self._cache: dict = {"online": 0, "items": [], "bus": "shared", "holding": 0,
                             "summary": {"online": 0, "holding": 0, "hot": 0}}

    # ---------------- 生命周期 ----------------
    async def start(self) -> None:
        # 不主动开串口：motiond 可能正占着总线，只有用户点"接管"才抢。
        if self.sim:
            self.log.info("舵机总线用内存假总线（ZELUX_SERVO_SIM=1）")

    async def stop(self) -> None:
        await self._stop_poll()
        if self._proto:
            # 关串口但不卸力：卸力鸭子会当场趴下，交还主程序才是正确的收尾。
            try:
                await asyncio.to_thread(self._proto.link.close)
            except Exception:
                pass
            self._proto = None
        self._taken = False
        self._clear_hold()

    def info(self) -> dict:
        return {"backend": self.backend, "port": self.port, "baud": self.baud,
                "sim": self.sim, "taken": self._taken, "speed_limit": self._speed_limit}

    # ---------------- 接管 / 交还 ----------------
    async def take(self) -> dict:
        async with self._lock:
            if self._taken:
                return {"ok": True, "online": self._cache.get("online", 0),
                        "missing": self._missing, "took_ms": self._took_ms, "again": True}
            t0 = time.monotonic()
            let_go = await asyncio.to_thread(self._signal_motiond)
            if let_go:
                # 给实时层 300 ms 收尾（它正跑 200 Hz 环，来不及放手会两边抢串口）
                await asyncio.sleep(0.3)
            res = await asyncio.to_thread(self._open_and_ping)
            self._took_ms = int((time.monotonic() - t0) * 1000)
            self._taken = True
            self._missing = res["missing"]
            self._start_poll()
            self.log.info("已接管舵机总线：%d/%d 在线，用时 %d ms",
                          res["online"], len(JOINT_IDS), self._took_ms)
            return {"ok": True, "online": res["online"], "missing": res["missing"],
                    "took_ms": self._took_ms}

    async def release(self) -> dict:
        async with self._lock:
            await self._stop_poll()
            if self._proto:
                try:
                    await asyncio.to_thread(self._proto.link.close)
                except Exception:
                    pass
                self._proto = None
            self._taken = False
            await asyncio.to_thread(self._clear_hold)
            self._cache = {**self._cache, "bus": "shared"}
            self.log.info("已交还舵机总线（舵机仍保持当前位置，不会卸力）")
            return {"ok": True, "note": "舵机仍保持住，主程序可以接手"}

    def _signal_motiond(self) -> bool:
        """写 hold 文件让 motiond 让路；文件系统只读或目录没建都不算致命。"""
        try:
            os.makedirs(os.path.dirname(self.hold_path), exist_ok=True)
            with open(self.hold_path, "w", encoding="utf-8") as fh:
                fh.write(f"{time.time():.3f}\n")
            return True
        except OSError as exc:
            self.log.warning("没写成 %s（%s），继续按'没有实时层'处理", self.hold_path, exc)
            return False

    def _clear_hold(self) -> None:
        try:
            os.unlink(self.hold_path)
        except OSError:
            pass

    def _make_link(self):
        return FakeLink() if self.sim else SerialLink(self.port, self.baud)

    def _open_and_ping(self) -> dict:
        try:
            link = self._make_link()
        except Exception as exc:
            raise hardware(str(getattr(exc, "message", exc))) from None
        proto = ServoProto(link, retries=2, timeout=0.02)
        online, missing = 0, []
        for sid in JOINT_IDS:
            if proto.ping(sid):
                online += 1
            else:
                missing.append(sid)
        if online == 0:
            link.close()
            raise hardware("总线上一颗舵机都没应答，检查电源和 TTL 线")
        self._proto = proto
        if missing:
            # 缺一两颗还能干活（多半是 ID 没刷或线序问题），但要让人马上看见
            self.log.warning("这些 ID 没应答：%s（共 %d 颗）", missing, len(missing))
        self._read_all_sync()
        return {"online": online, "missing": missing}

    # ---------------- 后台刷新 ----------------
    def _start_poll(self) -> None:
        if self._poll is None or self._poll.done():
            self._poll = asyncio.create_task(self._poll_loop())

    async def _stop_poll(self) -> None:
        task, self._poll = self._poll, None
        if not task:
            return
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass            # 就是我们自己 cancel 的
        except Exception:
            pass

    async def _poll_loop(self) -> None:
        # 2 Hz 够页面看，也不至于把总线占满（手动写寄存器时锁会让路）
        while self._taken:
            try:
                await self.read_all()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.log.warning("轮询舵机失败：%s", exc)
            await asyncio.sleep(0.5)

    # ---------------- 单颗 ----------------
    def _ensure(self) -> ServoProto:
        if not self._proto:
            raise hardware("还没接管舵机总线：先点「开始调舵机」")
        return self._proto

    async def servo_read(self, sid: int) -> dict:
        async with self._lock:
            proto = self._ensure()
            rec = await asyncio.to_thread(proto.read_servo, int(sid), True)
            if not rec:
                raise hardware(f"{sid} 号舵机没应答，检查这根线或它的 ID")
            return rec

    async def servo_write(self, sid: int, reg: int, value: int, eeprom: bool = False) -> None:
        async with self._lock:
            proto = self._ensure()
            width = ADDR_WIDTH.get(int(reg), 1)
            ok = await asyncio.to_thread(proto.write, int(sid), int(reg),
                                         encode_reg(int(value), width))
            if not ok:
                raise hardware(f"{sid} 号舵机没确认写入（寄存器 {reg}），可能掉线或限位拒绝")
            if eeprom:
                self.log.warning("EEPROM 写入（%s 号 寄存器 %s=%s）：掉电保持，改前想清楚",
                                 sid, reg, value)

    async def servo_torque(self, on: bool, ids: list[int] | None = None) -> None:
        async with self._lock:
            proto = self._ensure()
            targets = [int(i) for i in (ids or JOINT_IDS)]
            items = [(i, bytes((1 if on else 0,))) for i in targets if i in JOINT_IDS]
            if not items:
                return
            await asyncio.to_thread(proto.sync_write, REG["torque"][0], 1, items)

    # ---------------- 批量 ----------------
    async def write_goal_batch(self, goals: dict[int, int], speed: int | None = None) -> None:
        """一帧 SYNC_WRITE 写满所有目标：位置 2 字节 + 速度 2 字节，从 41 开始 4 字节。"""
        spd = self._speed_limit if speed is None else int(speed)
        items = []
        for sid, pos in goals.items():
            sid = int(sid)
            if sid not in JOINT_IDS:
                continue
            p = max(0, min(FULL_TURN - 1, int(pos)))
            items.append((sid, struct.pack("<H", p) + encode_speed(spd)))
        if not items:
            return
        async with self._lock:
            proto = self._ensure()
            await asyncio.to_thread(proto.sync_write, REG["goal_pos"][0], 4, items)

    async def servo_sync_goal(self, goals: dict[int, int]) -> None:
        await self.write_goal_batch(goals)

    # ---------------- 批量读 ----------------
    def _read_all_sync(self) -> dict:
        proto = self._proto
        if not proto:
            return self._cache
        items: list[dict] = []
        for sid in JOINT_IDS:
            rec = proto.read_servo(sid)
            if rec:
                items.append(rec)
        # 只有接管成功才会走到这里，所以 bus 一定是 taken（take 里那一刻 _taken 还没置位）
        self._cache = self._compose(items, taken=True)
        return self._cache

    def _compose(self, items: list[dict], taken: bool = True) -> dict:
        temps = [i["temp"] for i in items]
        return {
            "online": len(items),
            "ids": [i["id"] for i in items],
            "items": items,
            "bus": "taken" if taken else "shared",
            "port": self.port,
            "holding": sum(1 for i in items if i["torque"]),
            "missing": list(self._missing),
            "ts": time.time(),
            "summary": {"online": len(items), "holding": sum(1 for i in items if i["torque"]),
                        "hot": sum(1 for t in temps if t >= 60),
                        "max_temp": max(temps) if temps else None},
        }

    async def read_all(self) -> dict:
        async with self._lock:
            self._ensure()
            return await asyncio.to_thread(self._read_all_sync)

    def snapshot_servos(self) -> dict:
        return self._cache

    # ---------------- 标定辅助 ----------------
    async def set_offset(self, sid: int, ticks: int) -> None:
        """0 位偏移：写 EEPROM 里的 offset，负值按 16 位补码。"""
        await self.servo_write(int(sid), REG["offset"][0], int(ticks), eeprom=True)

    async def set_direction(self, sid: int, sign: int) -> None:
        """方向：只动相位寄存器 bit0，别把速度单位位（bit2/3）冲掉。"""
        async with self._lock:
            proto = self._ensure()
            raw = await asyncio.to_thread(proto.read, int(sid), REG["phase"][0], 1)
            if not raw:
                raise hardware(f"{sid} 号舵机读不到相位寄存器")
            phase = raw[0] | 0x01 if int(sign) < 0 else raw[0] & ~0x01
            ok = await asyncio.to_thread(proto.write, int(sid), REG["phase"][0], bytes((phase,)))
            if not ok:
                raise hardware(f"{sid} 号舵机方向没写进去（可能 0 位限位挡住）")

    async def save_offsets(self, items: Iterable) -> int:
        """批量落 0 位：一帧 SYNC_WRITE 写完，比逐颗快一个数量级。"""
        rows = []
        for it in items:
            if isinstance(it, dict):
                sid, ticks = int(it.get("id")), int(it.get("offset", 0))
            else:
                sid, ticks = int(it[0]), int(it[1])
            if sid in JOINT_IDS:
                rows.append((sid, encode_reg(ticks, 2)))
        if not rows:
            return 0
        async with self._lock:
            proto = self._ensure()
            await asyncio.to_thread(proto.sync_write, REG["offset"][0], 2, rows)
        return len(rows)
