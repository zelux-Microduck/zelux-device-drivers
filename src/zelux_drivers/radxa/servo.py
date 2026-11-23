"""Servo adapter: robotd exclusively owns the physical bus.

Production operations use the connection-bound robot.servoSession API v39.
The local protocol helpers and FakeLink serve simulation and protocol tests only;
there is no direct-UART fallback if robotd or its IPC is unavailable.
"""
from __future__ import annotations

import asyncio
import logging
import math
import os
import json
import socket
import select
import struct
import time
from typing import Iterable

from ..errors import hardware
from ..base import JOINT_IDS

log = logging.getLogger("zeluxd.servo")

# ---------------------------------------------------------------- 协议常量
PING, READ, WRITE, REG_WRITE, ACTION = 1, 2, 3, 4, 5
RESET, REBOOT, CALIBRATE, SYNC_READ, SYNC_WRITE = 6, 8, 0x0B, 0x82, 0x83
BROADCAST = 0xFE
CENTER = 2048
FULL_TURN = 4096
TICKS_PER_RAD = FULL_TURN / (2 * math.pi)      # 651.8986 步/弧度
MAX_SPEED = 3400                               # HD-1910 速度上限，步/秒

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
    "max_torque": (16, 2),
    "min_start_torque": (24, 2),
    "deadband_cw": (26, 1),
    "deadband_ccw": (27, 1),
    "phase": (18, 1),        # bit0=方向反转，bit2/3=速度单位（日志里 18=52 → 1 步/秒）
    "protect": (19, 1),      # 卸载条件（堵转保护开关）
    "p": (21, 1),
    "d": (22, 1),
    "i": (23, 1),
    "offset": (31, 2),
    "mode": (33, 1),
    "torque": (40, 1),
    "acceleration": (41, 1),
    "goal_pos": (42, 2),
    "goal_time": (44, 2),
    "goal_speed": (46, 2),
    "torque_limit": (48, 2),
    "p_run": (50, 1),
    "d_run": (51, 1),
    "lock": (55, 1),
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
    """飞特 STS/SCS SYNC_WRITE：地址和数据长度各 1 字节。"""
    params = bytes((addr & 0xFF, width & 0xFF))
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


# HD-1910 profile: field readback on IDs 10/24 confirms bit15 sign.
# Do not reuse this codec for other STS models without a verified profile.
OFFSET_SIGN = 0x8000
OFFSET_MAGNITUDE = 0x07FF


def encode_signed_magnitude(value: int) -> bytes:
    """HD-1910 register 31: bit15 sign, magnitude limited to 2047."""
    value = int(value)
    if abs(value) > OFFSET_MAGNITUDE:
        raise ValueError("HD-1910 offset must be within -2047..2047")
    return struct.pack("<H", abs(value) | (OFFSET_SIGN if value < 0 else 0))


def decode_signed_magnitude(raw: int) -> int:
    if raw < 0 or raw > 0xFFFF or raw & ~(OFFSET_SIGN | OFFSET_MAGNITUDE):
        raise ValueError(f"invalid HD-1910 offset encoding; register 31 raw={raw:#06x} ({raw}); "
                         "expected bit15 sign and magnitude <=2047; no offset write attempted")
    return -(raw & OFFSET_MAGNITUDE) if raw & OFFSET_SIGN else raw

def center_offset(offset: int, position: int, target: int = CENTER) -> int:
    """present = (encoder - offset) mod 4096; preserve the physical pose."""
    if abs(offset) > 2047 or not 0 <= position <= 4095 or target != CENTER:
        raise ValueError("invalid STS center preview")
    result = (offset + position - target + 2048) % 4096 - 2048
    if result == -2048:
        raise ValueError("center offset is at the unrepresentable half-turn boundary")
    return result


def encode_reg(value: int, width: int, *, signed_magnitude: bool = False) -> bytes:
    if width == 2:
        return encode_signed_magnitude(value) if signed_magnitude else struct.pack("<H", value & 0xFFFF)
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
                21: 16, 22: 32, 23: 8, 31: 0, 33: 0, 40: 0, 41: 0,
                42: CENTER, 44: 0, 46: 0, 48: 1000, 50: 5, 51: 0, 55: 1,
                56: CENTER, 58: 0, 60: 0, 62: 78, 63: 32}

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
        elif instr == SYNC_READ and len(p) >= 3:
            addr, width, targets = p[0], p[1], p[2:]
            for target in targets:
                if target in self.regs:
                    mem = self._mem(target)
                    self._reply(target, bytes(mem.get(addr + k, 0) for k in range(width)))
        elif instr == SYNC_WRITE and len(p) >= 2:
            addr, width = p[0], p[1]
            body = p[2:]
            for k in range(0, len(body) - width, width + 1):
                target = body[k]
                if target in self.regs:
                    self._poke(target, addr, body[k + 1:k + 1 + width])
        elif instr == CALIBRATE and sid in self.regs:
            target = _le16(p, 0) if len(p) >= 2 else CENTER
            current = self.regs[sid].get(REG["pos"][0], CENTER)
            old_offset = decode_signed_magnitude(self.regs[sid].get(REG["offset"][0], 0))
            new_offset = center_offset(old_offset, current, target)
            self.regs[sid][REG["offset"][0]] = struct.unpack("<H", encode_signed_magnitude(new_offset))[0]
            self.regs[sid][REG["pos"][0]] = target
            self._reply(sid, b"")
        elif instr == REBOOT and sid in self.regs:
            self._reply(sid, b"")
        elif instr == RESET and sid in self.regs:
            self.regs[sid] = self._defaults(sid)
        # ACTION 无应答，符合协议

    # -- 测试辅助 --
    def settle(self) -> None:
        """模拟"舵机走到目标位置"：让现值跟上目标，测姿势时可以看最终落点。"""
        for sid, regs in self.regs.items():
            regs[56] = regs.get(42, CENTER)


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
        """广播写没有应答，发出去就算成功（真丢包靠回读校验）。"""
        self.link.send(sync_write_frame(addr, width, items))

    def _scan_any(self, allowed: set[int], n_params: int) -> tuple[int, bytes] | None:
        """从缓冲区取任意一颗允许 ID 的状态包，过滤 TTL 回显。"""
        buf, i, need = self._buf, 0, n_params + 2
        while i + 4 <= len(buf):
            if buf[i] != 0xFF or buf[i + 1] != 0xFF:
                i += 1
                continue
            sid = buf[i + 2]
            if sid not in allowed or buf[i + 3] != need:
                i += 1
                continue
            total = 4 + need
            if len(buf) < i + total:
                return None
            body = bytes(buf[i + 2:i + total - 1])
            if checksum(body) != buf[i + total - 1] or body[2] != 0:
                i += 1
                continue
            del buf[:i + total]
            return sid, bytes(body[3:])
        del buf[:i]
        return None

    def sync_read(self, ids: list[int], addr: int, n: int) -> dict[int, bytes | None]:
        """飞特 SYNC_READ：一次请求，收齐多颗状态包；缺包记 None。"""
        ids = [int(sid) for sid in ids]
        out: dict[int, bytes | None] = dict.fromkeys(ids)
        if not ids:
            return out
        self._buf.clear()
        self.link.send(frame(BROADCAST, SYNC_READ, bytes((addr & 0xFF, n & 0xFF, *ids))))
        waiting = set(ids)
        # 50 Hz 刷新必须在一个 20 ms 周期内结束；缺一颗不能把整帧拖到数百毫秒。
        deadline = time.monotonic() + max(0.01, self.timeout * 0.9)
        while waiting and time.monotonic() < deadline:
            got = self._scan_any(waiting, n)
            if got is not None:
                sid, data = got
                out[sid] = data
                waiting.discard(sid)
                continue
            chunk = self.link.recv(min(0.004, max(0.0, deadline - time.monotonic())))
            if chunk:
                self._buf += chunk
            else:
                time.sleep(0.0005)
        return out

    def read_states_fast(self, ids: list[int]) -> dict[int, dict | None]:
        raw = self.sync_read(ids, PRESENT_ADDR, PRESENT_N)
        out: dict[int, dict | None] = {}
        for sid, block in raw.items():
            if not block:
                out[sid] = None
                continue
            out[sid] = {
                "id": sid,
                "pos": _le16(block, 0),
                "speed": decode_speed(_le16(block, 2)),
                "load": round(decode_load(_le16(block, 4)), 1),
                "volt": round(block[6] / 10.0, 1),
                "temp": block[7],
            }
        return out

    def calibrate_to(self, sid: int, target: int = CENTER) -> bool:
        """把舵机当前物理位置校准为 target；HD-1910 使用 0x0B 指令。"""
        return self.transact(sid, CALIBRATE, struct.pack("<H", target & 0xFFFF), 0) is not None

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
            # 页面单颗详情：EEPROM 偏移/相位，以及运行中的限力/P/D。
            off = self.read(sid, REG["offset"][0], 2)
            phase = self.read(sid, REG["phase"][0], 1)
            gains = self.read(sid, REG["torque_limit"][0], 4)
            if off:
                raw = _le16(off, 0)
                rec["offset"] = decode_signed_magnitude(raw)
            if phase:
                rec["phase"] = phase[0]
                rec["phase_direction"] = -1 if phase[0] & 0x01 else 1
                rec["direction"] = -1  # 软件模型方向；不是相位寄存器 bit0
            if gains and len(gains) == 4:
                rec["torque_limit"] = _le16(gains, 0)
                rec["p_run"], rec["d_run"] = gains[2], gains[3]
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

        self._lock = asyncio.Lock()
        self._proto: ServoProto | None = None
        self._ipc_socket: socket.socket | None = None
        self._ipc_reader = None
        self._session_id: str | None = None
        self._rpc_id = 0
        self._taken = False
        self._took_ms = 0
        self._missing: list[int] = []
        self._speed_limit = int(os.environ.get("ZELUX_SERVO_SPEED", "900"))
        self._poll: asyncio.Task | None = None
        self._cache: dict = {"online": 0, "items": [], "bus": "shared", "holding": 0,
                             "summary": {"online": 0, "holding": 0, "hot": 0}}

    def _robotd_call_sync(self, params: dict) -> dict:
        """Send one request on the persistent robotd connection that owns our lease."""
        if self._ipc_socket is None:
            path = os.environ.get("ZELUX_ROBOTD_SOCKET", "/run/robotd.sock")
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect(path)
                self._ipc_socket = sock
                self._ipc_reader = sock.makefile("r", encoding="utf-8", newline="\n")
            except OSError as exc:
                raise hardware(f"robotd servo session unavailable: {exc}") from None
        self._rpc_id += 1
        request = {"jsonrpc": "2.0", "id": self._rpc_id,
                   "method": "robot.servoSession", "params": params}
        try:
            self._ipc_socket.sendall((json.dumps(request, separators=(",", ":")) + "\n").encode())
            line = self._ipc_reader.readline()
            if not line:
                raise OSError("robotd closed the servo session connection")
            response = json.loads(line)
        except (OSError, ValueError, TypeError) as exc:
            self._close_robotd_socket()
            if self._taken:
                self._taken = False
                self._cache = {**self._cache, "bus": "shared"}
            raise hardware(f"robotd servo session IPC failed; lease will expire after disconnect: {exc}") from None
        if response.get("error"):
            error = response["error"]
            message = str(error.get("message", "robotd refused servo operation"))
            code = error.get("code")
            if (code == -32601 or "unknown method" in message.lower()
                    or "method not found" in message.lower()
                    or ("api version" in message.lower() and "unsupported" in message.lower())):
                self._close_robotd_socket()
                raise hardware("robotd update required: installed robotd does not support servo maintenance IPC")
            raise hardware(message)
        result = response.get("result")
        if not isinstance(result, dict):
            self._close_robotd_socket()
            raise hardware("robotd returned an invalid servo session response")
        return result

    def _close_robotd_socket(self) -> None:
        reader, sock = self._ipc_reader, self._ipc_socket
        self._ipc_reader, self._ipc_socket = None, None
        self._session_id = None
        if reader:
            try:
                reader.close()
            except OSError:
                pass
        if sock:
            try:
                sock.close()
            except OSError:
                pass

    def _servo_command_sync(self, op: str, **fields) -> dict:
        if not self._session_id:
            raise hardware("servo maintenance session is not active")
        return self._robotd_call_sync({"action": "command", "session_id": self._session_id,
                                       "command": {"op": op, **fields}})

    # ---------------- 生命周期 ----------------
    async def start(self) -> None:
        # The physical bus belongs to robotd. Only simulation creates an in-memory transport.
        if self.sim:
            self.log.info("舵机总线用内存假总线（ZELUX_SERVO_SIM=1）")

    async def stop(self) -> None:
        if self._taken:
            try:
                await self.release()
            except Exception as exc:
                self.log.warning("停止时无法正常结束舵机维护会话：%s", exc)
                self._close_robotd_socket()
                self._taken = False
        else:
            await self._stop_poll()
            if self._proto:
                try:
                    await asyncio.to_thread(self._proto.link.close)
                except Exception:
                    pass
                self._proto = None

    def info(self) -> dict:
        return {"backend": self.backend, "port": self.port, "baud": self.baud,
                "sim": self.sim, "taken": self._taken, "speed_limit": self._speed_limit}

    # ---------------- 接管 / 交还 ----------------
    async def take(self, supported: bool = False) -> dict:
        async with self._lock:
            if self._taken:
                return {"ok": True, "online": self._cache.get("online", 0),
                        "missing": self._missing, "took_ms": self._took_ms, "again": True}
            t0 = time.monotonic()
            if self.sim:
                res = await asyncio.to_thread(self._open_and_ping)
                self._taken = True
                self._missing = res["missing"]
                self._start_poll()
                return {"ok": True, "online": res["online"], "missing": self._missing,
                        "took_ms": int((time.monotonic() - t0) * 1000)}
            result = await asyncio.to_thread(self._robotd_call_sync, {"action": "begin", "supported": supported})
            self._session_id = result.get("session_id")
            if not self._session_id:
                self._close_robotd_socket()
                raise hardware("robotd did not return a servo session token")
            protocol = result.get("bus_protocol")
            if protocol != "feetech-sts":
                try:
                    await asyncio.to_thread(
                        self._robotd_call_sync,
                        {"action": "end", "session_id": self._session_id},
                    )
                finally:
                    self._close_robotd_socket()
                if protocol is None:
                    raise hardware("robotd update required: servo session does not report bus protocol")
                raise hardware(f"Studio servo adapter does not support {protocol}; no register data was interpreted")
            self._taken = True
            self._took_ms = int((time.monotonic() - t0) * 1000)
            lease_id = self._session_id
            try:
                snapshot = await asyncio.to_thread(self._read_all_sync)
            except Exception:
                # A failed initial bus read must not strand local "taken" state or the
                # robotd lease. Explicit release is valid only on the original connection:
                # after a transport failure the connection-bound lease can only expire by TTL.
                try:
                    if self._session_id == lease_id and lease_id is not None:
                        await asyncio.to_thread(
                            self._robotd_call_sync,
                            {"action": "end", "session_id": lease_id},
                        )
                except Exception:
                    pass
                finally:
                    self._close_robotd_socket()
                    self._taken = False
                    self._cache = {**self._cache, "bus": "shared"}
                raise
            self._missing = [sid for sid in JOINT_IDS if sid not in snapshot.get("ids", [])]
            self._start_poll()
            self.log.info("已通过 robotd 建立舵机维护会话：%d/%d 在线，用时 %d ms",
                          snapshot.get("online", 0), len(JOINT_IDS), self._took_ms)
            return {"ok": True, "online": snapshot.get("online", 0),
                    "missing": self._missing, "took_ms": self._took_ms}

    async def release(self) -> dict:
        async with self._lock:
            await self._stop_poll()
            if self.sim:
                if self._proto:
                    try:
                        await asyncio.to_thread(self._proto.link.close)
                    except Exception:
                        pass
                    self._proto = None
            elif self._session_id:
                try:
                    await asyncio.to_thread(self._robotd_call_sync,
                                            {"action": "end", "session_id": self._session_id})
                finally:
                    self._close_robotd_socket()
            self._taken = False
            self._cache = {**self._cache, "bus": "shared"}
            self.log.info("已交还舵机维护会话；舵机不卸力")
            return {"ok": True, "note": "舵机仍保持住，robotd 恢复控制"}

    def _make_link(self):
        if not self.sim:
            raise hardware("physical servo UART is owned exclusively by robotd")
        return FakeLink()

    def _open_and_ping(self) -> dict:
        if not self.sim:
            raise hardware("direct physical servo access is disabled; use robotd servo session")
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
            if self.sim:
                proto = self._ensure()
                rec = await asyncio.to_thread(proto.read_servo, int(sid), True)
                if not rec:
                    raise hardware(f"{sid} 号舵机没应答，检查这根线或它的 ID")
                return rec
            self._need_robotd_session()
            present = await asyncio.to_thread(self._servo_command_sync, "read_register",
                                              servo_id=int(sid), address=56, length=8)
            raw = bytes(present.get("data") or [])
            if len(raw) != 8:
                raise hardware(f"{sid} 号舵机反馈数据长度错误")
            torque = await asyncio.to_thread(self._servo_command_sync, "read_register",
                                             servo_id=int(sid), address=40, length=1)
            offset = await asyncio.to_thread(self._servo_command_sync, "read_register",
                                             servo_id=int(sid), address=31, length=2)
            phase = await asyncio.to_thread(self._servo_command_sync, "read_register",
                                            servo_id=int(sid), address=18, length=1)
            off = bytes(offset.get("data") or [])
            ph = bytes(phase.get("data") or [])
            rec = {"id": int(sid), "pos": _le16(raw, 0),
                   "speed": decode_speed(_le16(raw, 2)),
                   "load": round(decode_load(_le16(raw, 4)), 1),
                   "volt": round(raw[6] / 10.0, 1), "temp": raw[7],
                   "torque": bool((torque.get("data") or [0])[0])}
            if len(off) == 2:
                signed = _le16(off, 0)
                try:
                    rec["offset"] = decode_signed_magnitude(signed)
                except ValueError as exc:
                    raise hardware(f"{sid} 号舵机偏移编码异常：{exc}；保留备份，禁止猜测转换") from exc
            if ph:
                rec["phase"] = ph[0]
                rec["direction"] = -1 if ph[0] & 1 else 1
            return rec

    def _need_robotd_session(self) -> None:
        if not self._taken or not self._session_id:
            raise hardware("还没接管舵机总线：先点「开始调舵机」")

    async def servo_write(self, sid: int, reg: int, value: int, eeprom: bool = False,
                          confirmed: bool = False) -> None:
        if int(sid) not in JOINT_IDS or (int(reg) == 31 and abs(int(value)) > 2047):
            raise hardware("invalid servo ID or HD-1910 offset")
        async with self._lock:
            if self.sim:
                proto = self._ensure()
                width = ADDR_WIDTH.get(int(reg), 1)
                ok = await asyncio.to_thread(proto.write, int(sid), int(reg),
                                             (encode_signed_magnitude(int(value)) if int(reg) == 31 else encode_reg(int(value), width)))
                if not ok:
                    raise hardware(f"{sid} 号舵机没确认写入（寄存器 {reg}）")
            else:
                self._need_robotd_session()
                width = ADDR_WIDTH.get(int(reg), 1)
                await asyncio.to_thread(self._servo_command_sync, "write_register",
                                         servo_id=int(sid), address=int(reg),
                                         data=list(encode_signed_magnitude(int(value)) if int(reg) == 31 else encode_reg(int(value), width)), eeprom=bool(eeprom),
                                         confirmed=bool(confirmed))
            if eeprom:
                self.log.warning("EEPROM 写入（%s 号 寄存器 %s=%s）：掉电保持，改前想清楚",
                                 sid, reg, value)

    async def servo_torque(self, on: bool, ids: list[int] | None = None) -> None:
        async with self._lock:
            targets = [int(i) for i in (ids or JOINT_IDS)]
            if self.sim:
                proto = self._ensure()
                items = [(i, bytes((1 if on else 0,))) for i in targets if i in JOINT_IDS]
                if items:
                    await asyncio.to_thread(proto.sync_write, REG["torque"][0], 1, items)
            else:
                self._need_robotd_session()
                await asyncio.to_thread(self._servo_command_sync, "set_torque",
                                         on=bool(on), servo_ids=targets)

    async def write_goal_batch(self, goals: dict[int, int], speed: int | None = None) -> None:
        speed_value = self._speed_limit if speed is None else int(speed)
        rows = []
        for sid, pos in goals.items():
            sid = int(sid)
            if sid not in JOINT_IDS:
                continue
            rows.append((sid, max(0, min(FULL_TURN - 1, int(pos)))))
        if not rows:
            return
        async with self._lock:
            if self.sim:
                items = [(sid, struct.pack("<HH", pos, 0) + encode_speed(speed_value)) for sid, pos in rows]
                await asyncio.to_thread(self._ensure().sync_write, REG["goal_pos"][0], 6, items)
            else:
                self._need_robotd_session()
                encoded_speed = int.from_bytes(encode_speed(speed_value), "little")
                goals_payload = [{"servo_id": sid, "position": pos, "speed": encoded_speed}
                                 for sid, pos in rows]
                await asyncio.to_thread(self._servo_command_sync, "sync_goals", goals=goals_payload)

    async def servo_sync_goal(self, goals: dict[int, int]) -> None:
        await self.write_goal_batch(goals)

    def _read_all_sync(self) -> dict:
        if self.sim:
            proto = self._proto
            if not proto:
                return self._cache
            items = [rec for sid in JOINT_IDS if (rec := proto.read_servo(sid))]
        else:
            self._need_robotd_session()
            result = self._servo_command_sync("snapshot")
            try:
                payload = json.loads(bytes(result.get("data") or []).decode("utf-8"))
            except (ValueError, UnicodeDecodeError, TypeError) as exc:
                raise hardware(f"robotd returned an invalid servo snapshot: {exc}") from None
            items = []
            for row in payload.get("items", []):
                raw = bytes(row.get("raw") or [])
                if len(raw) < 8:
                    continue
                items.append({"id": int(row["id"]), "pos": _le16(raw, 0),
                              "speed": decode_speed(_le16(raw, 2)),
                              "load": round(decode_load(_le16(raw, 4)), 1),
                              "volt": round(raw[6] / 10.0, 1), "temp": raw[7],
                              "model_direction": row.get("model_direction"),
                              "torque": bool(row.get("torque", False))})
        self._cache = self._compose(items, taken=True)
        return self._cache

    def _compose(self, items: list[dict], taken: bool = True) -> dict:
        temps = [i["temp"] for i in items]
        return {"source": "sim" if self.sim else "robotd", "online": len(items), "ids": [i["id"] for i in items], "items": items,
                "bus": "taken" if taken else "shared", "port": self.port,
                "holding": sum(1 for i in items if i["torque"]), "missing": list(self._missing),
                "ts": time.time(), "summary": {"online": len(items),
                    "holding": sum(1 for i in items if i["torque"]),
                    "hot": sum(1 for t in temps if t >= 60),
                    "max_temp": max(temps) if temps else None}}

    async def read_all(self) -> dict:
        async with self._lock:
            if not self._taken:
                raise hardware("还没接管舵机总线：先点「开始调舵机」")
            return await asyncio.to_thread(self._read_all_sync)

    def snapshot_servos(self) -> dict:
        return self._cache

    # ---------------- 标定辅助 ----------------
    async def servo_goal(self, sid: int, pos: int) -> dict:
        if int(sid) not in JOINT_IDS or not 0 <= int(pos) < FULL_TURN:
            raise hardware("invalid servo goal")
        await self.write_goal_batch({int(sid): int(pos)}, speed=200)
        return {"id": int(sid), "pos": int(pos)}

    async def center_backup(self, ids: list[int]) -> dict:
        backup = {}
        for sid in ids:
            rec = await self.servo_read(int(sid))
            if rec.get("torque"):
                raise hardware(f"{sid} 号仍在加力：先固定支撑机体并显式卸力，再进行中位校准")
            backup[int(sid)] = {"pos": rec["pos"], "offset": rec["offset"], "torque": False}
        return backup

    async def _offset_compare_set(self, sid: int, expected: int, offset: int,
                                  position: int | None = None) -> dict:
        if abs(expected) > 2047 or abs(offset) > 2047:
            raise hardware("HD-1910 offset must be within -2047..2047")
        async with self._lock:
            if self.sim:
                proto = self._ensure()
                rec = await asyncio.to_thread(proto.read_servo, sid, True)
                if not rec or rec["torque"] or rec["offset"] != expected:
                    raise hardware("center backup is stale or torque is on")
                if position is not None:
                    if abs(rec["pos"] - position) > 2:
                        raise hardware("center preview is stale")
                    ok = await asyncio.to_thread(proto.calibrate_to, sid, CENTER)
                else:
                    ok = await asyncio.to_thread(proto.write, sid, 31, encode_signed_magnitude(offset))
                if not ok:
                    raise hardware("offset write failed")
                return {"offset_before": expected, "offset_after": offset}
            self._need_robotd_session()
            response = await asyncio.to_thread(self._servo_command_sync, "set_offset",
                servo_id=sid, expected_offset=expected, offset=offset,
                expected_position=position, confirmed=True)
            return json.loads(bytes(response.get("data") or []).decode("utf-8"))

    async def center_servos(self, ids: list[int], target: int = CENTER, backup: dict | None = None) -> dict:
        if target != CENTER:
            raise hardware("only HD-1910 center 2048 is supported")
        rows = {int(k): v for k, v in (backup or {}).items()}
        # Validate the entire batch before the first EEPROM write.
        planned = {int(sid): center_offset(int(rows[int(sid)]["offset"]), int(rows[int(sid)]["pos"]), target)
                   for sid in ids}
        results = {}
        try:
            for sid in ids:
                row = rows[int(sid)]
                new = planned[int(sid)]
                results[int(sid)] = await self._offset_compare_set(int(sid), int(row["offset"]), new, int(row["pos"]))
        except Exception as error:
            failures = []
            for sid, result in reversed(list(results.items())):
                try:
                    await self._offset_compare_set(sid, result["offset_after"], int(rows[sid]["offset"]))
                except Exception as rollback:
                    failures.append(f"{sid}: {rollback}")
            raise hardware(f"中位校准未完成：{error}；已完成项回滚：{failures or '成功'}；保留备份，请检查后恢复") from error
        return {"results": results}


    async def restore_center_backup(self, backup: dict) -> dict:
        restored = []
        planned = []
        if not isinstance(backup, dict) or not backup:
            raise hardware("中位备份为空或格式错误：保留文件，不执行恢复")
        targets = {}
        for key, row in backup.items():
            # Accept canonical JSON keys and in-process integer IDs only.
            # Never truncate floats, accept booleans or silently merge aliases.
            if type(key) is int:
                sid = key
            elif type(key) is str and key.isascii() and key.isdecimal():
                sid = int(key)
                if str(sid) != key:
                    raise hardware(f"中位备份 ID 格式错误：{key!r}；保留备份")
            else:
                raise hardware(f"中位备份 ID 格式错误：{key!r}；保留备份")
            target = row.get("offset") if isinstance(row, dict) else None
            if sid not in JOINT_IDS or type(target) is not int or abs(target) > 2047:
                raise hardware(f"{sid} 号备份 ID 或偏移无效（offset={target!r}）：保留备份，不执行恢复")
            if sid in targets:
                raise hardware(f"中位备份包含重复舵机 {sid}：保留备份，不执行恢复")
            targets[sid] = target
        for sid, target in targets.items():
            current = await self.servo_read(sid)
            if current["torque"]:
                raise hardware(f"{sid} 号仍在加力：恢复偏移前请固定支撑并卸力")
            planned.append((sid, int(current["offset"]), target))
        # Preflight every entry before writing any. robotd still checks the
        # expected offset, torque and movement again at each actual write.
        for sid, expected, target in planned:
            if expected != target:
                try:
                    await self._offset_compare_set(sid, expected, target)
                except Exception as exc:
                    raise hardware(f"恢复中位在 {sid} 号失败：{exc}；已恢复 {restored}；"
                                   "备份保留，未自动加力，请检查后重试恢复") from exc
            restored.append(sid)
        return {"restored": restored}

    async def set_offset(self, sid: int, ticks: int) -> None:
        """0 位偏移：写 EEPROM 里的 offset，负值按符号-幅值编码。"""
        await self.servo_write(int(sid), REG["offset"][0], int(ticks), eeprom=True, confirmed=True)

    async def set_direction(self, sid: int, sign: int) -> None:
        async with self._lock:
            if self.sim:
                proto = self._ensure()
                raw = await asyncio.to_thread(proto.read, int(sid), REG["phase"][0], 1)
                if not raw:
                    raise hardware(f"{sid} 号舵机读不到相位寄存器")
                phase = raw[0] | 1 if int(sign) < 0 else raw[0] & ~1
                ok = await asyncio.to_thread(proto.write, int(sid), REG["phase"][0], bytes((phase,)))
                if not ok:
                    raise hardware(f"{sid} 号舵机方向没写进去")
            else:
                self._need_robotd_session()
                result = await asyncio.to_thread(self._servo_command_sync, "read_register",
                                                 servo_id=int(sid), address=18, length=1)
                raw = result.get("data") or []
                if not raw:
                    raise hardware(f"{sid} 号舵机读不到相位寄存器")
                phase = (int(raw[0]) | 1) if int(sign) < 0 else (int(raw[0]) & ~1)
                await asyncio.to_thread(self._servo_command_sync, "write_register",
                                         servo_id=int(sid), address=18, data=[phase], eeprom=True,
                                         confirmed=True)

    async def save_offsets(self, items: Iterable) -> int:
        rows = []
        for item in items:
            if isinstance(item, dict):
                sid, ticks = int(item.get("id")), int(item.get("offset", 0))
            else:
                sid, ticks = int(item[0]), int(item[1])
            if sid in JOINT_IDS:
                rows.append((sid, ticks))
        if not rows:
            return 0
        async with self._lock:
            if self.sim:
                entries = [(sid, encode_signed_magnitude(ticks)) for sid, ticks in rows]
                await asyncio.to_thread(self._ensure().sync_write, REG["offset"][0], 2, entries)
            else:
                self._need_robotd_session()
                for sid, ticks in rows:
                    await asyncio.to_thread(self._servo_command_sync, "write_register",
                                             servo_id=sid, address=31,
                                             data=list(encode_reg(ticks, 2)), eeprom=True,
                                             confirmed=True)
        return len(rows)
