"""V4L2/libcamera 摄像头适配器。"""
from __future__ import annotations

import asyncio
import shutil
import socket
import subprocess
from pathlib import Path


class RadxaCamera:
    def __init__(self, ctx, device: str | None = None, stream_port: int = 8443) -> None:
        self.ctx = ctx
        self.device = device
        self.stream_port = stream_port

    async def start(self) -> None:
        return None

    async def stop(self) -> None:
        return None

    def info(self) -> dict:
        dev = self.device or self._video_device()
        return {"driver": "v4l2-libcamera", "device": dev, "available": bool(dev)}

    async def snapshot(self, path: str) -> bool:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        dev = self.device or self._video_device()
        commands: list[tuple[list[str], Path]] = []
        if dev and shutil.which("ffmpeg"):
            commands.append((["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "v4l2",
                              "-i", dev, "-frames:v", "1", "-y", str(target)], target))
        if shutil.which("libcamera-still"):
            jpg = target.with_suffix(".jpg")
            commands.append((["libcamera-still", "-n", "-t", "1", "--width", "1280",
                              "--height", "720", "-o", str(jpg)], jpg))
        for command, output in commands:
            try:
                proc = await asyncio.create_subprocess_exec(
                    *command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                await asyncio.wait_for(proc.wait(), timeout=8)
                if proc.returncode == 0 and output.exists():
                    return True
            except Exception:
                continue
        return False

    def stream_url(self) -> str | None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.15)
        try:
            return "/camera/stream" if sock.connect_ex(("127.0.0.1", self.stream_port)) == 0 else None
        finally:
            sock.close()

    @staticmethod
    def _video_device() -> str | None:
        return next((str(path) for path in sorted(Path("/dev").glob("video*"))), None)
