class DriverHardwareError(RuntimeError):
    """驱动访问硬件失败。Agent 在边界处把它转换为公共错误码。"""


def hardware(message: str) -> DriverHardwareError:
    return DriverHardwareError(message)
