"""ZELUX 可插拔设备驱动及内置型号目录。"""

__version__ = "1.2.0"

from .catalog import create, descriptor, descriptors, register
from .contracts import (CameraDriver, DeviceDriver, DeviceKind, DriverDescriptor,
                        FanDriver, ImuDriver, ServoDriver, TofDriver)
from .radxa import RadxaCamera, RadxaFan, RadxaIMU, RadxaServo, RadxaTof

_VERSION = __version__
register(DriverDescriptor("zelux.servo.feetech-sts-scs", DeviceKind.SERVO, _VERSION,
                          "Feetech", ("STS3215", "STS", "SCS"),
                          ("read", "write", "sync-goal", "torque", "calibration")), RadxaServo)
register(DriverDescriptor("zelux.tof.st-vl53l5cx", DeviceKind.TOF, _VERSION,
                          "STMicroelectronics", ("VL53L5CX",),
                          ("8x8", "distance-mm", "15hz")), RadxaTof)
register(DriverDescriptor("zelux.camera.v4l2-libcamera", DeviceKind.CAMERA, _VERSION,
                          "Linux", ("UVC", "V4L2", "libcamera"),
                          ("snapshot", "stream")), RadxaCamera)
register(DriverDescriptor("zelux.fan.radxa-pwm", DeviceKind.FAN, _VERSION,
                          "Radxa", ("Zero 3W PWM fan instances",),
                          ("pwm", "tachometer", "temperature-curve")), RadxaFan)
register(DriverDescriptor("zelux.imu.multi-i2c", DeviceKind.IMU, _VERSION,
                          "ZELUX", ("QMI8658", "MPU6050", "ICM42688"),
                          ("acceleration", "gyroscope", "orientation", "auto-detect")), RadxaIMU)

__all__ = ["CameraDriver", "DeviceDriver", "DeviceKind", "DriverDescriptor", "FanDriver",
           "ImuDriver", "ServoDriver", "TofDriver", "create", "descriptor", "descriptors",
           "RadxaCamera", "RadxaFan", "RadxaIMU", "RadxaServo", "RadxaTof"]
