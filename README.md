# ZELUX Device Drivers

舵机、ToF、摄像头、风扇和 IMU 的可插拔驱动实现。本仓库独立于 Device Agent 发布。

当前首批迁移内容：

- Feetech STS/SCS 舵机总线；
- VL53L5CX ToF；
- QMI8658、MPU6050、ICM42688 IMU；
- Radxa PWM/Tach 风扇。
- V4L2 / libcamera 摄像头。

业务侧使用稳定型号 ID 创建驱动，不直接导入实现类：

```python
from zelux_drivers import create, descriptors

print([item.to_dict() for item in descriptors("imu")])
imu = create("zelux.imu.multi-i2c", ctx)
```

内置型号 ID：

- `zelux.servo.feetech-sts-scs`
- `zelux.tof.st-vl53l5cx`
- `zelux.camera.v4l2-libcamera`
- `zelux.fan.radxa-pwm` (风扇数量由设备实例配置决定)
- `zelux.imu.multi-i2c`

```powershell
python -m pip install -e .
python -m compileall -q src
```

迁移期 Agent 仍保留一份内置驱动作为兼容回退。下一步由 Agent 优先加载本包，验证完成后
删除重复实现。驱动包和固件包的清单 Schema 由 `zelux-protocol` 维护。
