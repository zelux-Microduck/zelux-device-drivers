# ZELUX Device Drivers

舵机、ToF、摄像头、风扇和 IMU 的可插拔驱动实现。本仓库独立于 Device Agent 发布。

当前首批迁移内容：

- Feetech STS/SCS 舵机总线；
- VL53L5CX ToF；
- QMI8658、MPU6050、ICM42688 IMU；
- Radxa PWM/Tach 风扇。

```powershell
python -m pip install -e .
python -m compileall -q src
```

迁移期 Agent 仍保留一份内置驱动作为兼容回退。下一步由 Agent 优先加载本包，验证完成后
删除重复实现。驱动包和固件包的清单 Schema 由 `zelux-protocol` 维护。
