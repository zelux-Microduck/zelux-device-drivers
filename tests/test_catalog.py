from zelux_drivers import (CameraDriver, DeviceKind, FanDriver, ImuDriver, ServoDriver,
                           TofDriver, create, descriptor, descriptors)


class Context:
    class Config:
        from pathlib import Path

        servo_port = "none"
        servo_baud = 1_000_000
        fan_helper_socket = "none"
        fan_default_pct = 60
        fan_curve = []
        i2c_bus = 3
        state_dir = Path(".")

    config = Config()
    import logging
    log = logging.getLogger("test")


def test_all_device_kinds_have_a_model_adapter():
    assert {item.kind for item in descriptors()} == set(DeviceKind)


def test_descriptors_are_serializable_and_stable():
    item = descriptor("zelux.tof.st-vl53l5cx")
    assert item.to_dict()["kind"] == "tof"
    assert item.models == ("VL53L5CX",)


def test_factories_satisfy_complete_contracts():
    ctx = Context()
    assert isinstance(create("zelux.servo.feetech-sts-scs", ctx), ServoDriver)
    assert isinstance(create("zelux.tof.st-vl53l5cx", ctx, bus=3, addr=0x52), TofDriver)
    assert isinstance(create("zelux.camera.v4l2-libcamera", ctx), CameraDriver)
    assert isinstance(create("zelux.fan.radxa-pwm", ctx), FanDriver)
    assert isinstance(create("zelux.imu.multi-i2c", ctx), ImuDriver)


def test_hd1910_optional_maintenance_contract():
    from zelux_drivers import ServoCenterDriver
    assert isinstance(create("zelux.servo.feetech-sts-scs", Context()), ServoCenterDriver)
