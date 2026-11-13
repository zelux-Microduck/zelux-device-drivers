from __future__ import annotations

import math

from zelux_drivers.radxa.imu import control_orientation


def test_upright_control_orientation_matches_robotd_frame() -> None:
    frame = control_orientation(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)

    assert frame["gravity"] == [0.0, -0.0, -1.0]
    assert frame["quaternion_wxyz"] == [1.0, 0.0, 0.0, 0.0]
    assert math.isclose(frame["gyro_rad_s"][0], math.pi)


def test_quaternion_stays_normalized() -> None:
    frame = control_orientation(12.0, -7.0, 31.0, 0.0, 0.0, 0.0)
    norm = math.sqrt(sum(value * value for value in frame["quaternion_wxyz"]))

    assert math.isclose(norm, 1.0, abs_tol=1e-12)
