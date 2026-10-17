"""
Unit tests for core/geometry.py (no ROS, no Gazebo needed).

Run:
  colcon test --packages-select caging_control --event-handlers console_direct+
or, from src/caging_control:
  python3 -m pytest test/
"""
import math

from caging_control.core.geometry import (
    pose_error, transform_pose, wrap_angle, yaw_from_quaternion)


def test_wrap_angle():
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(wrap_angle(-3 * math.pi / 2), math.pi / 2)
    assert math.isclose(wrap_angle(0.5), 0.5)


def test_yaw_from_quaternion():
    theta = 1.2
    # quaternion of a rotation theta about z: (0, 0, sin(theta/2), cos(theta/2))
    assert math.isclose(yaw_from_quaternion(0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2)), theta)


def test_transform_pose():
    # Frame F placed at (1, 2) and rotated 90 deg: its x axis points along world y
    origin = (1.0, 2.0, math.pi / 2)
    x, y, theta = transform_pose((1.0, 0.0, 0.0), origin)
    assert math.isclose(x, 1.0, abs_tol=1e-12)
    assert math.isclose(y, 3.0)
    assert math.isclose(theta, math.pi / 2)


def test_pose_error():
    ex, ey, etheta, distance = pose_error((3.0, 4.0, math.pi - 0.1), (0.0, 0.0, -math.pi + 0.1))
    assert (ex, ey) == (3.0, 4.0)
    assert math.isclose(distance, 5.0)
    assert math.isclose(etheta, -0.2)   # wrapped: the short way around
