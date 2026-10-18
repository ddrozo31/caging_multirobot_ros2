"""
Unit tests for core/lqr.py and global_to_robot (no ROS, no Gazebo needed).

Run:
  colcon test --packages-select caging_control --event-handlers console_direct+
or, from src/caging_control:
  python3 -m pytest test/
"""
import math

import numpy as np

from caging_control.core.geometry import global_to_robot
from caging_control.core.lqr import lqr_step, paper_gain, saturate, solve_riccati


def test_riccati_closed_form():
    # With A = B = I and diagonal Q = q I, R = r I every axis is the same scalar
    # problem. Eq. 12 becomes P^2 - q P - q r = 0, so P = (q + sqrt(q^2 + 4 q r)) / 2
    # and Eq. 11 gives K = P / (r + P).
    q, r = 0.1, 0.8
    K, P = paper_gain(q, r)
    p_expected = (q + math.sqrt(q * q + 4 * q * r)) / 2
    k_expected = p_expected / (r + p_expected)
    assert np.allclose(P, p_expected * np.eye(3))
    assert np.allclose(K, k_expected * np.eye(3))
    assert math.isclose(k_expected, 0.2965, abs_tol=1e-4)   # value quoted in the step guide


def test_riccati_residual():
    # A less trivial system: P from solve_riccati must satisfy Eq. 12
    A = np.array([[1.0, 0.1], [0.0, 1.0]])
    B = np.array([[0.0], [0.1]])
    Q = np.eye(2)
    R = np.array([[0.5]])
    P = solve_riccati(A, B, Q, R)
    rhs = A.T @ (P - P @ B @ np.linalg.inv(B.T @ P @ B + R) @ B.T @ P) @ A + Q
    assert np.allclose(P, rhs)


def test_closed_loop_converges():
    # Linearized model (Eq. 8) with v = K (q* - q): the error shrinks by (1 - K) each step
    K, _ = paper_gain()
    pose = (0.5, -2.5, math.pi / 2)
    goal = (0.0, 0.0, math.pi)
    for _ in range(100):
        vx, vy, vtheta = lqr_step(K, pose, goal)
        pose = (pose[0] + vx, pose[1] + vy, pose[2] + vtheta)
    assert math.isclose(pose[0], goal[0], abs_tol=1e-6)
    assert math.isclose(pose[1], goal[1], abs_tol=1e-6)
    assert math.isclose(pose[2], goal[2], abs_tol=1e-6)


def test_heading_error_short_way():
    # From 170 deg to -170 deg: turn +20 deg, not -340 deg
    K, _ = paper_gain()
    _, _, vtheta = lqr_step(K, (0.0, 0.0, math.radians(170)), (0.0, 0.0, math.radians(-170)))
    assert vtheta > 0
    assert math.isclose(vtheta, K[2, 2] * math.radians(20))


def test_saturate_keeps_direction():
    vx, vy, wz = saturate(3.0, 4.0, 5.0, max_linear=0.3, max_angular=1.0)
    assert math.isclose(math.hypot(vx, vy), 0.3)
    assert math.isclose(vy / vx, 4.0 / 3.0)   # same direction as before
    assert wz == 1.0
    # below the limits nothing changes
    assert saturate(0.1, 0.1, -0.5, 0.3, 1.0) == (0.1, 0.1, -0.5)


def test_global_to_robot():
    # Robot facing +y (theta = 90 deg): a global velocity along +y is "forward" for it
    vx_r, vy_r = global_to_robot(0.0, 1.0, math.pi / 2)
    assert math.isclose(vx_r, 1.0)
    assert math.isclose(vy_r, 0.0, abs_tol=1e-12)
    # ... and a global velocity along +x is to its RIGHT (negative y_R)
    vx_r, vy_r = global_to_robot(1.0, 0.0, math.pi / 2)
    assert math.isclose(vx_r, 0.0, abs_tol=1e-12)
    assert math.isclose(vy_r, -1.0)
