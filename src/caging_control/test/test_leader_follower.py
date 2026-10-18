"""
Unit tests for core/leader_follower.py (no ROS, no Gazebo needed).

Run:
  colcon test --packages-select caging_control --event-handlers console_direct+
or, from src/caging_control:
  python3 -m pytest test/
"""
import math

import numpy as np

from caging_control.core.leader_follower import (
    follower_command, formation_matrices, formation_state)


def rotation(theta):
    """R(theta) of Eq. 3: body twist -> global velocity."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def move(pose, u, dt):
    """Eq. 14 with Euler: q(k) = q(k-1) + R(theta) u dt."""
    return tuple(np.asarray(pose) + dt * rotation(pose[2]) @ np.asarray(u))


def test_formation_state_facing():
    # Follower at the origin looking at the leader 0.52 m away along +x
    d, alpha, theta_f = formation_state((0.52, 0.0, math.pi), (0.0, 0.0, 0.0))
    assert math.isclose(d, 0.52) and math.isclose(alpha, 0.0, abs_tol=1e-12)
    assert theta_f == 0.0


def test_matrices_match_numerical_derivative():
    # Eq. 21 must give the same eta_dot as moving both robots a tiny step (Eq. 14)
    rng = np.random.default_rng(0)
    for _ in range(200):
        leader = tuple(rng.uniform(-1, 1, 3) * [1, 1, 3])
        follower = tuple(rng.uniform(-1, 1, 3) * [1, 1, 3])
        u_l, u_f = rng.uniform(-1, 1, 3), rng.uniform(-1, 1, 3)
        h = 1e-7
        eta0 = np.array(formation_state(leader, follower))
        eta1 = np.array(formation_state(move(leader, u_l, h), move(follower, u_f, h)))
        diff = eta1 - eta0
        diff[1] = math.remainder(diff[1], 2 * math.pi)
        numeric = diff / h
        d, alpha, theta_f = eta0
        A, B = formation_matrices(d, alpha, theta_f, leader[2])
        assert np.allclose(numeric, A @ u_l - B @ u_f, atol=1e-4 / min(d, 1.0) ** 2)


def test_b_determinant_is_one_over_d():
    A, B = formation_matrices(0.52, 0.3, 1.0, -2.0)
    assert math.isclose(np.linalg.det(B), 1 / 0.52)


def test_closed_loop_error_decays_like_eq25():
    # Leader drives and turns; the follower starts 3 cm too far and 0.1 rad off.
    # Eq. 25: each error decays as exp(-k t).
    gains = (2.0, 1.0, 1.0)
    leader = (0.0, 0.0, math.pi)
    follower = (-0.55, 0.02, 0.1)
    u_l = (0.05, 0.03, 0.2)
    dt, t_end = 0.001, 2.0
    _, e0 = follower_command(leader, follower, u_l, 0.52, 0.0, gains)
    for _ in range(int(t_end / dt)):
        u_f, _ = follower_command(leader, follower, u_l, 0.52, 0.0, gains)
        leader, follower = move(leader, u_l, dt), move(follower, u_f, dt)
    _, e = follower_command(leader, follower, u_l, 0.52, 0.0, gains)
    for i, k in enumerate(gains):
        assert math.isclose(e[i], e0[i] * math.exp(-k * t_end), rel_tol=0.02, abs_tol=1e-4)


def test_formation_at_rest_needs_no_command():
    # Perfect formation and a still leader -> the follower does not move
    u_f, e = follower_command((0.26, 0.0, math.pi), (-0.26, 0.0, 0.0), (0, 0, 0), 0.52, 0.0, (2, 1, 1))
    assert np.allclose(u_f, 0.0, atol=1e-12) and np.allclose(e, 0.0, atol=1e-12)
