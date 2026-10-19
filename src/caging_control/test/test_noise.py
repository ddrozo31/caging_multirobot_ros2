"""
Unit tests for core/noise.py (Step 11). No ROS needed.

Run:  python3 -m pytest test/test_noise.py   (from src/caging_control)
"""
import math

import numpy as np

from caging_control.core.noise import make_generator, noisy_pose


def test_zero_noise_returns_the_same_pose():
    pose = (1.0, -2.0, 0.5)
    assert noisy_pose(pose, 0.0, 0.0, make_generator(1)) == pose


def test_same_seed_same_noise():
    pose = (0.0, 0.0, 0.0)
    a = noisy_pose(pose, 0.002, 0.005, make_generator(7))
    b = noisy_pose(pose, 0.002, 0.005, make_generator(7))
    c = noisy_pose(pose, 0.002, 0.005, make_generator(8))
    assert a == b       # repeatable
    assert a != c       # another seed, another noise


def test_noise_has_the_requested_standard_deviation():
    generator = make_generator(42)
    samples = np.array([noisy_pose((0.0, 0.0, 0.0), 0.002, 0.005, generator) for _ in range(20000)])
    assert abs(samples[:, 0].mean()) < 1e-4                    # zero mean
    assert math.isclose(samples[:, 0].std(), 0.002, rel_tol=0.05)
    assert math.isclose(samples[:, 1].std(), 0.002, rel_tol=0.05)
    assert math.isclose(samples[:, 2].std(), 0.005, rel_tol=0.05)


def test_angle_stays_wrapped():
    theta = noisy_pose((0.0, 0.0, math.pi), 0.0, 0.5, make_generator(3))[2]
    assert -math.pi <= theta <= math.pi
