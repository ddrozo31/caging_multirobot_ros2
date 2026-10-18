"""
Unit tests for core/trajectory.py (no ROS, no Gazebo needed).

Run:
  colcon test --packages-select caging_control --event-handlers console_direct+
or, from src/caging_control:
  python3 -m pytest test/
"""
import math

from caging_control.core.trajectory import circle_pose, circle_waypoints, leader_goal_from_object


def test_circle_has_n_points_on_the_circle_and_closes():
    waypoints = circle_waypoints(n=30, radius=1.0)
    assert len(waypoints) == 30
    assert all(math.isclose(math.hypot(x, y), 1.0) for x, y, _ in waypoints)
    first, last = waypoints[0], waypoints[-1]
    assert math.isclose(first[0], 1.0) and math.isclose(first[1], 0.0, abs_tol=1e-12)
    assert math.isclose(last[0], 1.0) and math.isclose(last[1], 0.0, abs_tol=1e-9)   # phi = 2 pi


def test_orientations():
    phi = 2 * math.pi * 5 / 29
    _, _, static = circle_waypoints(orientation='static')[5]
    _, _, tangent = circle_waypoints(orientation='tangent')[5]
    _, _, radial = circle_waypoints(orientation='radial')[5]
    assert math.isclose(abs(static), math.pi)
    assert math.isclose(radial, phi)
    assert math.isclose(tangent, phi + math.pi / 2)


def test_eq26_leader_behind_object():
    # Object at (1, 0) with theta = pi: the leader looks along -x, so it stands at +x
    x, y, theta = leader_goal_from_object((1.0, 0.0, math.pi), 0.52)
    assert math.isclose(x, 1.26) and math.isclose(y, 0.0, abs_tol=1e-12)
    assert math.isclose(theta, math.pi)
    # ... and the object is d/2 in front of the leader
    assert math.isclose(x + 0.26 * math.cos(theta), 1.0)


def test_circle_pose_matches_waypoints():
    # The smooth reference passes exactly through the waypoints of the paper
    for orientation in ('static', 'tangent', 'radial'):
        waypoints = circle_waypoints(orientation=orientation)
        for i in (0, 10, 29):
            pose = circle_pose(2 * math.pi * i / 29, orientation=orientation)
            assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(pose, waypoints[i]))
