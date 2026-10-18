"""
Unit tests for core/approach.py (Eq. 13) and core/astar.py (no ROS, no Gazebo needed).

Run:
  colcon test --packages-select caging_control --event-handlers console_direct+
or, from src/caging_control:
  python3 -m pytest test/
"""
import math

from caging_control.core.approach import approach_distance, approach_poses
from caging_control.core.astar import Grid, astar, line_is_free, simplify_path


def test_approach_distance_table1():
    # d_a = 2 r_O + l_g = 2 * 0.15 + 0.12 (Table 1)
    assert math.isclose(approach_distance(0.15, 0.12), 0.42)


def test_approach_poses_table1():
    # Object at q_O = [0, 0, pi] (Table 1)
    leader, follower = approach_poses((0.0, 0.0, math.pi), 0.15, 0.12)
    assert math.isclose(leader[0], 0.42) and math.isclose(leader[1], 0.0, abs_tol=1e-12)
    assert math.isclose(abs(leader[2]), math.pi)          # looks along -x, at the object
    assert math.isclose(follower[0], -0.42) and math.isclose(follower[1], 0.0, abs_tol=1e-12)
    assert math.isclose(follower[2], 0.0, abs_tol=1e-12)  # looks along +x, at the leader


def test_approach_poses_face_each_other():
    # For any object pose: the object center is the midpoint, and headings differ by pi
    leader, follower = approach_poses((0.3, -0.2, 0.7), 0.15, 0.12)
    assert math.isclose((leader[0] + follower[0]) / 2, 0.3)
    assert math.isclose((leader[1] + follower[1]) / 2, -0.2)
    assert math.isclose(abs(math.remainder(leader[2] - follower[2], 2 * math.pi)), math.pi)


def test_astar_free_grid_is_straight():
    grid = Grid(0.0, 1.0, 0.0, 1.0, 0.1)
    path = astar(grid, (0, 0), (9, 9))
    assert path[0] == (0, 0) and path[-1] == (9, 9)
    assert len(path) == 10               # pure diagonal: 9 steps


def test_astar_goes_around_obstacle():
    grid = Grid(-1.0, 1.0, -1.0, 1.0, 0.05)
    grid.add_circle(0.0, 0.0, 0.4)
    start = grid.world_to_cell(0.0, -0.8)
    goal = grid.world_to_cell(0.0, 0.8)
    path = astar(grid, start, goal)
    assert path is not None
    assert all(grid.is_free(*cell) for cell in path)
    # It must leave the straight line x = 0 to get around the circle
    assert max(abs(grid.cell_to_world(*cell)[0]) for cell in path) > 0.4


def test_astar_no_path():
    grid = Grid(0.0, 1.0, 0.0, 1.0, 0.1)
    grid.occupied[:, 5] = True           # a wall from top to bottom
    assert astar(grid, (0, 0), (0, 9)) is None


def test_simplify_path_keeps_free_corners():
    grid = Grid(-1.0, 1.0, -1.0, 1.0, 0.05)
    grid.add_circle(0.0, 0.0, 0.4)
    start = grid.world_to_cell(0.0, -0.8)
    goal = grid.world_to_cell(0.0, 0.8)
    points = [grid.cell_to_world(*cell) for cell in astar(grid, start, goal)]
    short = simplify_path(grid, points)
    assert short[0] == points[0] and short[-1] == points[-1]
    assert len(short) < len(points) / 5
    for p1, p2 in zip(short, short[1:]):
        assert line_is_free(grid, p1, p2)
