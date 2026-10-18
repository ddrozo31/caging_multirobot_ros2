"""
A* path planning on a grid (Tejada et al. 2025, §2.3: collision-free approach).

The floor is cut into square cells (an "occupancy grid"):

    occupied[row, col] = True   -> the robot CENTER must not be in this cell

Obstacles (the object, the other robot) are circles, made bigger by the size of
the robot ("inflation"), so we can plan for one point: the robot center.

    row = index along y,  col = index along x       (like an image: grid[y, x])

A* explores cells in the order of  f = g + h:
    g = length of the best path found from the start to this cell
    h = straight-line distance from this cell to the goal (never too big, so A*
        always finds the SHORTEST path)

No ROS here: only numpy, math and heapq.
"""
import heapq
import math

import numpy as np


class Grid:
    """A rectangle of the floor, cut into cells of size `resolution` [m]."""

    def __init__(self, x_min, x_max, y_min, y_max, resolution):
        self.x_min = x_min
        self.y_min = y_min
        self.resolution = resolution
        self.cols = int(round((x_max - x_min) / resolution))   # cells along x
        self.rows = int(round((y_max - y_min) / resolution))   # cells along y
        self.occupied = np.zeros((self.rows, self.cols), dtype=bool)
        self.circles = []   # exact obstacles (x, y, radius), to check points without rounding

        # Center of every cell, in meters (used to draw the circles)
        cols = np.arange(self.cols)
        rows = np.arange(self.rows)
        self.cell_x, self.cell_y = np.meshgrid(x_min + (cols + 0.5) * resolution,
                                               y_min + (rows + 0.5) * resolution)

    def world_to_cell(self, x, y):
        """Point (x, y) [m] -> cell (row, col)."""
        col = int((x - self.x_min) / self.resolution)
        row = int((y - self.y_min) / self.resolution)
        return row, col

    def cell_to_world(self, row, col):
        """Cell (row, col) -> its center (x, y) [m]."""
        return (self.x_min + (col + 0.5) * self.resolution,
                self.y_min + (row + 0.5) * self.resolution)

    def inside(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_free(self, row, col):
        return self.inside(row, col) and not self.occupied[row, col]

    def add_circle(self, x, y, radius):
        """Mark as occupied every cell whose center is closer than `radius` to (x, y)."""
        distance_squared = (self.cell_x - x) ** 2 + (self.cell_y - y) ** 2
        self.occupied |= distance_squared <= radius ** 2
        self.circles.append((x, y, radius))

    def point_is_free(self, x, y):
        """Exact test of a POINT (no rounding to cells): inside the area and outside every circle.

        A cell can be occupied although a point inside it is free: the cell is tested at its
        CENTER, which can be up to resolution * sqrt(2) / 2 (3.5 cm for 5 cm cells) away.
        """
        inside_area = (self.x_min <= x < self.x_min + self.cols * self.resolution
                       and self.y_min <= y < self.y_min + self.rows * self.resolution)
        return inside_area and all(math.hypot(x - cx, y - cy) > r for cx, cy, r in self.circles)


# The 8 neighbors of a cell: (row step, col step, cost in cells)
NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
             (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def astar(grid, start, goal):
    """Shortest path of free cells from `start` to `goal` (both (row, col)).

    Returns the list of cells [start, ..., goal], or None if there is no path.
    """
    if not grid.is_free(*goal):
        return None

    def h(cell):   # heuristic: straight-line distance to the goal, in cells
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

    g = {start: 0.0}          # best known cost from the start
    came_from = {}            # came_from[cell] = previous cell on the best path
    # Priority queue of (f, counter, cell). The counter only breaks ties.
    open_list = [(h(start), 0, start)]
    counter = 1
    closed = set()            # cells already expanded

    while open_list:
        _, _, cell = heapq.heappop(open_list)   # the cell with the smallest f
        if cell == goal:
            # Walk back from the goal to the start
            path = [cell]
            while cell in came_from:
                cell = came_from[cell]
                path.append(cell)
            return path[::-1]
        if cell in closed:
            continue
        closed.add(cell)

        for d_row, d_col, cost in NEIGHBORS:
            neighbor = (cell[0] + d_row, cell[1] + d_col)
            if not grid.is_free(*neighbor):
                continue
            # A diagonal step must not cut the corner of an obstacle
            if d_row != 0 and d_col != 0:
                if not (grid.is_free(cell[0] + d_row, cell[1]) and grid.is_free(cell[0], cell[1] + d_col)):
                    continue
            new_g = g[cell] + cost
            if new_g < g.get(neighbor, math.inf):
                g[neighbor] = new_g
                came_from[neighbor] = cell
                heapq.heappush(open_list, (new_g + h(neighbor), counter, neighbor))
                counter += 1
    return None


def line_is_free(grid, p1, p2):
    """True if the straight segment p1 -> p2 (points in meters) crosses only free cells.

    We check points along the segment every half cell.
    """
    length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    steps = max(1, int(length / (grid.resolution / 2)))
    for i in range(steps + 1):
        t = i / steps
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        if not grid.is_free(*grid.world_to_cell(x, y)):
            return False
    return True


def simplify_path(grid, points):
    """Keep only the corners of a path (points in meters).

    From each kept point, jump to the FARTHEST later point that can be reached
    in a straight free line. A 5 cm-step path of 60 points becomes 2-4 points,
    so the robot does not stop and go at every cell.
    """
    if len(points) <= 2:
        return list(points)
    kept = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not line_is_free(grid, points[i], points[j]):
            j -= 1
        kept.append(points[j])
        i = j
    return kept
