"""
Object trajectories and the control point translation
(Tejada et al. 2025, §2.4.2, §2.5, Eq. 26-27).

The multi-robot system (MRS) is steered from the OBJECT center: we say where the
object must be, and Eq. 26 tells where the LEADER must be for that:

    q_L* = q_O* + (d/2) [-cos(theta_L*), -sin(theta_L*), 0],   theta_L* = theta_O*     (Eq. 26)

(the leader stands d/2 behind the object, looking at it; d = distance between the
robots). The orientation of the MRS is the heading of the leader (theta_MRS = theta_L).

Circular trajectory of §2.5 (radius 1 m, N = 30 waypoints, 30 s):

    phi_i = 2 pi i / (N - 1),   i = 0 ... N-1
    x* = cos(phi_i),  y* = sin(phi_i)
    Smooth reference (Step 9, our option): phi(t) = 2 pi t / T, updated at 20 Hz,
    instead of jumping to the next waypoint every T / N seconds (the paper).

    theta*: experiment 1 'static'  -> pi
            experiment 2 'tangent' -> phi_i + pi/2   (the text: "tangent to the circle")
                         'radial'  -> atan2(y*, x*) = phi_i   (the printed formula)

A pose is a tuple (x, y, theta). No ROS here.
"""
import math

from caging_control.core.geometry import wrap_angle


def circle_pose(phi, radius=1.0, center=(0.0, 0.0), orientation='static', static_heading=math.pi):
    """Desired object pose q_MRS*(phi) at the angle phi of the circle (counter-clockwise)."""
    if orientation not in ('static', 'tangent', 'radial'):
        raise ValueError(f'orientation must be static, tangent or radial, not {orientation}')
    x = center[0] + radius * math.cos(phi)
    y = center[1] + radius * math.sin(phi)
    if orientation == 'static':
        theta = static_heading
    elif orientation == 'tangent':
        theta = phi + math.pi / 2.0          # direction of travel (counter-clockwise)
    else:
        theta = math.atan2(y - center[1], x - center[0])   # pointing away from the center
    return (x, y, wrap_angle(theta))


def circle_waypoints(n=30, radius=1.0, center=(0.0, 0.0), orientation='static', static_heading=math.pi):
    """The N waypoints of §2.5: q_MRS*(phi_i), phi_i = 2 pi i / (N - 1)."""
    return [circle_pose(2.0 * math.pi * i / (n - 1), radius, center, orientation, static_heading)
            for i in range(n)]


def leader_goal_from_object(object_goal, distance):
    """Eq. 26: where the leader must be so that the object is at `object_goal`."""
    x_o, y_o, theta = object_goal
    return (x_o - distance / 2.0 * math.cos(theta),
            y_o - distance / 2.0 * math.sin(theta),
            theta)
