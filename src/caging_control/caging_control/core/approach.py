"""
Approach poses of the leader and the follower (Tejada et al. 2025, §2.3, Fig. 7, Eq. 13).

Both robots must stop in front of the object, on opposite sides, looking at it:

            follower                object               leader
     (x_O + d_a cos, ...)  --->  (x_O, y_O, theta_O)  <---  (x_O - d_a cos, ...)

    d_a = 2 r_O + l_g            r_O: object radius, l_g: robot center -> gripper center

Grasp offset (our extension, Step 7): the V gripper holds a CORNER best (the corner
sits in the apex of the V). For the box the corners are at 45 deg from its axes, so the
approach axis is rotated: theta_grasp = theta_O + grasp_offset (box: pi/4). With
grasp_offset = 0 this is exactly Eq. 13 of the paper.

A pose is a tuple (x, y, theta), as in core/geometry.py. No ROS here.
"""
import math

from caging_control.core.geometry import wrap_angle


def approach_distance(object_radius, gripper_distance):
    """d_a = 2 r_O + l_g  (§2.3): distance from the object center to each robot center."""
    return 2.0 * object_radius + gripper_distance


def approach_poses(object_pose, object_radius, gripper_distance, grasp_offset=0.0):
    """Eq. 13: desired poses q_L* (leader) and q_F* (follower).

        q_L* = q_O + d_a [-cos(theta_O), -sin(theta_O),  0      ]
        q_F* = q_O + d_a [ cos(theta_O),  sin(theta_O), -pi/d_a ]

    The third row of q_F* gives theta_F* = theta_O - pi: the follower faces the
    leader, with the object between them.

    grasp_offset [rad] rotates the approach axis (theta_O -> theta_O + grasp_offset),
    e.g. pi/4 to take the box by its corners. 0 = the paper's Eq. 13.
    """
    x_o, y_o, theta_o = object_pose
    theta_o = theta_o + grasp_offset
    d_a = approach_distance(object_radius, gripper_distance)
    leader = (x_o - d_a * math.cos(theta_o),
              y_o - d_a * math.sin(theta_o),
              wrap_angle(theta_o))
    follower = (x_o + d_a * math.cos(theta_o),
                y_o + d_a * math.sin(theta_o),
                wrap_angle(theta_o - math.pi))
    return leader, follower
