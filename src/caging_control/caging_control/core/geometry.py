"""
Planar geometry helpers: angles, quaternions and poses (x, y, theta).

A pose is a tuple (x, y, theta):  x, y in meters, theta in radians.
"""
import math


def wrap_angle(angle):
    """Return the same angle inside [-pi, pi].

    Example: wrap_angle(3*pi/2) = -pi/2  (turning 270 deg left = 90 deg right)
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(qx, qy, qz, qw):
    """Rotation about z (yaw) of a quaternion.

    ROS messages store orientations as quaternions (qx, qy, qz, qw).
    For a robot on the floor we only need the yaw angle theta.
    """
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def transform_pose(pose, frame_origin):
    """Express a pose given in a frame F in the parent frame.

    pose         = (x, y, theta) measured in frame F
    frame_origin = (x0, y0, theta0) = pose of frame F in the parent frame

    This is the rotation matrix R(theta0) of Eq. 3 plus a translation:
        [x_parent]   [x0]   [cos(theta0)  -sin(theta0)] [x]
        [y_parent] = [y0] + [sin(theta0)   cos(theta0)] [y]
        theta_parent = theta0 + theta
    """
    x, y, theta = pose
    x0, y0, theta0 = frame_origin
    c = math.cos(theta0)
    s = math.sin(theta0)
    return (x0 + c * x - s * y,
            y0 + s * x + c * y,
            wrap_angle(theta0 + theta))


def global_to_robot(vx, vy, theta):
    """Rotate a velocity from the global frame {G} to the robot frame {R}.

    Inverse of Eq. 3: [vx_R, vy_R] = R(theta)^T [vx, vy]
        vx_R =  cos(theta) vx + sin(theta) vy
        vy_R = -sin(theta) vx + cos(theta) vy
    (The angular velocity is the same in both frames.)
    """
    c = math.cos(theta)
    s = math.sin(theta)
    return c * vx + s * vy, -s * vx + c * vy


def pose_error(estimated, reference):
    """Error estimated - reference.

    Returns (ex, ey, etheta, distance):
      ex, ey   : position error [m]
      etheta   : heading error, wrapped to [-pi, pi] [rad]
      distance : sqrt(ex^2 + ey^2) [m]
    """
    ex = estimated[0] - reference[0]
    ey = estimated[1] - reference[1]
    etheta = wrap_angle(estimated[2] - reference[2])
    return ex, ey, etheta, math.hypot(ex, ey)
