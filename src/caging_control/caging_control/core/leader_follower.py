"""
Leader-Follower formation controller (Tejada et al. 2025, §2.4, Fig. 8, Eq. 14-25).

The follower keeps the object caged by holding three numbers constant:

    eta = [d, alpha, theta_F]
      d       distance between the robot centers                     (Eq. 15)
      alpha   angle of the follower with respect to the line F -> L   (Eq. 16)
      theta_F heading of the follower

Each robot moves with a velocity u_r = [x_dot_R, y_dot_R, theta_dot] given in ITS
OWN frame (Eq. 14: q_dot_r = R(theta_r) u_r), the same body twist our mecanum
controller receives. The formation changes as

    eta_dot = A u_L - B u_F                                            (Eq. 21)

and the follower chooses u_F so that the error e = eta - eta* decays:

    u_F = B^-1 (A u_L - p_d)                                           (Eq. 22)
    p_d = [ d*_dot     - k_d     (d - d*),
            alpha*_dot - k_alpha (alpha - alpha*),
            thF*_dot   - k_thF   (theta_F - theta_F*) ]                (Eq. 23)
    =>  e_dot = -K e,   K = diag(k_d, k_alpha, k_thF)                  (Eq. 24-25)

References: d* constant, alpha* = 0 (the follower looks at the leader),
theta_F* = theta_L - pi, so d*_dot = 0, alpha*_dot = 0, thF*_dot = theta_L_dot.

A pose is a tuple (x, y, theta). No ROS here: only numpy and math.
"""
import math

import numpy as np

from caging_control.core.geometry import wrap_angle


def formation_state(leader, follower):
    """eta = (d, alpha, theta_F) from the two poses (Eq. 15-16).

        d     = sqrt((x_L - x_F)^2 + (y_L - y_F)^2)
        alpha = theta_F - atan2(y_L - y_F, x_L - x_F)     (wrapped to [-pi, pi])
    """
    dx = leader[0] - follower[0]
    dy = leader[1] - follower[1]
    d = math.hypot(dx, dy)
    alpha = wrap_angle(follower[2] - math.atan2(dy, dx))
    return d, alpha, follower[2]


def formation_matrices(d, alpha, theta_f, theta_l):
    """Matrices A and B of the formation model eta_dot = A u_L - B u_F (Eq. 21).

        gamma = theta_L - theta_F + alpha
        A = [[ cos(gamma),    -sin(gamma),    0],
             [-sin(gamma)/d,  -cos(gamma)/d,  0],
             [ 0,              0,             0]]
        B = [[ cos(alpha),    -sin(alpha),    0],
             [-sin(alpha)/d,  -cos(alpha)/d, -1],
             [ 0,              0,            -1]]

    det(B) = 1/d: B can be inverted for any d > 0.
    """
    gamma = theta_l - theta_f + alpha
    A = np.array([[math.cos(gamma), -math.sin(gamma), 0.0],
                  [-math.sin(gamma) / d, -math.cos(gamma) / d, 0.0],
                  [0.0, 0.0, 0.0]])
    B = np.array([[math.cos(alpha), -math.sin(alpha), 0.0],
                  [-math.sin(alpha) / d, -math.cos(alpha) / d, -1.0],
                  [0.0, 0.0, -1.0]])
    return A, B


def follower_command(leader, follower, u_leader, d_ref, alpha_ref, gains):
    """Body twist u_F of the follower (Eq. 22-23).

    leader, follower : poses (x, y, theta)
    u_leader         : body twist of the leader (vx, vy, wz)
    d_ref, alpha_ref : d*, alpha*   (theta_F* = theta_L - pi)
    gains            : (k_d, k_alpha, k_thF)

    Returns (u_F, e) with u_F = (vx, vy, wz) and e = (e_d, e_alpha, e_thF).
    """
    k_d, k_alpha, k_thf = gains
    d, alpha, theta_f = formation_state(leader, follower)
    theta_f_ref = leader[2] - math.pi

    # Errors (angles wrapped: the short way around)
    e = np.array([d - d_ref,
                  wrap_angle(alpha - alpha_ref),
                  wrap_angle(theta_f - theta_f_ref)])

    # Eq. 23 with d*_dot = 0, alpha*_dot = 0, theta_F*_dot = theta_L_dot = u_L[2]
    p_d = np.array([0.0 - k_d * e[0],
                    0.0 - k_alpha * e[1],
                    u_leader[2] - k_thf * e[2]])

    # Eq. 22
    A, B = formation_matrices(d, alpha, theta_f, leader[2])
    u_f = np.linalg.solve(B, A @ np.asarray(u_leader, dtype=float) - p_d)
    return (float(u_f[0]), float(u_f[1]), float(u_f[2])), (float(e[0]), float(e[1]), float(e[2]))
