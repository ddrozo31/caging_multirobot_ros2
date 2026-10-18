"""
LQR pose controller of ONE omnidirectional robot (Tejada et al. 2025, §2.2, Eq. 7-12).

The whole idea in four lines:

  1. Discrete robot model (Eq. 6):        q(k) = q(k-1) + Ts * H(theta) * mu(k-1)
  2. Feedback linearization (Eq. 7):      mu = (Ts * H(theta))^-1 * v
     -> linear model (Eq. 8):             q(k) = q(k-1) + v(k-1)        (A_l = B_l = I)
  3. LQR on that linear model (Eq. 9-12): v = K (q* - q)
  4. Back to the robot:                   v / Ts = global velocity [m/s, rad/s]

So "v" is how much the pose should change in ONE sample (meters and radians per
step), not a velocity. Dividing by Ts gives the velocity in the global frame {G}.

A pose is a tuple (x, y, theta), as in core/geometry.py.
No ROS here: only numpy and math.
"""
import math

import numpy as np

from caging_control.core.geometry import wrap_angle


def solve_riccati(A, B, Q, R, tolerance=1e-12, max_iterations=10000):
    """Solve the discrete algebraic Riccati equation (Eq. 12) by iteration.

        P = A^T (P - P B (B^T P B + R)^-1 B^T P) A + Q            (Eq. 12)

    Start with P = Q and apply the right-hand side again and again until
    P stops changing. (The paper writes A_l instead of A_l^T: for A_l = I
    both are the same.)
    """
    P = Q.copy()
    for _ in range(max_iterations):
        inner = P - P @ B @ np.linalg.inv(B.T @ P @ B + R) @ B.T @ P
        P_next = A.T @ inner @ A + Q
        if np.max(np.abs(P_next - P)) < tolerance:
            return P_next
        P = P_next
    raise RuntimeError('Riccati iteration did not converge')


def lqr_gain(A, B, Q, R):
    """LQR gain K (Eq. 11) and the Riccati solution P (Eq. 12).

        K = (R + B^T P B)^-1 B^T P A                               (Eq. 11)
    """
    P = solve_riccati(A, B, Q, R)
    K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
    return K, P


def paper_gain(q_weight=0.1, r_weight=0.8):
    """K and P for the linearized model of the paper (Eq. 8) and Table 1.

        A_l = B_l = I (3x3),   Q = 0.1 I,   R = 0.8 I
    """
    identity = np.eye(3)
    return lqr_gain(identity, identity, q_weight * identity, r_weight * identity)


def lqr_step(K, pose, goal):
    """Control law (Eq. 10): v = K (q* - q).

    Returns v = (vx, vy, vtheta): the change of the pose wanted for the NEXT
    sample, in the global frame {G} [m per step, m per step, rad per step].

    The heading error is wrapped to [-pi, pi], so the robot always turns the
    short way (from 170 deg to -170 deg it turns 20 deg, not 340 deg).
    """
    error = np.array([goal[0] - pose[0],
                      goal[1] - pose[1],
                      wrap_angle(goal[2] - pose[2])])
    v = K @ error
    return float(v[0]), float(v[1]), float(v[2])


def saturate(vx, vy, wz, max_linear, max_angular):
    """Limit the velocity so the real wheels could follow it.

    - Linear part: if the speed sqrt(vx^2 + vy^2) is above max_linear, BOTH
      components are scaled down by the same factor. The robot keeps moving
      straight toward the goal, only slower. (Clipping vx and vy separately
      would bend the path.)
    - Angular part: clipped to [-max_angular, max_angular].
    """
    speed = math.hypot(vx, vy)
    if speed > max_linear:
        scale = max_linear / speed
        vx = vx * scale
        vy = vy * scale
    wz = max(-max_angular, min(max_angular, wz))
    return vx, vy, wz
