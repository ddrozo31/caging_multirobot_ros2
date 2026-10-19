"""
Measurement noise of a motion-capture system (Step 11).

The paper measured the poses with a Vicon system; a real measurement is never
perfect. Here we add zero-mean Gaussian (normal) noise to a perfect pose:

    x_meas     = x     + n_x,      n_x     ~ N(0, sigma_xy^2)
    y_meas     = y     + n_y,      n_y     ~ N(0, sigma_xy^2)
    theta_meas = theta + n_theta,  n_theta ~ N(0, sigma_yaw^2)

A pose is a tuple (x, y, theta): x, y in meters, theta in radians.
No ROS here: only numpy.
"""
import numpy as np

from caging_control.core.geometry import wrap_angle


def make_generator(seed):
    """Random number generator. The same seed always gives the same noise
    (a run can be repeated exactly); a different seed gives different noise."""
    return np.random.default_rng(seed)


def noisy_pose(pose, sigma_xy, sigma_yaw, generator):
    """Return the pose with Gaussian noise added (sigma = 0 -> no noise)."""
    x, y, theta = pose
    n_x, n_y = generator.normal(0.0, sigma_xy, size=2) if sigma_xy > 0 else (0.0, 0.0)
    n_theta = generator.normal(0.0, sigma_yaw) if sigma_yaw > 0 else 0.0
    return (x + n_x, y + n_y, wrap_angle(theta + n_theta))
