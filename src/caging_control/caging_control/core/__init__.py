"""
core/: the math of the paper, in plain Python (+ numpy).

RULE: nothing in this folder imports ROS (rclpy, messages, ...).
That way every function can be tested in a Python shell, a notebook or
with pytest, without starting Gazebo.
"""
