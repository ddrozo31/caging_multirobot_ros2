from setuptools import find_packages, setup

package_name = 'caging_control'

setup(
    name=package_name,
    version='0.0.1',
    # find_packages() finds caging_control, caging_control.core and caging_control.nodes
    # (every folder with an __init__.py)
    packages=find_packages(exclude=['test']),
    data_files=[
        # Marker file so ROS 2 can find this package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Rozo-Osorio',
    maintainer_email='david.rozo31@eia.edu.co',
    description='Algorithms of Tejada et al. (2025): pure math in core/, ROS 2 wrappers in nodes/.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Executables:  ros2 run caging_control <name>
    #   name = python.module.path:function_to_call
    entry_points={
        'console_scripts': [
            'odom_error_monitor = caging_control.nodes.odom_error_monitor:main',
            'lqr_controller = caging_control.nodes.lqr_controller:main',
            'object_marker = caging_control.nodes.object_marker:main',
            'approach_manager = caging_control.nodes.approach_manager:main',
            'leader_follower = caging_control.nodes.leader_follower:main',
        ],
    },
)
