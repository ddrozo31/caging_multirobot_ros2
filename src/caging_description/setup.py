from glob import glob

from setuptools import find_packages, setup

package_name = 'caging_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    # data_files = (destination folder inside install/, [files to copy]).
    # glob() only matches files of ONE folder: every new sub-folder needs its own line.
    data_files=[
        # Marker file so ROS 2 can find this package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Robot model, launch files, RViz and ros2_control configuration
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Objects of the paper (Step 5): SDF models and their meshes
        ('share/' + package_name + '/objects', glob('objects/*.sdf')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Rozo-Osorio',
    maintainer_email='david.rozo31@eia.edu.co',
    description='Robot model (URDF/xacro) of the Yahboom ROSMASTER X3 and the objects of the caging workshop.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
