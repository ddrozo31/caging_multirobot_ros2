from glob import glob

from setuptools import find_packages, setup

package_name = 'caging_bringup'

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
        # Launch files and parameter files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Rozo-Osorio',
    maintainer_email='david.rozo31@eia.edu.co',
    description='Launch files and parameters that tie the caging workshop together.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
