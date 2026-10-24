import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'turtle_motion'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Victor Kamal',
    maintainer_email='student@alexu.edu.eg',
    description='Turtlesim motion control: goToPose, reset, square and speed-limit nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_pose_server = turtle_motion.go_to_pose_server:main',
            'reset_service = turtle_motion.reset_service:main',
            'square_action_server = turtle_motion.square_action_server:main',
        ],
    },
)
