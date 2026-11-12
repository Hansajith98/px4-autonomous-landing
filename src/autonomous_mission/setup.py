from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_follower = autonomous_mission.waypoint_follower:main',
            'takeoff_to_1m = autonomous_mission.takeoff_to_1m:main',
            'live_aruco_detector = autonomous_mission.live_aruco_detector:main',
            'aruco_pose_estimator = autonomous_mission.aruco_pose_estimator:main',
            'precision_landing_controller = autonomous_mission.precision_landing_controller:main',
        ],
    },
)
