from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ros2test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viet',
    maintainer_email='viet@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test = ros2test.test_node:main",
            "keyboard = ros2test.keyboard:main",
            "lane_follower = ros2test.lane_follower:main",
            "nav_goal_sender = ros2test.nav_goal_sender:main",
            "odom = ros2test.odom:main",
            "waypoints = ros2test.waypoints:main",
            "pure_pursuit = ros2test.pure_pursuit:main"
        ],
    },
)
