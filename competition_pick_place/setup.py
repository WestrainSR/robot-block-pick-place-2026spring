import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'competition_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='student@example.com',
    description='Closed-loop block pick and place task runner for Hiwonder LanderPi.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'competition_node = competition_pick_place.competition_node:main',
            'capture_dataset = competition_pick_place.capture_dataset:main',
            'action_group_runner = competition_pick_place.action_group_runner:main',
            'open_loop_drive = competition_pick_place.open_loop_drive:main',
            'place_target_align = competition_pick_place.place_target_align:main',
            'delivery_agent = competition_pick_place.delivery_agent:main',
        ],
    },
)
