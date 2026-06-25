from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'real_simulation_ur5'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lior',
    maintainer_email='liorieiz1@gmail.com',
    description='Real-hardware UR5 weed detection and laser targeting',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_node      = real_simulation_ur5.detection_node:main',
            'arm_controller_node = real_simulation_ur5.arm_controller_node:main',
            'laser_effect_node   = real_simulation_ur5.laser_effect_node:main',
            'sim_detection_node  = real_simulation_ur5.sim_detection_node:main',
            'sim_arm_controller  = real_simulation_ur5.sim_arm_controller:main',
            'joint_state_restamper = real_simulation_ur5.joint_state_restamper:main',
        ],
    },
)
