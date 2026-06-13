from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'watermelon_demo'

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
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lior',
    maintainer_email='liorieiz1@gmail.com',
    description='UR5 weed detection and laser burning simulation demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_node = watermelon_demo.detection_node:main',
            'arm_controller_node = watermelon_demo.arm_controller_node:main',
            'laser_effect_node = watermelon_demo.laser_effect_node:main',
            'field_manager_node = watermelon_demo.field_manager_node:main',
        ],
    },
)
