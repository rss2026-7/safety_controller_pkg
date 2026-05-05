import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'safety_controller_pkg'

setup(
    name=package_name,
    version='2.5.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='racecar@todo.todo',
    description='Advanced safety controller with velocity-dependent braking for Final Challenge 2026',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_controller_pkg = safety_controller_pkg.safety_node:main',
            'example_forward = safety_controller_pkg.example_forward:main',
        ],
    },
)
