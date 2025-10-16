from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #line for launch file:
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shadow0',
    maintainer_email='Shahazad.abdulla.engineer@gmail.com',
    description='Package of tom and jerry usign turtlesim for learning purpose with projects',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'simple_mover = turtle_control.simple_mover:main',
            'turtle_tom = turtle_control.turtle_tom:main'
        ],
    },
)
