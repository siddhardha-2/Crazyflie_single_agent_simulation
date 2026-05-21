from setuptools import setup
import os
from glob import glob

package_name = 'Guidance'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='you@email.com',
    description='Guidance package for Crazyflie BMO simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pattern_node = Guidance.pattern_node:main',
        ],
    },
)