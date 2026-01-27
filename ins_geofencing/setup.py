from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ins_geofencing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tucker',
    maintainer_email='peter31101@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zone_recorder = ins_geofencing.zone_recorder_pro:main',
            'zone_visualizer = ins_geofencing.zone_visualizer:main',
            'geofence_monitor = ins_geofencing.geofence_monitor:main',
        ],
    },
)
