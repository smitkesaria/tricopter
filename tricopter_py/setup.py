import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tricopter_py'

setup(
    name=package_name,
    version='0.0.0',
    packages= ['scservo_sdk',package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Smit',
    maintainer_email='smitkesaris@gmail.com',
    description='Python package for implmentation of tricopter',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller = tricopter_py.servo_controller:main',
            'imu_publisher = tricopter_py.imu_publisher:main',
        ],
    },
)
