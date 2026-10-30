from glob import glob

from setuptools import find_packages, setup

package_name = 'calibration_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gfranzese',
    maintainer_email='franzesegiovanni@outlook.com',
    description='Joint state recorder for the MUKCa kinematic calibration',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'record_joint_states_dataset = calibration_tools.record_joint_states_dataset:main',
        ],
    },
)
