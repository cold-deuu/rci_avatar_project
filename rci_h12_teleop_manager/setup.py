from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rci_h12_teleop_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='lee081847@khu.ac.kr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rb5_controller_node = rci_h12_teleop_manager.rb5_controller_node:main',
            'rci_h12_teleop_arm_node = rci_h12_teleop_manager.rci_h12_teleop_arm_main:main',
            'unity_wrist_node = rci_h12_teleop_manager.unity_wrist_publisher:main',
            'rci_h12_teleop_hand_node = rci_h12_teleop_manager.rci_h12_teleop_hand_main:main',
        ],
    },
)
