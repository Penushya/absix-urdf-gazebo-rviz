from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'forward_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Forward Kinematics package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_node = forward_kinematics.fk_node:main',
            'workspace_checker = forward_kinematics.workspace_checker:main',
            'interactive_fk = forward_kinematics.interactive_fk:main',
            'joint_commander = forward_kinematics.joint_commander:main',
            'ik_node = forward_kinematics.ik_node:main',
            'ik_service = forward_kinematics.ik_service:main',
            'interactive_ik = forward_kinematics.interactive_ik:main',
        ],
    },
)