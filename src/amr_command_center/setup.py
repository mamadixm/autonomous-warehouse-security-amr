import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_command_center'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmdixm',
    maintainer_email='mohammadmahdifarahani@gmail.com',
    description='Command Center Backend for AMR Surveillance & Security',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_center_node = amr_command_center.command_center_node:main',
            'mission_orchestrator = amr_command_center.mission_orchestrator:main',
        ],
    },
)
