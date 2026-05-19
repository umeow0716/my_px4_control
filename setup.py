from setuptools import find_packages, setup
from glob import glob

package_name = 'my_px4_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='umeow',
    maintainer_email='jj30462281@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'offboard_control = my_px4_control.offboard_control:main',
            'command_publisher = my_px4_control.command_publisher:main',
            'only_heartbeat = my_px4_control.only_heartbeat:main',
            'position_record = my_px4_control.position_record:main',
            'websocket_publisher = my_px4_control.websocket_publisher:main',
            'websocket_control = my_px4_control.websocket_control:main',
        ],
    },
)
