from glob import glob
import os

from setuptools import setup

package_name = 'skyw_detection'


setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        package_name + '.python',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join(package_name, 'launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboticistprogrammer',
    maintainer_email='user@example.com',
    description='Sky Warrior detection utilities (HSV pad detection + ROS2 node).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'color_detector_node = skyw_detection.python.color_detector_node:main',
            'qrcode_detector = skyw_detection.python.qr_code_detector_node:main',
        ],
    },
)

