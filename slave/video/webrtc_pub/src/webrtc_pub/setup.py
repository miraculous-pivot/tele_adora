from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'webrtc_pub'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='feng',
    maintainer_email='1608749191@qq.com',
    description='ROS2 HTTP MJPEG Camera Streaming Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'http_camera_node = webrtc_pub.http_camera_node:main',
        ],
    },
)
