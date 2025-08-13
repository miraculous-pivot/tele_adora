import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vr_arm_sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'script'), glob('script/*.sh')),
        (os.path.join('share', package_name), ['init.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='feng',
    maintainer_email='1608749191@qq.com',
    description='VR arm synchronization package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_arm_sync_node = vr_arm_sync.vr_arm_sync_node:main',
        ],
    },
)
