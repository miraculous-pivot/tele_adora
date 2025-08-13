from setuptools import find_packages, setup

package_name = 'my_teleop_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zz',
    maintainer_email='18092288389@163.com',
    description='keyboards teleoperation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = my_teleop_pkg.teleop_node:main',
            'dual_hand_teleop = my_teleop_pkg.dual_hand_teleop:main',
            'bimanual_teleop = my_teleop_pkg.bimanual_teleop:main',
        ],
    },
)
