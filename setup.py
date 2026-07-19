from setuptools import find_packages, setup

package_name = 'pixhawk_servo_cntl'

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
    maintainer='quad',
    maintainer_email='quad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'servo_offboard_test_node = pixhawk_servo_cntl.servo_test_node:main',
            'servo_bridge_node = pixhawk_servo_cntl.servo_bridge_node:main',
            'servo_control_node = pixhawk_servo_cntl.servo_control_node:main',
        ],
    },
)
