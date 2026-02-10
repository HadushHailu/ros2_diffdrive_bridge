from setuptools import setup

package_name = 'diffdrive_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Hadush',
    maintainer_email='hadush@example.com',
    description='ROS2 bridge: /cmd_vel -> Arduino serial motor commands',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = diffdrive_serial_bridge.bridge_node:main',
        ],
    },
)

