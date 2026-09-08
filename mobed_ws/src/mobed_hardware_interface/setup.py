from setuptools import find_packages, setup

package_name = 'mobed_hardware_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricardo',
    maintainer_email='ricardo@todo.todo',
    description='UDP hardware interface bridge between ROS2 control node and MuJoCo simulation on Mac',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_interface = mobed_hardware_interface.udp_interface:main'
        ],
    },
)
