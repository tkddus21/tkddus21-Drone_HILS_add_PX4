from setuptools import find_packages, setup

package_name = 'offboard'

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
    maintainer='jetson2',
    maintainer_email='jetson2@todo.todo',
    description='PX4 Offboard Control Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_mode = offboard.offboard_mode:main',
        ],
    },
)
