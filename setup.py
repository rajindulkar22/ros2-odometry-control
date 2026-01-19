from setuptools import find_packages, setup

package_name = 'odometry_control'

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
    maintainer='raj',
    maintainer_email='raj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'physics_node = odometry_control.physics_node:main',
            'odometry = odometry_control.odometry:main',
            'square_drive = odometry_control.square_drive:main',
            'goal_controller = odometry_control.goal_controller:main',
            'sensor_fusion = odometry_control.sensor_fusion:main',
        ],
    },
)
