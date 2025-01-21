from setuptools import find_packages, setup

package_name = 'turtle_localization_pkg'

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
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "d1_publisher_with_noise = turtle_localization_pkg.d1_publisher:main",
            "d2_publisher_with_noise = turtle_localization_pkg.d2_publisher:main",
            "d3_publisher_with_noise = turtle_localization_pkg.d3_publisher:main",
            "d4_publisher_with_noise = turtle_localization_pkg.d4_publisher:main",
            "d5_publisher_with_noise = turtle_localization_pkg.d5_publisher:main",
            "d6_publisher_with_noise = turtle_localization_pkg.d6_publisher:main",
            "robot_position_ukf = turtle_localization_pkg.robot_position_ukf:main",
            "robot_position_noise = turtle_localization_pkg.robot_position_noise:main",
            "robot_position_std_kf = turtle_localization_pkg.robot_position_std_kf:main",
            "circle_motion = turtle_localization_pkg.circle_motion:main"
        ],
    },
)