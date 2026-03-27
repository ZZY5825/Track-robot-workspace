from setuptools import setup

package_name = 'track_robot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pynput'
    ],
    zip_safe=True,
    maintainer='track-robot',
    maintainer_email='track-robot@todo.todo',
    description='Teleoperation nodes for track robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop_node = track_robot_teleop.keyboard_teleop_node:main',
        ],
    },
)
