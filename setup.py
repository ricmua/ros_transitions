from setuptools import setup

package_name = 'ros_transitions'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}/example'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a.whit',
    maintainer_email='nml@whit.contact',
    description='A ROS2 interface to the pytransitions Python state machine package.',
    license='Mozilla Public License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example = ros_transitions.example:main'
        ],
    },
)
