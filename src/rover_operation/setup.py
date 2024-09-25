from setuptools import find_packages, setup

package_name = 'rover_operation'

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
    maintainer='dsu-aiml-509-37',
    maintainer_email='dsu-aiml-509-37@todo.todo',
    description='Node to send navigation data to rover',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_teleop_key = rover_operation.rover_teleop:main',
        ],
    },
)
