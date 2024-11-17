from setuptools import setup

package_name = 'dynamic_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dynamic_tests_launch.py']),
    ],
    install_requires=['setuptools', 'ackermann_msgs', ''],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for dynamic tests with parameters.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_tests_node = dynamic_tests.dynamic_tests_node:main',
        ],
    },
)
