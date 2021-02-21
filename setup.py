from setuptools import find_packages
from setuptools import setup

package_name = 'ros2top'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['asciimatics', 'setuptools'],
    zip_safe=True,
    maintainer='Hironori Fujimoto',
    maintainer_email='fujimoto-weldyn@outlook.jp',
    url='https://github.com/fujimo-t/ros2top',
    description='Top-like monitoring command for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2top = ros2top.ros2top:main',
        ],
    },
)
