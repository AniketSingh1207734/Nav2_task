from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch/'),
         glob('launch/*launch.[pxy][yma]*')),
         (os.path.join('share',package_name,'description/'),
          glob('description/*.urdf')),
        (os.path.join('share',package_name,'description/'),
          glob('description/*.xacro')),
        (os.path.join('share',package_name,'worlds/'),
          glob('worlds/*.world')),
        (os.path.join('share',package_name,'config/'),
          glob('config/*.yaml')),
        (os.path.join('share',package_name,'maps/'),
          glob('maps/*.yaml')),
        (os.path.join('share',package_name,'maps/'),
          glob('maps/*.pgm'))
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
            "test_node = my_robot.my_first_node:main",
            "robot_navigator=my_robot.robot_navigator:main",
            "set_init_amcl_pose=my_robot.set_init_amcl_pose:main"
        ],
    },
)
