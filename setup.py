from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'adaface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Path launch directory add
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
      'setuptools',
    ],
    zip_safe=True,
    maintainer='minha',
    maintainer_email='alsgk0404@hanyang.ac.kr',
    description='Main Adaface ROS2 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'face_recognition = adaface.adaface_ros2:main',
        ],
    },
)
