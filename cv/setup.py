from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    # install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ammarvora',
    maintainer_email='ammarmust4@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_object = cv.detect_object:main',
            'follow_object = cv.follow_object:main',
            'detect_lanes = cv.detect_lanes:main',
        ],
    },
)
