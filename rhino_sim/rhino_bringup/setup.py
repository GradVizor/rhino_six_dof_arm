from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rhino_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reishabh',
    maintainer_email='reishabhrathore2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "position_test = rhino_bringup.position_test:main",
            "ikpy_test = rhino_bringup.ikpy_test:main",
            "task1 = rhino_bringup.task1:main",
            "feedback_test = rhino_bringup.feedback_test:main",
            "task2 = rhino_bringup.task2:main"
        ],
    },
)

