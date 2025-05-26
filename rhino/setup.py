from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rhino'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*'))
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
            "joint_testor = rhino.joint_testor:main",
            "ikpy_testor = rhino.ikpy_testor:main",
            "task1 = rhino.task1:main",
            "feedback_test = rhino.feedback_test:main",
            "task2 = rhino.task2:main",
            "color_detector = rhino.color_detector:main"
        ],
    },
)

