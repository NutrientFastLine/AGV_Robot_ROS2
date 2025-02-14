import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*') + glob('launch/*/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*') + glob('config/slam_toolbox/*.*')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='feiyu_orinnx',
    maintainer_email='Mr.zhu345@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
