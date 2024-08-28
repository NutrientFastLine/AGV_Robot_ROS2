import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imu_driver_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        # 'pyserial',  # 在这里添加pyserial作为依赖
    ],
    zip_safe=True,
    maintainer='feiyu',
    maintainer_email='Mr.zhu345@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_driver_node = imu_driver_py.imu_driver_node:main'
        ],
    },
)
