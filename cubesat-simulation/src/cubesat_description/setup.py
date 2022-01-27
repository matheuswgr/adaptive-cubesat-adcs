from setuptools import setup
import os
from glob import glob

package_name = 'cubesat_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, ''), glob('spawn_cubesat.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matheuswagner',
    maintainer_email='matheuswagnr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
