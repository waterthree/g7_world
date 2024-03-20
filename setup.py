import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'g7_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'custom_world.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group 7',
    maintainer_email='waterthree@gmail.com',
    description='Group 7 custom world Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
     #       'walker = g7_world.CPSC_5207EL-02_A3_G7:main',
        ],
    },
)
