import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mr_goto'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Blue',
    maintainer_email='team.blue@tuwien.ac.at',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto = mr_goto.goto:main'
        ],
    },
)
