import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smarter_bag'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.config.rviz')),
        (os.path.join('share', package_name, 'data'), glob('data/data.log')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Persson',
    maintainer_email='andreas.persson@oru.se',
    description='The smarter_bag package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logfile_parser = smarter_bag.logfile_parser:main',
        ],
    },
)
