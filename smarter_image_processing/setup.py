import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smarter_image_processing'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Persson',
    maintainer_email='andreas.persson@oru.se',
    description='The smarter_image_processing package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gray_scale_converter = smarter_image_processing.gray_scale_converter:main'
        ],
    },
)
