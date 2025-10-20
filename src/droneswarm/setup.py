from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'droneswarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')), # Include all launch files.
        #(os.path.join('lib', package_name), glob('our_data/*')), # not sure this is the "correct" place to put it, but its the easiest to access from code
        (os.path.join('share', package_name, 'our_data'), glob('our_data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niels',
    maintainer_email='fynniels12@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'our_offboard_control = droneswarm.offboard_control_example:main',
            'px4_controller = droneswarm.px4_controller:main',
        ],
    },
)
