from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'moraimpc'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=[
        'sensing','sensing.*','navigation','navigation.*',
        'parking','parking.*','visualization','visualization.*',
    ]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/moraimpc']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'morai_bridge_node    = sensing.morai_bridge_node:main',
            'eskf_node            = sensing.eskf_node:main',
            'parking_manager_node = parking.parking_manager_node:main',
            'path_follower_node   = navigation.path_follower_node:main',
            'path_maker_node      = navigation.path_maker_node:main',
            'iridescence_gui_node = visualization.iridescence_gui_node:main',
        ],
    },
)
