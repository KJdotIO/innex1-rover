import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lunabot_localisation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drkwonk',
    maintainer_email='drkwonk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'start_zone_localiser = '
            'lunabot_localisation.start_zone_localiser:main',
            'tag_pose_publisher = lunabot_localisation.tag_pose_publisher:main',
            'sim_camera_info_publisher = '
            'lunabot_localisation.sim_camera_info_publisher:main',
            'stereo_camera_info_publisher = '
            'lunabot_localisation.stereo_camera_info_publisher:main',
            'visual_odometry_gate = lunabot_localisation.visual_odometry_gate:main',
        ],
    },

)
