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
        # This line installs every launch file in your 'launch/' folder
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # This line installs every yaml file in your 'config/' folder
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drkwonk',
    maintainer_email='drkwonk@todo.todo',
    description='Hard Readiness Gate for Localization',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tag_pose_publisher = lunabot_localisation.tag_pose_publisher:main',
            # Registered readiness_gate node
            'readiness_gate = lunabot_localisation.readiness_gate:main',
        ],
    },
)
