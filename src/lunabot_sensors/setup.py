"""Setup script for the lunabot_sensors package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lunabot_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="lunabotics@le.ac.uk",
    description="Hardware sensor wrappers for Lunabot",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "camera_contract_adapter = lunabot_sensors.camera_contract_adapter:main",
        ],
    },
)
