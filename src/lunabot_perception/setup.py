"""Package setup for lunabot_perception."""

from setuptools import find_packages, setup

package_name = "lunabot_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="ko129@student.le.ac.uk",
    description="Perception nodes for crater and hazard extraction",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "hazard_detection = lunabot_perception.hazard_detection:main",
        ]
    },
)
