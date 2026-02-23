from setuptools import find_packages, setup

package_name = "lunabot_control"

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
        ("share/" + package_name + "/launch", ["launch/material_actions.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="lunabotics@le.ac.uk",
    description="Control and mission bridge nodes for the rover",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "material_action_server = lunabot_control.material_action_server:main",
            "material_action_client = lunabot_control.material_action_client:main",
        ],
    },
)
