"""Package configuration for lunabot_excavation."""

from setuptools import find_packages, setup

package_name = "lunabot_excavation"

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
            "share/" + package_name + "/launch",
            [
                "launch/excavation_controller.launch.py",
                "launch/excavation_action_server.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="ko129@student.le.ac.uk",
    description="Excavation subsystem controller and tooling",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "excavation_controller = lunabot_excavation.excavation_controller:main",
            "excavation_action_server = lunabot_excavation.excavation_action_server:main",
        ],
    },
)
