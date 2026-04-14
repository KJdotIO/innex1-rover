"""Package configuration for lunabot_drivetrain."""

from setuptools import find_packages, setup

package_name = "lunabot_drivetrain"

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
            "share/" + package_name + "/config",
            ["config/drivetrain.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="ko129@student.le.ac.uk",
    description=(
        "Drivetrain bridge for the INNEX-1 four-wheel skid-steer rover. "
        "Converts cmd_vel to Sabertooth Packetized Serial and publishes "
        "encoder-derived odometry and telemetry."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "drivetrain_bridge = lunabot_drivetrain.drivetrain_bridge:main",
        ],
    },
)
