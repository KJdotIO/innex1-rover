from pathlib import Path

from setuptools import find_packages, setup

package_name = "lunabot_localisation"
package_root = Path(__file__).resolve().parent

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                str(path.relative_to(package_root))
                for path in package_root.joinpath("launch").glob("*launch.[pxy][yma]*")
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                str(path.relative_to(package_root))
                for path in package_root.joinpath("config").glob("*.yaml")
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="lunabotics@le.ac.uk",
    description="Localisation launch, configuration, and AprilTag readiness nodes for Lunabot",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "camera_info_stamp_aligner = "
            "lunabot_localisation.camera_info_stamp_aligner:main",
            "start_zone_localiser = lunabot_localisation.start_zone_localiser:main",
            "tag_pose_publisher = lunabot_localisation.tag_pose_publisher:main",
        ],
    },
)
