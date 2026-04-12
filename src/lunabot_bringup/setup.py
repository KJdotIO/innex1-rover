"""Setup script for the lunabot_bringup package."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "lunabot_bringup"
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
                for path in package_root.joinpath("launch").glob("*.launch.py")
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                str(path.relative_to(package_root))
                for path in package_root.joinpath("config").glob("*.yaml")
            ],
        ),
        (
            f"share/{package_name}/rviz",
            [
                str(path.relative_to(package_root))
                for path in package_root.joinpath("rviz").glob("*.rviz")
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="ko129@student.le.ac.uk",
    description="System bringup launch files and checks for Lunabot",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "navigate_to_pose_gate = lunabot_bringup.navigate_to_pose_gate:main",
            "mission_dry_run = lunabot_bringup.mission_dry_run:main",
            "mission_manager = lunabot_bringup.mission_manager:main",
            "preflight_check = lunabot_bringup.preflight_check:main",
        ],
    },
)
