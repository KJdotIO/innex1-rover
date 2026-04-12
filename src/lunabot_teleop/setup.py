from pathlib import Path

from setuptools import find_packages, setup

package_name = "lunabot_teleop"
package_root = Path(__file__).resolve().parent


def _relative_matches(pattern: str) -> list[str]:
    """Return package-local files relative to the setup.py directory."""
    return sorted(
        str(path.relative_to(package_root)) for path in package_root.glob(pattern)
    )


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
        (
            f"share/{package_name}/launch",
            _relative_matches("launch/*.launch.py"),
        ),
        (
            f"share/{package_name}/config",
            _relative_matches("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="ko129@student.le.ac.uk",
    description="Joystick teleoperation launch files and configs",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
