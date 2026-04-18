"""Package setup for lunabot_perception."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "lunabot_perception"
package_root = Path(__file__).resolve().parent


def _relative_matches(pattern: str) -> list[str]:
    """Return package-local files relative to the setup.py directory."""
    return sorted(
        str(path.relative_to(package_root)) for path in package_root.glob(pattern)
    )


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
        (f"share/{package_name}/config", _relative_matches("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leicester Lunabotics Team",
    maintainer_email="lunabotics@le.ac.uk",
    description="Perception nodes for obstacle and terrain processing",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "crater_detection = lunabot_perception.crater_detection:main",
            "wall_exclusion_filter = lunabot_perception.wall_exclusion_filter:main",
        ],
    },
)
