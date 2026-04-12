from pathlib import Path

from setuptools import find_packages, setup

package_name = "lunabot_navigation"
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
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/config", _relative_matches("config/*.yaml")),
        (
            f"share/{package_name}/behavior_trees",
            _relative_matches("behavior_trees/*.xml"),
        ),
        (
            f"share/{package_name}/maps",
            _relative_matches("maps/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="drkwonk",
    maintainer_email="drkwonk@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
