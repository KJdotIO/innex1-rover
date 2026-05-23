from pathlib import Path

from setuptools import find_packages, setup

package_name = "lunabot_safety"
package_root = Path(__file__).resolve().parent

setup(
    name=package_name,
    version="0.0.1",
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
                str(path.relative_to(package_root))
                for path in package_root.joinpath("launch").glob("*.launch.py")
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="INNEX1 Team",
    maintainer_email="todo@example.com",
    description="E-stop and motion-inhibit safety node for the INNEX1 rover.",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "estop_node = lunabot_safety.estop_node:main",
            "physical_estop_input = lunabot_safety.physical_estop_input:main",
        ],
    },
)
