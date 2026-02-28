from setuptools import find_packages, setup
from glob import glob
import os

package_name = "hw4_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config/rviz"), glob("config/rviz/*")),
        (os.path.join("share", package_name, "world"), glob("world/*.world")),
        (
            os.path.join("share", package_name, "world/include"),
            glob("world/include/*"),
        ),
        (
            os.path.join("share", package_name, "world/bitmaps"),
            glob("world/bitmaps/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Magraz",
    maintainer_email="agraz.manuel@outlook.com",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "scan_max_to_inf = hw4_localization.scan_max_to_inf:main",
            "goal_relay = hw4_localization.goal_relay:main",
            "target_waypoint_patrol = hw4_localization.target_waypoint_patrol:main",
            "chase_target = hw4_localization.chase_target:main",
            "mcl = hw4_localization.mcl:main",
        ],
    },
)
