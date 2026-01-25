from setuptools import find_packages, setup
from glob import glob
import os

package_name = "hw2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.png")),
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
            "waypoint_publisher = hw2.waypoint_publisher:main",
            "waypoint_follower = hw2.waypoint_follower:main",
            "map_publisher = hw2.map_publisher:main",
            "vfh_follower = hw2.vfh_follower:main",
        ],
    },
)
