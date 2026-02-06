from setuptools import find_packages, setup
from glob import glob
import os

package_name = "hw3"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Magraz",
    maintainer_email="agrazvallejo@live.com",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "waypoint_publisher = hw3.waypoint_publisher:main",
            "waypoint_follower = hw3.waypoint_follower:main",
            "map_publisher = hw3.map_publisher:main",
            "vfh_follower = hw3.vfh_follower:main",
            "mapper = hw3.mapper:main",
        ],
    },
)
