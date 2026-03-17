from setuptools import find_packages, setup
from glob import glob
import os

package_name = "hw5_nav2"

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
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Magraz",
    maintainer_email="agrazvallejo@live.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "my_node = hw5_nav2.my_node:main",
            "go_to_goal = hw5_nav2.go_to_goal:main",
            "visit_rooms = hw5_nav2.visit_rooms:main",
        ],
    },
)
