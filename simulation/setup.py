import os
from glob import glob
from setuptools import find_packages, setup

package_name = "simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dee",
    maintainer_email="k4aro0@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={},
    entry_points={
        "console_scripts": [],
    },
)
