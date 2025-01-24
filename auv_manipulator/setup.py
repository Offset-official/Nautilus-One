from setuptools import find_packages, setup
import os
from glob import glob

package_name = "auv_manipulator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include any launch files or other resources if present
        # (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=[
        "setuptools",
        "adafruit-circuitpython-servokit",  # External Python dependency
    ],
    zip_safe=True,
    maintainer="abhinav",
    maintainer_email="this.abhinav26@gmail.com",
    description="everything manipulator",
    license="NIL",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "manip_controller = auv_manipulator.manip_controller:main",
        ],
    },
)
