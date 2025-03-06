from setuptools import find_packages, setup
import os
from glob import glob

package_name = "auv_ml"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools", "onnxruntime", "opencv-python", "numpy", "sort-tracker"],
    zip_safe=True,
    maintainer="Subham Jalan",
    maintainer_email="subham.jalan@plaksha.edu.in",
    description="This package acts as the eyes of the AUV.",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],  # or other testing dependencies
    },
    entry_points={
        "console_scripts": [
            "yolo_inference_server_gate = auv_ml.yolo_inference_server_gate:main",
            # "yolo_inference_test_gate = auv_ml.yolo_inference_test_gate:main",
            "yolo_inference_server_bucket = auv_ml.yolo_inference_server_bucket:main",
            "uncomp = auv_ml.uncomp:main",
        
            ],

    },
)
