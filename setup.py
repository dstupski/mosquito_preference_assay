import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mosquito_preference_assay"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "experiments"), glob("experiments/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David Stupski",
    maintainer_email="dstupski@uw.edu",
    description="Two-choice mosquito visual preference assay (py5) with a ROS 2 "
                "node publishing a full JSON description of the left/right stimuli.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stimulus_publisher = mosquito_preference_assay.stimulus_publisher_node:main",
            "test_trigger = mosquito_preference_assay.test_trigger_node:main",
            "mosquito_detector = mosquito_preference_assay.mosquito_detector_node:main",
            "video_publisher = mosquito_preference_assay.video_publisher_node:main",
            "assay = mosquito_preference_assay.assay:main",
        ],
    },
)
