from glob import glob
import os

from setuptools import setup

package_name = "a3_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a3_arm",
    maintainer_email="dev@example.com",
    description="A3 视觉感知：人脸检测与视觉伺服跟踪（F183）",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "face_detect_node = a3_perception.face_detect_node:main",
            "face_track_controller = a3_perception.face_track_controller:main",
        ],
    },
    tests_require=["pytest"],
)
