from setuptools import setup
import os
from glob import glob

package_name = "app_melodion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.json") + glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "scores"),
         glob("config/scores/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a3_arm",
    maintainer_email="dev@example.com",
    description="A3 arm melodion playing app",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "melodion_player = app_melodion.melodion_player_node:main",
            "melodion_ik_probe = app_melodion.ik_probe:main",
        ],
    },
)
