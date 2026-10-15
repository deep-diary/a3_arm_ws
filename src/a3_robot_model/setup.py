from setuptools import setup
import os
from glob import glob

package_name = "a3_robot_model"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a3_arm",
    maintainer_email="dev@example.com",
    description="系统级机器人模型配置：默认描述包与配套档位文件（F174）",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "a3_robot_model = a3_robot_model.robot_model:_cli",
        ],
    },
    tests_require=['ament_lint_auto', 'ament_flake8', 'ament_pep257', 'pytest']
)
