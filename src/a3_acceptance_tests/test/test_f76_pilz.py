"""
launch_testing 封装 F76：Pilz 工业运动规划器验收（PTP/LIN/CIRC + Sequence）.

被测栈：a3_bringup.launch.py hardware:=mock use_mqtt:=false（domain 62）。
验收脚本：scripts/a3_test/f76_pilz_acceptance.py（单一事实源，12 项）。
"""

import os
import sys
import unittest

import launch
import launch_testing
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

DOMAIN = "62"


def generate_test_description():
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("a3_bringup"),
                "launch",
                "a3_bringup.launch.py",
            )
        ),
        launch_arguments={
            "hardware": "mock",
            "use_rviz": "false",
            "use_mqtt": "false",
        }.items(),
    )
    harness = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(
                get_package_share_directory("a3_acceptance_tests"),
                "harness",
                "f76_pilz_acceptance.py",
            ),
        ],
        name="f76_harness",
        output="screen",
    )
    return launch.LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", DOMAIN),
        SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
        stack,
        TimerAction(period=1.0, actions=[harness]),
        launch_testing.actions.ReadyToTest(),
    ]), {"stack": stack, "harness": harness}


class TestF76Acceptance(unittest.TestCase):
    def test_harness_passes(self, proc_info, harness):
        proc_info.assertWaitForShutdown(harness, timeout=240)
        launch_testing.asserts.assertExitCodes(proc_info, process=harness)
