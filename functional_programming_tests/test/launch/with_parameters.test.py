# -*- coding: utf-8 -*-
import launch
from launch_ros.actions import Node
import launch_testing
import unittest


def generate_test_description():

    with_parameters_test = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "with_parameters",
            ]
        ),
        output="screen",
        emulate_tty=True,
        parameters=[
            {"robot_size": 1},
        ],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            with_parameters_test,
            # In tests where all of the procs under tests terminate themselves, it's necessary
            # to add a dummy process not under test to keep the launch alive. launch_test
            # provides a simple launch action that does this:
            # https://github.com/ros2/launch/blob/aed025e04e0b143362c69bf29a67b2ffff4c59ee/launch_testing/test/launch_testing/examples/args_launch_test.py#L63
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "with_parameters_test": with_parameters_test,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, with_parameters_test):
        self.proc_info.assertWaitForShutdown(with_parameters_test, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes
    def test_gtest_pass(self, proc_info, with_parameters_test):
        launch_testing.asserts.assertExitCodes(proc_info, process=with_parameters_test)
