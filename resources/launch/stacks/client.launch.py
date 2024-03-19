from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction,
    DeclareLaunchArgument,
    LogInfo,
)
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from raubase_ros.config import get_top_namespace
from dtu_robocup_24 import ROBOT_NAME

# =============================================================================
#                         Behaviour stack Launcher
#  This launch file will launch all the nodes needed for the behaviour plans.
# =============================================================================


def generate_launch_description():
    debug_setup = LaunchConfiguration("debug_setup") == "robot"
    decl = DeclareLaunchArgument("setup", default_value="robot")

    return LaunchDescription(
        [
            GroupAction(
                [
                    decl,
                    LogInfo(
                        condition=LaunchConfigurationEquals("setup", "robot"),
                        msg=[f"Running on local server"],
                    ),
                    LogInfo(
                        condition=LaunchConfigurationNotEquals("setup", "robot"),
                        msg=[f"Running on robot server"],
                    ),
                    PushRosNamespace(
                        ROBOT_NAME if debug_setup else get_top_namespace()
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                FindPackageShare("dtu_robocup_24"),
                                "/launch",
                                "/components",
                                "/plan.launch.py",
                            ],
                        ),
                    ),
                ]
            )
        ]
    )
