from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from raubase_ros.config import get_top_namespace

# =============================================================================
#                         Processing stack Launcher
#  This launch file will launch all the processing units needed for the plans.
# =============================================================================


def generate_launch_description():
    return LaunchDescription(
        [
            GroupAction(
                [
                    PushRosNamespace(get_top_namespace()),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                FindPackageShare("dtu_robocup_24"),
                                "/launch",
                                "/components",
                                "/processor.launch.py",
                            ],
                        ),
                    ),
                ]
            )
        ]
    )
