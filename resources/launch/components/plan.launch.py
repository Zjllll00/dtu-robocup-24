from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from raubase_ros.config import get_top_namespace

# =============================================================================
#                            Behavior Plan Launcher
#  This launch file let users launch behaviors plan with all its tasks.
# =============================================================================


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dtu_robocup_24",
                executable="plan",
                emulate_tty=True,
            )
        ],
    )
