from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config import ConfigFile
from raubase_ros.constants.namespaces import CAMERA_NAMESPACE

# =============================================================================
#                               Camera Launcher
#  This launch file let users launch the Camera Node configured with the file
#  at (~/config/camera.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile("processor", "dtu_robocup_24")
    return LaunchDescription(
        [
            Node(
                package="dtu_robocup_24",
                executable="image_processor",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
                namespace=CAMERA_NAMESPACE,
            )
        ],
    )
