
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Main launch file to include px4_launch with a custom namespace.
    """
    agent_name = LaunchConfiguration("agent_name")
    agent_name_ = DeclareLaunchArgument(
        "agent_name",
        default_value="agent",
        description="name of the agent"
    )

    px4_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("hawk_control"), "/launch/px4.launch.py"]
        ),
        launch_arguments={
            "namespace": [TextSubstitution(text="/uav/"), agent_name]
        }.items()
    )

    offboard_node = Node(
        package="hawk_control",
        executable="offboard",
        name="offboard",
        namespace=[TextSubstitution(text="/uav/"), agent_name],
        parameters=[{
            "agent_name": agent_name
        }]
    )

    ld = LaunchDescription()
    ld.add_action(agent_name_)
    ld.add_action(px4_launch_file)
    ld.add_action(offboard_node)

    return ld