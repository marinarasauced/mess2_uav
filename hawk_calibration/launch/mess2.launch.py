
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """
    
    """
    agent_name = LaunchConfiguration("agent_name")
    agent_name_ = DeclareLaunchArgument(
        "agent_name",
        default_value="agent",
        description="name of the agent"
    )

    num_measurements = LaunchConfiguration("num_measurements")
    num_measurements_ = DeclareLaunchArgument(
        "num_measurements",
        default_value="1000",
        description="number of vicon measurements to collect at each pose"
    )

    server_node = Node(
        package="hawk_calibration",
        executable="server",
        name="calibration_server",
        namespace=[TextSubstitution(text="/uav/"), agent_name],
        parameters=[{
            "agent_name": agent_name
        }]
    )

    client_node = Node(
        package="hawk_calibration",
        executable="client",
        name="calibration_client",
        namespace=[TextSubstitution(text="/uav/"), agent_name],
        parameters=[{
            "agent_name": agent_name,
            "num_measurements": num_measurements
        }]
    )

    ld = LaunchDescription()
    ld.add_action(agent_name_)
    ld.add_action(num_measurements_)
    ld.add_action(server_node)
    ld.add_action(client_node)
    return ld
