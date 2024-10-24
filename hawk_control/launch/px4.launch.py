
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Main launch file to include px4_launch with a custom namespace.
    """
    namespace = LaunchConfiguration("namespace")
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="custom_namespace",
        description="Custom namespace for px4_launch"
    )

    px4_launch_file = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [FindPackageShare('mavros'), '/launch/px4.launch']
        ),
        launch_arguments={
            'namespace': namespace,
            'fcu_url': '/dev/ttyAMA0:921600',
            'gcs_url': '',
            'tgt_system': '1',
            'tgt_component': '1',
            'log_output': 'screen',
            'fcu_protocol': 'v2.0',
            'respawn_mavros': 'false'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(px4_launch_file)

    return ld