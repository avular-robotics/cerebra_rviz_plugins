import launch
from launch.substitutions import PathJoinSubstitution
from ament_index_python import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory('cerebra_rviz_plugins')

    rviz_config_file_path = PathJoinSubstitution([pkg_dir, 'launch', 'cerebra.rviz'])

    return launch.LaunchDescription(
        [
            launch.actions.ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config_file_path]
            )
        ]
    )
