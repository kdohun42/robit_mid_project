from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('gamecontroller'),
        'config',
        'config.yaml'
    )
    
    gamecontroller = Node(
        package='gamecontroller',
        executable='gamecontroller_app',
        name='gamecontroller',
        output='screen',
        parameters=[config_dir],
    )

    return LaunchDescription([
        gamecontroller,
    ])