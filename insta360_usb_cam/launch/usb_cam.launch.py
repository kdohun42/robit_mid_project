from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('insta360_usb_cam'),
        'config',
        'camera_config.yaml'
    )

    info_dir = os.path.join(
        get_package_share_directory('insta360_usb_cam'),
        'config',
        'camera_info_config.yaml'
    )

    # YAML 설정 파일 로드
    with open(config_dir, 'r') as file:
        config_params = yaml.safe_load(file)
        camera_name = config_params['/**']['ros__parameters']['camera_name']

    node_name = f'pan_tilt_camera_node_{camera_name}'

    pan_tilt_camera_node = Node(
        package='insta360_usb_cam',
        executable='pan_tilt_camera_node',
        name=node_name,
        output='screen',
        parameters=[config_dir, info_dir]
    )

    return LaunchDescription([
        pan_tilt_camera_node
    ])
