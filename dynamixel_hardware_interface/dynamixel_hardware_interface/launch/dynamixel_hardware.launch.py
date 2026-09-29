from lifecycle_msgs.msg import Transition

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory(
        "dynamixel_hardware_interface"
    )

    config_file = os.path.join(
        package_share,
        "config",
        "motor_settings.yaml"
    )

    dynamixel_node = LifecycleNode(
        package="dynamixel_hardware_interface",
        executable="dynamixel_hardware_interface_node",
        name="dynamixel_hardware_interface",
        namespace="",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        dynamixel_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=dynamixel_node,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=lambda action: action == dynamixel_node,
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=dynamixel_node,
                # configure 직후에만 activate (activate 실패 후 inactive 복귀 시 재시도 루프 방지)
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=lambda action: action == dynamixel_node,
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        )
    ])
