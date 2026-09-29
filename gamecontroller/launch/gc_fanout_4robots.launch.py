"""GameController 출력을 robit 팀 4대로 나눈다 (시뮬레이션용).

  /gamecontroldata  ->  /robit_1/gamecontroldata  (robotnum 1)
                        /robit_2/gamecontroldata  (robotnum 2)
                        /robit_3/gamecontroldata  (robotnum 3)
                        /robit_4/gamecontroldata  (robotnum 4)

  ros2 launch gamecontroller gc_fanout_4robots.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


ROBIT_NAMES = ["robit_1", "robit_2", "robit_3", "robit_4"]


def generate_launch_description():

    robit_fanout = Node(
        package="gamecontroller",
        executable="gc_fanout_node",
        name="gc_fanout_robit",
        output="screen",
        parameters=[{
            "input_topic": "/gamecontroldata",
            "robot_names": ROBIT_NAMES,
        }],
    )

    return LaunchDescription([
        robit_fanout,
    ])
