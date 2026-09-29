"""GameController 출력을 robit 4대 + opp 4대로 나눈다 (시뮬레이션용).

팀마다 gamecontroller_app 이 따로 뜨고(팀 번호가 다르므로), 팬아웃도 팀당
하나씩 붙는다. 상대팀 gamecontroller_app 은 pub_topic 을
'gamecontroldata_opp' 로 띄워야 한다.

  /gamecontroldata      ->  /robit_1..4/gamecontroldata  (robotnum 1..4)
  /gamecontroldata_opp  ->  /opp_1..4/gamecontroldata    (robotnum 1..4)

  ros2 launch gamecontroller gc_fanout_8robots.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


ROBIT_NAMES = ["robit_1", "robit_2", "robit_3", "robit_4"]
OPP_NAMES = ["opp_1", "opp_2", "opp_3", "opp_4"]


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

    opp_fanout = Node(
        package="gamecontroller",
        executable="gc_fanout_node",
        name="gc_fanout_opp",
        output="screen",
        parameters=[{
            "input_topic": "/gamecontroldata_opp",
            "robot_names": OPP_NAMES,
        }],
    )

    return LaunchDescription([
        robit_fanout,
        opp_fanout,
    ])
