/**
 * @file /src/fanout/gc_fanout_sim.cpp
 *
 * @brief gc_fanout_sim.hpp 구현

 **/

#include "gamecontroller/fanout/gc_fanout_sim.hpp"

#include <cctype>
#include <cstdlib>

namespace
{

/// main_window.hpp 의 POSITION_GK 와 같은 값. 0 은 콤보박스의 "SET POSITION",
/// 즉 아직 정하지 않았다는 뜻이라 필드 플레이어로 통한다.
constexpr int POSITION_GOALKEEPER = 1;
constexpr int POSITION_FIELD = 0;

/// "robit_2" -> 2. 마지막 '_' 뒤를 번호로 본다.
/// 번호를 못 뽑으면 fallback(선언 순서 + 1)을 쓴다.
int robotNumberFromName(const std::string &name, int fallback)
{
  const std::size_t pos = name.rfind('_');
  if (pos == std::string::npos || pos + 1 >= name.size())
  {
    return fallback;
  }

  const std::string suffix = name.substr(pos + 1);
  for (std::size_t i = 0; i < suffix.size(); ++i)
  {
    if (!std::isdigit(static_cast<unsigned char>(suffix[i])))
    {
      return fallback;
    }
  }

  return std::atoi(suffix.c_str());
}

}  // namespace

GcFanoutSim::GcFanoutSim() : rclcpp::Node("gc_fanout")
{
  declare_parameter("input_topic", std::string("gamecontroldata"));
  declare_parameter("output_topic", std::string("gamecontroldata"));

  std::vector<std::string> default_names;
  default_names.push_back("robit_1");
  default_names.push_back("robit_2");
  default_names.push_back("robit_3");
  default_names.push_back("robit_4");
  declare_parameter("robot_names", default_names);

  const std::string input_topic = get_parameter("input_topic").as_string();
  const std::string output_topic = get_parameter("output_topic").as_string();
  robot_names = get_parameter("robot_names").as_string_array();

  if (robot_names.empty())
  {
    RCLCPP_ERROR(
        get_logger(),
        "robot_names 가 비어 있다. 나눠줄 대상이 없어 아무것도 발행하지 않는다.");
  }

  // position 은 로봇마다 달라야 한다. 실기는 로봇마다 자기 GameController
  // 설정을 갖지만 시뮬은 한 창이 팀 전체를 담당해서, 그대로 흘리면 네 대가
  // 같은 값을 받는다 -- 콤보박스가 GOALKEEPER 면 전원이 골키퍼가 되고,
  // 기본값(SET POSITION=0)이면 지정 골키퍼가 아예 없어서 골키퍼 자리의
  // 로봇도 필드 플레이어로 움직인다.
  //
  // 기본은 첫 로봇만 골키퍼. 다르게 두려면 robot_names 와 같은 길이로:
  //   -p robot_positions:="[0, 1, 0, 0]"
  std::vector<int64_t> default_positions(robot_names.size(), POSITION_FIELD);
  if (!default_positions.empty())
  {
    default_positions[0] = POSITION_GOALKEEPER;
  }
  declare_parameter("robot_positions", default_positions);

  std::vector<int64_t> positions =
      get_parameter("robot_positions").as_integer_array();
  if (positions.size() != robot_names.size())
  {
    RCLCPP_ERROR(
        get_logger(),
        "robot_positions 가 %zu 개인데 robot_names 는 %zu 개다. "
        "첫 로봇만 골키퍼로 두는 기본값을 쓴다.",
        positions.size(), robot_names.size());
    positions = default_positions;
  }

  for (std::size_t i = 0; i < robot_names.size(); ++i)
  {
    const std::string &name = robot_names[i];
    const int robot_number =
        robotNumberFromName(name, static_cast<int>(i) + 1);

    robot_numbers.push_back(robot_number);
    robot_positions.push_back(static_cast<int>(positions[i]));
    publishers.push_back(
        create_publisher<gamecontroller::msg::Gamecontroldata>(
            name + "/" + output_topic, 10));

    RCLCPP_INFO(
        get_logger(), "  %s/%s  <- robotnum %d, position %d%s",
        name.c_str(), output_topic.c_str(), robot_number,
        robot_positions.back(),
        robot_positions.back() == POSITION_GOALKEEPER ? " (goalkeeper)" : "");
  }

  subscription = create_subscription<gamecontroller::msg::Gamecontroldata>(
      input_topic, 10,
      std::bind(&GcFanoutSim::gamecontrolCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(
      get_logger(), "'%s' 를 로봇 %zu 대로 나눈다.",
      input_topic.c_str(), robot_names.size());
}

void GcFanoutSim::gamecontrolCallback(
    const gamecontroller::msg::Gamecontroldata::SharedPtr msg)
{
  for (std::size_t i = 0; i < publishers.size(); ++i)
  {
    // robotnum 과 position 만 로봇별로 갈아끼우고 나머지는 그대로 흘린다.
    // penalty 는 GC 가 이미 자기 playerNum 기준으로 채워 보낸 값이라
    // 전 로봇이 같은 값을 받는다 — 로봇별 페널티가 필요하면 여기다.
    gamecontroller::msg::Gamecontroldata out = *msg;
    out.robotnum = robot_numbers[i];
    out.position = robot_positions[i];

    publishers[i]->publish(out);
  }
}
