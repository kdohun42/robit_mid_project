/**
 * @file /include/gamecontroller/fanout/gc_fanout_sim.hpp
 *
 * @brief GameController 출력 하나를 로봇 N대로 나눠 재발행하는 노드.
 **/

#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gamecontroller/msg/gamecontroldata.hpp"

class GcFanoutSim : public rclcpp::Node
{
public:
  GcFanoutSim();

private:
  void gamecontrolCallback(
      const gamecontroller::msg::Gamecontroldata::SharedPtr msg);

  std::vector<std::string> robot_names;
  std::vector<int> robot_numbers;
  std::vector<int> robot_positions;
  std::vector<
      rclcpp::Publisher<gamecontroller::msg::Gamecontroldata>::SharedPtr>
      publishers;
  rclcpp::Subscription<gamecontroller::msg::Gamecontroldata>::SharedPtr
      subscription;
};
