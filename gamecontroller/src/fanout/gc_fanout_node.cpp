/**
 * @file /src/fanout/gc_fanout_node.cpp
 *
 * @brief gc_fanout_sim 노드 진입점
 **/

#include "gamecontroller/fanout/gc_fanout_sim.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GcFanoutSim>());
  rclcpp::shutdown();

  return 0;
}
