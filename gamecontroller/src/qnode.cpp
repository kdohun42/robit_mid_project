/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/gamecontroller/qnode.hpp"

QNode::QNode(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("gamecontroller");

  node->declare_parameter("pub_topic", std::string("gamecontroldata"));
  node->declare_parameter("TEAM_ROBIT_NUMBER", 4);
  node->declare_parameter("TEAM_ROBIT_RED", 22);
  node->declare_parameter("TEAM_ROBIT_BLUE", 23);
  node->declare_parameter("GAMECONTROLLER_DATA_PORT", 3838);
  node->declare_parameter("GAMECONTROLLER_RETURN_PORT", 3939);

  pubTopic = node->get_parameter("pub_topic").as_string();
  teamRobit = (int)node->get_parameter("TEAM_ROBIT_NUMBER").as_int();
  teamRobitRed = (int)node->get_parameter("TEAM_ROBIT_RED").as_int();
  teamRobitBlue = (int)node->get_parameter("TEAM_ROBIT_BLUE").as_int();
  dataPort = (int)node->get_parameter("GAMECONTROLLER_DATA_PORT").as_int();
  returnPort = (int)node->get_parameter("GAMECONTROLLER_RETURN_PORT").as_int();

  gamecontrollerPub = node->create_publisher<gamecontroller::msg::Gamecontroldata>(pubTopic, 10);

  this->start();
}


QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}
