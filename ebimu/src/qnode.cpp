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
#include "std_msgs/msg/string.hpp"
#include "../include/ebimu/qnode.hpp"

extern rclcpp::Publisher<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_publisher_;

namespace e2box_imu {

QNode::QNode(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  qnode = rclcpp::Node::make_shared("ebimu_qnode");
  node = std::make_shared<e2box_imu::E2BoxIMUNode>();

  node->declare_parameter<std::string>("imu_bridge_sub_topic", "bridge");
  node->declare_parameter<std::string>("imu_pub_topic", "Imu");
  node->declare_parameter<std::string>("imu_set_yaw_pub_topic", "imu_set_yaw");
  std::string imu_bridge_sub_topic = node->get_parameter("imu_bridge_sub_topic").as_string();
  std::string imu_pub_topic = node->get_parameter("imu_pub_topic").as_string();
  std::string imu_set_yaw_pub_topic = node->get_parameter("imu_set_yaw_pub_topic").as_string();

  Imu_Sub = node->create_subscription<humanoid_interfaces::msg::ImuMsg>(
    imu_bridge_sub_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&QNode::imu_callback, this, std::placeholders::_1));
  imu_publisher_ = node->create_publisher<humanoid_interfaces::msg::ImuMsg>(
    imu_pub_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  set_yaw_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    imu_set_yaw_pub_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}
void QNode::imu_callback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  Q_EMIT dial_update(msg->roll, msg->pitch, msg->yaw);
}


void QNode::clearFlag()
{
  imu_flag_ = -1;
}

void QNode::publishSetYaw(double target_yaw_deg)
{
  std_msgs::msg::Float64 msg;
  msg.data = target_yaw_deg;
  set_yaw_pub_->publish(msg);
}

void QNode::run()
{
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(qnode);
  executor.spin();
  Q_EMIT rosShutDown();
}

}
