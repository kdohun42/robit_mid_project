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
#include <string>
#include <sstream>

#include "std_msgs/msg/string.hpp"
#include "../include/tune_walk/qnode.hpp"

namespace tune_walk
{

  double pitch, roll, yaw;
  double left_x_zmp, left_y_zmp, right_x_zmp, right_y_zmp, total_x_zmp, total_y_zmp;
  bool Left_Foot, Right_Foot, Both_Feet;

  QNode::QNode()
  {
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("tune_walk");

    tune2walk_Pub = node->create_publisher<humanoid_interfaces::msg::Tune2IkMsg>("tune2walk", 10);
    imuSub = node->create_subscription<humanoid_interfaces::msg::ImuMsg>(
        "Imu",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(),
        std::bind(&QNode::imuCallback, this, std::placeholders::_1));
    zmpSub = node->create_subscription<humanoid_interfaces::msg::ZmpMsg>(
        "zmp",
        10,
        std::bind(&QNode::zmpCallback, this, std::placeholders::_1));
    LandingSub = node->create_subscription<humanoid_interfaces::msg::IkLTCMsg>(
        "Landing_Time_Control",
        10,
        std::bind(&QNode::LandingCallback, this, std::placeholders::_1));


    this->start();
  }

  void QNode::LandingCallback(const humanoid_interfaces::msg::IkLTCMsg::SharedPtr msg)
  {
    Landing_info.entire_time = msg->entire_time;
    Landing_info.swing_gain_l = msg->swing_gain_l;
    Landing_info.swing_gain_r = msg->swing_gain_r;
    Landing_info.warning = msg->warning;
    Landing_info.safe = msg->safe;

    Landing_info.landing_time_l = msg->landing_time_l;
    Landing_info.landing_time_r = msg->landing_time_r;
    Landing_info.landing_error_l = msg->landing_error_l;
    Landing_info.landing_error_r = msg->landing_error_r;

    Q_EMIT Landing_callback();
  }

  void QNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
  {
    pitch = msg->pitch;
    roll = msg->roll;
    yaw = msg->yaw;
  }

  void QNode::zmpCallback(const humanoid_interfaces::msg::ZmpMsg::SharedPtr msg)
  {
    left_x_zmp = msg->left_x_zmp;
    left_y_zmp = msg->left_y_zmp;
    right_x_zmp = msg->right_x_zmp;
    right_y_zmp = msg->right_y_zmp;
    total_x_zmp = msg->total_x_zmp;
    total_y_zmp = msg->total_y_zmp;
    Left_Foot = msg->left_foot;
    Right_Foot = msg->right_foot;
    Both_Feet = msg->both_feet;
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
}
