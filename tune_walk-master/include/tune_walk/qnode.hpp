/**
 * @file /include/tune_walk/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tune_walk_QNODE_HPP_
#define tune_walk_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>

#include "humanoid_interfaces/msg/tune2_ik_msg.hpp"
#include "humanoid_interfaces/msg/imu_msg.hpp"
#include "humanoid_interfaces/msg/zmp_msg.hpp"
#include "humanoid_interfaces/msg/ik_pattern_msg.hpp"
#include "humanoid_interfaces/msg/ik_ltc_msg.hpp"

/*****************************************************************************
** Class
*****************************************************************************/

using namespace std;

namespace tune_walk{

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();
  rclcpp::Publisher<humanoid_interfaces::msg::Tune2IkMsg>::SharedPtr tune2walk_Pub;

  humanoid_interfaces::msg::IkLTCMsg Landing_info;

  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void zmpCallback(const humanoid_interfaces::msg::ZmpMsg::SharedPtr msg);
  void LandingCallback(const humanoid_interfaces::msg::IkLTCMsg::SharedPtr msg);

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>> imuSub;
  std::shared_ptr<rclcpp::Subscription<humanoid_interfaces::msg::ZmpMsg>> zmpSub;
  std::shared_ptr<rclcpp::Subscription<humanoid_interfaces::msg::IkLTCMsg>> LandingSub;
  

Q_SIGNALS:
  void rosShutDown();
  void Landing_callback();

};
}
#endif /* tune_walk_QNODE_HPP_ */
