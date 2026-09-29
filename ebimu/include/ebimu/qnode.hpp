/**
 * @file /include/ebimu/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ebimu_QNODE_HPP_
#define ebimu_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <thread>
#include <memory>
#include <string>

#include "serial_manager.hpp"
#include "e2box_imu_node.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

namespace e2box_imu
{
  /*****************************************************************************
  ** Class
  *****************************************************************************/

  class QNode : public QThread
  {
    Q_OBJECT
  public:
    QNode(int argc, char** argv);
    ~QNode();
    int imu_flag_;
    void clearFlag();
    void publishSetYaw(double target_yaw_deg);

  protected:
    void run();

  private:
    std::shared_ptr<e2box_imu::E2BoxIMUNode> node;
    std::shared_ptr<rclcpp::Node> qnode;

    void imu_callback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
    std::shared_ptr<rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>> Imu_Sub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> set_yaw_pub_;

  public:
    void setYawOffset(double target_yaw_deg)
    {
      node->yaw_offset_ = target_yaw_deg - node->raw_yaw_deg_;
      node->reset_desire_yaw_pending_ = true;
    }
    void setRollOffset(double target_roll_deg) { node->roll_offset_ = target_roll_deg - node->raw_roll_deg_; }
    void setPitchOffset(double target_pitch_deg) { node->pitch_offset_ = target_pitch_deg - node->raw_pitch_deg_; }
    double getRawYawDeg() { return node->raw_yaw_deg_; }

    double past_yaw;
    double past_roll;
    double past_pitch;

    double roll_;
    double pitch_;
    double yaw_;

    double abs_roll_;
    double abs_pitch_;
    double abs_yaw_;

  private:
  Q_SIGNALS:
    void dial_update(double roll, double pitch, double yaw);
    void rosShutDown();
    void dial_callback();
  };
} // namespace ebimu

#endif /* ebimu_QNODE_HPP_ */
