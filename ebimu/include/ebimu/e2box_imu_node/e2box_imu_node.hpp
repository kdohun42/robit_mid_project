/*
 * Copyright 2024 Myeong Jin Lee
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef E2BOX_IMU_NODE_HPP
#define E2BOX_IMU_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_manager.hpp"

#include "humanoid_interfaces/msg/imu_msg.hpp"
#include <sensor_msgs/msg/imu.hpp>
// #include "vision_interfaces/msg/pan_angle_compensation.hpp"

// extern double roll, pitch, yaw;

namespace e2box_imu
{

class E2BoxIMUNode : public rclcpp::Node
{
  public:
    E2BoxIMUNode();
    ~E2BoxIMUNode();

    std::string port_name;
    int baudrate;

    double angular_velocity_threshold;
    double linear_acceleration_threshold;

    rclcpp::Publisher<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_Pub;
    rclcpp::Subscription<vision_interfaces::msg::PanAngleCompensation>::SharedPtr yaw_set_sub_;
    void yawSetVisionCallback(const vision_interfaces::msg::PanAngleCompensation::SharedPtr msg);
    double yaw_offset_ = 0.0;
    bool reset_desire_yaw_pending_ = false;
    double raw_yaw_deg_ = 0.0;
    double roll_offset_ = 0.0;
    double raw_roll_deg_ = 0.0;
    double pitch_offset_ = 0.0;
    double raw_pitch_deg_ = 0.0;
    void onReceiveImu();
    void publishIMUData();
    void publishEulerData();

  private:
    SerialManager serial_manager_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_shutting_down_;

    void timerCallback();

    bool data_acquisision;

    bool m_boolUpdateDataFlag;
    void setStatusUpdateData(bool on_off)
    {
        m_boolUpdateDataFlag = on_off;
    }
    bool getStatusUpdateData()
    {
        return m_boolUpdateDataFlag;
    }

    unsigned int m_dwordCounterCheckSumPass;
    unsigned int m_dwordCounterCheckSumFail;

    bool m_boolHeaderDetectFlag;
    void setStatusHeaderDetect(bool on_off)
    {
        m_boolHeaderDetectFlag = on_off;
    }
    bool getStatusHeaderDetect()
    {
        return m_boolHeaderDetectFlag;
    }

    int m_iRawDataIndex;

    bool handleRawIMUData();

    double m_dQuaternion[4];
    double m_dAccel[3];
    double m_dAngleRate[3];

    byte m_abyteRawData[90];
    char m_acCopiedRawData[90];

    unsigned int i_checksum;
    unsigned int i_candisum;

    void initialize();

    void interpretGeneral();
    bool calculateChecksum();
    void extractData(byte byte_data);

    DWORD checkByteError(byte byte_cpr1, byte byte_cpr2);

    void setCallback();

    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::Imu imu_msg_temp_;
    sensor_msgs::msg::Imu imu_msg_prev_;

    double gap_ang_vel_x = 0.0;
    double gap_ang_vel_y = 0.0;
    double gap_ang_vel_z = 0.0;
    double gap_acc_x = 0.0;
    double gap_acc_y = 0.0;
    double gap_acc_z = 0.0;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;

    bool use_complementary_filter_ = false;
    double cf_alpha_ = 0.95;
    double cf_yaw_ = 0.0;
    bool cf_initialized_ = false;
    rclcpp::Time cf_prev_time_;

    double applyComplementaryFilter(double raw_yaw_deg, double gyro_z_rps);

    bool use_low_pass_filter_ = false;
    double lpf_alpha_ = 0.3;
    bool lpf_yaw_initialized_ = false;
    double lpf_yaw_ = 0.0;

    double applyLowPassFilter(double current, double previous);
};

} // namespace e2box_imu

#endif // E2BOX_IMU_NODE_HPP
