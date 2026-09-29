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

#include "../../include/ebimu/e2box_imu_node/e2box_imu_node.hpp"

pthread_t thread_serial;
pthread_t tSerialThread;

rclcpp::Publisher<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_publisher_;

namespace e2box_imu
{

E2BoxIMUNode::E2BoxIMUNode() : Node("e2box_imu_node"), is_shutting_down_(false)
{
  RCLCPP_INFO(this->get_logger(), "E2BoxIMUNode Constructed.");

  this->declare_parameter<std::string>("settings.port_name", "/dev/ttyUSB-EBIMU");
  this->declare_parameter<int>("settings.baud_rate", 115200);
  this->declare_parameter<int>("settings.loop_rate", 100);
  this->declare_parameter<double>("settings.angular_velocity_threshold", 0.3);
  this->declare_parameter<double>("settings.linear_acceleration_threshold", 2.0);
  this->declare_parameter<bool>("settings.complementary_filter", false);
  this->declare_parameter<double>("settings.complementary_filter_alpha", 0.95);
  this->declare_parameter<bool>("settings.low_pass_filter", false);
  this->declare_parameter<double>("settings.low_pass_filter_alpha", 0.3);
  this->declare_parameter<std::string>("topics.imu_bridge_pub_topic", "bridge");
  this->declare_parameter<std::string>("topics.yaw_set_sub_topic", "/pan_angle_compensation");

  this->get_parameter("settings.port_name", port_name);
  this->get_parameter("settings.baud_rate", baudrate);
  int loop_rate;
  this->get_parameter("settings.loop_rate", loop_rate);
  this->get_parameter("settings.angular_velocity_threshold", angular_velocity_threshold);
  this->get_parameter("settings.linear_acceleration_threshold", linear_acceleration_threshold);
  this->get_parameter("settings.complementary_filter", use_complementary_filter_);
  this->get_parameter("settings.complementary_filter_alpha", cf_alpha_);
  this->get_parameter("settings.low_pass_filter", use_low_pass_filter_);
  this->get_parameter("settings.low_pass_filter_alpha", lpf_alpha_);
  RCLCPP_INFO(this->get_logger(), "[LPF] enabled=%d, alpha=%f", (int)use_low_pass_filter_, lpf_alpha_);
  std::string imu_bridge_pub_topic;
  this->get_parameter("topics.imu_bridge_pub_topic", imu_bridge_pub_topic);
  std::string yaw_set_sub_topic;
  this->get_parameter("topics.yaw_set_sub_topic", yaw_set_sub_topic);

  if (!serial_manager_.openSerial(port_name.c_str(), baudrate)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
    exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(this->get_logger(), "Serial port %s is opened.", port_name.c_str());
  RCLCPP_INFO(this->get_logger(), "Baudrate is set to %d.", baudrate);
  RCLCPP_INFO(this->get_logger(), "Loop rate is set to %d.", loop_rate);
  RCLCPP_INFO(
    this->get_logger(), "Angular velocity threshold is set to %f.", angular_velocity_threshold);
  RCLCPP_INFO(
    this->get_logger(), "Linear acceleration threshold is set to %f.",
    linear_acceleration_threshold);

  imu_Pub = this->create_publisher<humanoid_interfaces::msg::ImuMsg>(
    imu_bridge_pub_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

  yaw_set_sub_ = this->create_subscription<vision_interfaces::msg::PanAngleCompensation>(
    yaw_set_sub_topic, 5,
    std::bind(&E2BoxIMUNode::yawSetVisionCallback, this, std::placeholders::_1));

  auto period = std::chrono::milliseconds(1000 / loop_rate);
  timer_ = this->create_wall_timer(period, std::bind(&E2BoxIMUNode::timerCallback, this));

  initialize();

  setCallback();
}

E2BoxIMUNode::~E2BoxIMUNode()
{
  is_shutting_down_ = true;
  RCLCPP_INFO(this->get_logger(), "E2BoxIMUNode Destructed.");
}

void E2BoxIMUNode::initialize()
{
  m_dwordCounterCheckSumPass = 0;
  m_dwordCounterCheckSumFail = 0;

  memset(m_dQuaternion, 0, sizeof(m_dQuaternion));
  memset(m_dAngleRate, 0, sizeof(m_dAngleRate));
  memset(m_dAccel, 0, sizeof(m_dAccel));

  setStatusHeaderDetect(false);
  data_acquisision = false;
  setStatusUpdateData(false);

  m_iRawDataIndex = 0;
}

void E2BoxIMUNode::extractData(byte byte_data)
{
  if (!getStatusHeaderDetect()) {
    if (byte_data == 0x2A) {
      setStatusHeaderDetect(true);
      memset(m_abyteRawData, 0, sizeof(m_abyteRawData));
      m_abyteRawData[0] = byte_data;
      m_iRawDataIndex = 1;
    }
  }

  else {
    if (m_iRawDataIndex >= 89) {
      setStatusHeaderDetect(false);
      m_iRawDataIndex = 0;
      return;
    }
    m_abyteRawData[m_iRawDataIndex++] = byte_data;
    if (byte_data == 0x0D) {
      for (int i = 0; i < m_iRawDataIndex; i++) {
        m_acCopiedRawData[i] = m_abyteRawData[i];
      }
      data_acquisision = true;
      setStatusHeaderDetect(false);
    }
  }
}

void E2BoxIMUNode::interpretGeneral()
{
  char c_branch;
  sscanf(&m_acCopiedRawData[0], "%c", &c_branch);
  if (c_branch == '*') {
    sscanf(
      &m_acCopiedRawData[1], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &m_dQuaternion[0],
      &m_dQuaternion[1], &m_dQuaternion[2], &m_dQuaternion[3], &m_dAngleRate[0], &m_dAngleRate[1],
      &m_dAngleRate[2], &m_dAccel[0], &m_dAccel[1], &m_dAccel[2]);
  }

  setStatusUpdateData(true);
}

bool E2BoxIMUNode::handleRawIMUData()
{
  if (getStatusHeaderDetect() == false) {
    interpretGeneral();
    setStatusUpdateData(true);
    m_dwordCounterCheckSumPass++;
  } else {
    m_dwordCounterCheckSumFail++;
    return false;
  }
  return true;
}

bool E2BoxIMUNode::calculateChecksum()
{
  sscanf(&m_acCopiedRawData[m_iRawDataIndex - 3], "%x", &i_checksum);

  for (int i = 0; i < m_iRawDataIndex - 5; i++) {
    if (i == 1) {
      i_candisum = checkByteError(m_acCopiedRawData[i], m_acCopiedRawData[i + 1]);
    } else {
      i_candisum = checkByteError(i_candisum, m_acCopiedRawData[i]);
    }
  }

  if (i_checksum == i_candisum) {
    return true;
  }
  return false;
}

DWORD E2BoxIMUNode::checkByteError(byte byte_cpr1, byte byte_cpr2)
{
  byte byte_mask = 0x80;
  byte byte_result = 0;

  for (int i = 0; i < 8; i++) {
    byte_result <<= 1;

    if ((byte_cpr1 & byte_mask) == (byte_cpr2 & byte_mask)) {
      byte_result |= 0x0;
    } else {
      byte_result |= 0x1;
    }
    byte_mask >>= 1;
  }
  return byte_result;
}

void E2BoxIMUNode::timerCallback()
{
  onReceiveImu();
  publishIMUData();
  publishEulerData();
}

void E2BoxIMUNode::onReceiveImu()
{
  if (!serial_manager_.getStatusConnected() && !is_shutting_down_) {
    RCLCPP_ERROR(this->get_logger(), "Serial connection lost. Shutting down node.");
    rclcpp::shutdown();
    return;
  }

  int n = serial_manager_.getLength();
  byte * buffer = serial_manager_.getBuffer();

  if (n >= 10) {
    for (int i = 0; i < n; i++) {
      extractData(buffer[i]);
      if (data_acquisision) {
        serial_manager_.resetBuffer();
        handleRawIMUData();
        data_acquisision = false;
        break;
      }
    }
  }
}

void E2BoxIMUNode::publishIMUData()
{
  if (!data_acquisision) {
    imu_msg_.header.stamp = this->now();
    imu_msg_.header.frame_id = "imu_link";

    imu_msg_.orientation.x = m_dQuaternion[2];
    imu_msg_.orientation.y = m_dQuaternion[1];
    imu_msg_.orientation.z = m_dQuaternion[0];
    imu_msg_.orientation.w = m_dQuaternion[3];

    imu_msg_temp_.angular_velocity.x = m_dAngleRate[0] * M_PI / 180.0;
    imu_msg_temp_.angular_velocity.y = m_dAngleRate[1] * M_PI / 180.0;
    imu_msg_temp_.angular_velocity.z = m_dAngleRate[2] * M_PI / 180.0;

    imu_msg_temp_.linear_acceleration.x = m_dAccel[0] * 9.80665;
    imu_msg_temp_.linear_acceleration.y = m_dAccel[1] * 9.80665;
    imu_msg_temp_.linear_acceleration.z = m_dAccel[2] * 9.80665;

    gap_ang_vel_x = fabs(imu_msg_prev_.angular_velocity.x - imu_msg_temp_.angular_velocity.x) / 2.0;
    if (gap_ang_vel_x > angular_velocity_threshold) {
      imu_msg_temp_.angular_velocity.x = imu_msg_prev_.angular_velocity.x;
    }
    gap_ang_vel_y = fabs(imu_msg_prev_.angular_velocity.y - imu_msg_temp_.angular_velocity.y) / 2.0;
    if (gap_ang_vel_y > angular_velocity_threshold) {
      imu_msg_temp_.angular_velocity.y = imu_msg_prev_.angular_velocity.y;
    }
    gap_ang_vel_z = fabs(imu_msg_prev_.angular_velocity.z - imu_msg_temp_.angular_velocity.z) / 2.0;
    if (gap_ang_vel_z > angular_velocity_threshold) {
      imu_msg_temp_.angular_velocity.z = imu_msg_prev_.angular_velocity.z;
    }

    imu_msg_.angular_velocity.x =
      (imu_msg_temp_.angular_velocity.x + imu_msg_prev_.angular_velocity.x) / 2.0;
    imu_msg_.angular_velocity.y =
      (imu_msg_temp_.angular_velocity.y + imu_msg_prev_.angular_velocity.y) / 2.0;
    imu_msg_.angular_velocity.z =
      (imu_msg_temp_.angular_velocity.z + imu_msg_prev_.angular_velocity.z) / 2.0;

    gap_acc_x =
      fabs(imu_msg_prev_.linear_acceleration.x - imu_msg_temp_.linear_acceleration.x) / 2.0;
    if (gap_acc_x > linear_acceleration_threshold) {
      imu_msg_temp_.linear_acceleration.x = imu_msg_prev_.linear_acceleration.x;
    }
    gap_acc_y =
      fabs(imu_msg_prev_.linear_acceleration.y - imu_msg_temp_.linear_acceleration.y) / 2.0;
    if (gap_acc_y > linear_acceleration_threshold) {
      imu_msg_temp_.linear_acceleration.y = imu_msg_prev_.linear_acceleration.y;
    }
    gap_acc_z =
      fabs(imu_msg_prev_.linear_acceleration.z - imu_msg_temp_.linear_acceleration.z) / 2.0;
    if (gap_acc_z > linear_acceleration_threshold) {
      imu_msg_temp_.linear_acceleration.z = imu_msg_prev_.linear_acceleration.z;
    }

    imu_msg_.linear_acceleration.x =
      (imu_msg_temp_.linear_acceleration.x + imu_msg_prev_.linear_acceleration.x) / 2.0;
    imu_msg_.linear_acceleration.y =
      (imu_msg_temp_.linear_acceleration.y + imu_msg_prev_.linear_acceleration.y) / 2.0;
    imu_msg_.linear_acceleration.z =
      (imu_msg_temp_.linear_acceleration.z + imu_msg_prev_.linear_acceleration.z) / 2.0;

    imu_msg_prev_.angular_velocity.x = imu_msg_temp_.angular_velocity.x;
    imu_msg_prev_.angular_velocity.y = imu_msg_temp_.angular_velocity.y;
    imu_msg_prev_.angular_velocity.z = imu_msg_temp_.angular_velocity.z;

    imu_msg_prev_.linear_acceleration.x = imu_msg_temp_.linear_acceleration.x;
    imu_msg_prev_.linear_acceleration.y = imu_msg_temp_.linear_acceleration.y;
    imu_msg_prev_.linear_acceleration.z = imu_msg_temp_.linear_acceleration.z;

  }
}

double E2BoxIMUNode::applyComplementaryFilter(double raw_yaw_deg, double gyro_z_rps)
{
  rclcpp::Time now = this->now();

  if (!cf_initialized_) {
    cf_yaw_ = raw_yaw_deg;
    cf_prev_time_ = now;
    cf_initialized_ = true;
    return raw_yaw_deg;
  }

  double dt = (now - cf_prev_time_).seconds();
  cf_prev_time_ = now;

  if (dt <= 0.0 || dt > 1.0) {
    return cf_yaw_;
  }

  double gyro_z_dps = -gyro_z_rps * 180.0 / M_PI;
  double gyro_predicted = cf_yaw_ + gyro_z_dps * dt;

  double angle_diff = raw_yaw_deg - gyro_predicted;
  while (angle_diff > 180.0) angle_diff -= 360.0;
  while (angle_diff < -180.0) angle_diff += 360.0;

  cf_yaw_ = gyro_predicted + (1.0 - cf_alpha_) * angle_diff;

  return cf_yaw_;
}

double E2BoxIMUNode::applyLowPassFilter(double current, double previous)
{
  double diff = current - previous;
  while (diff >  180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return previous + lpf_alpha_ * diff;
}

void E2BoxIMUNode::yawSetVisionCallback(const vision_interfaces::msg::PanAngleCompensation::SharedPtr msg)
{
  // yaw_offset_ = msg->target_yaw_deg - raw_yaw_deg_;
}

void E2BoxIMUNode::publishEulerData()
{
  double t0 = 0.0, t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  x = m_dQuaternion[2];
  y = m_dQuaternion[1];
  z = m_dQuaternion[0];
  w = m_dQuaternion[3];

  t0 = 2.0 * (w * x + y * z);
  t1 = 1.0 - 2.0 * (x * x + y * y);
  roll = atan2(t0, t1);

  t2 = 2.0 * (w * y - z * x);
  if (t2 > 1.0) t2 = 1.0;
  else if (t2 < -1.0) t2 = -1.0;
  pitch = asin(t2);

  t3 = 2.0 * (w * z + x * y);
  t4 = 1.0 - 2.0 * (y * y + z * z);
  yaw = atan2(t3, t4);

  raw_yaw_deg_   = -1.0 * (yaw   * 180) / M_PI;
  raw_roll_deg_  =  1.0 * (pitch * 180) / M_PI;
  raw_pitch_deg_ =  1.0 * (roll  * 180) / M_PI;

  double filtered_yaw_deg = raw_yaw_deg_;
  if (use_complementary_filter_) {
    filtered_yaw_deg = applyComplementaryFilter(raw_yaw_deg_, imu_msg_.angular_velocity.z);
  }

  if (use_low_pass_filter_) {
    if (!lpf_yaw_initialized_) {
      lpf_yaw_ = filtered_yaw_deg;
      lpf_yaw_initialized_ = true;
    } else {
      lpf_yaw_ = applyLowPassFilter(filtered_yaw_deg, lpf_yaw_);
      while (lpf_yaw_ >  180.0) lpf_yaw_ -= 360.0;
      while (lpf_yaw_ < -180.0) lpf_yaw_ += 360.0;
    }
    filtered_yaw_deg = lpf_yaw_;
  }

  humanoid_interfaces::msg::ImuMsg imu_data;
  imu_data.roll  = raw_roll_deg_  + roll_offset_;
  double pitch_val = raw_pitch_deg_ + pitch_offset_;
  if (pitch_val > 180.0) pitch_val -= 360.0;
  else if (pitch_val < -180.0) pitch_val += 360.0;
  imu_data.pitch = pitch_val;
  imu_data.yaw   = filtered_yaw_deg + yaw_offset_;
  imu_data.reset_desire_yaw = reset_desire_yaw_pending_;

  imu_Pub->publish(imu_data);
  reset_desire_yaw_pending_ = false;

}

void E2BoxIMUNode::setCallback()
{
  serial_manager_.setCallback(
    [](void * arg) {
      auto node = static_cast<E2BoxIMUNode *>(arg);
      if (!node->serial_manager_.getStatusConnected() && !node->is_shutting_down_) {
        RCLCPP_ERROR(node->get_logger(), "Serial connection lost. Shutting down node.");
        rclcpp::shutdown();
      }
    },
    this);
}

}  // namespace e2box_imu

#ifdef BUILD_STANDALONE
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<e2box_imu::E2BoxIMUNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
