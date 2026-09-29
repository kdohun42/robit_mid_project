#pragma once

#include <cstdint>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <dynamixel_hardware_msgs/msg/dynamixel_control_msgs.hpp>
#include <dynamixel_hardware_msgs/msg/dynamixel_msgs.hpp>
#include <dynamixel_hardware_msgs/msg/current_motor_status.hpp>

#include "dynamixel_hardware_interface/motor_config.hpp"
#include "dynamixel_hardware_interface/motor_model.hpp"
#include "dynamixel_hardware_interface/motor_setting.hpp"
#include "dynamixel_hardware_interface/motor_status.hpp"

namespace dynamixel_hardware_interface
{

    struct MotorDevice
    {
        MotorConfig config;

        uint16_t detected_model_number{0};
        MotorModel detected_model{MotorModel::UNKNOWN};
        const MotorModelInfo *model_info{nullptr};

        MotorStatusData status;

        uint8_t current_mode{
            control_table::operating_mode::POSITION};

        int32_t last_goal_position_raw{0};
        int16_t last_goal_current_raw{0};

        bool connected{false};
        bool torque_enabled{false};
        bool prepared_for_activation{false};
    };

    class DynamixelHardwareInterface
        : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        using CallbackReturn =
            rclcpp_lifecycle::node_interfaces::
                LifecycleNodeInterface::CallbackReturn;

        explicit DynamixelHardwareInterface(
            const rclcpp::NodeOptions &options =
                rclcpp::NodeOptions());

        ~DynamixelHardwareInterface() override;

        CallbackReturn on_configure(
            const rclcpp_lifecycle::State &) override;

        CallbackReturn on_activate(
            const rclcpp_lifecycle::State &) override;

        CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &) override;

        CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &) override;

        CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &) override;

    private:
        void declareParameters();
        bool loadParameters();
        bool openPort();
        void closePort();

        bool detectMotors();
        bool initializeMotors();
        bool initializeMotor(MotorDevice &motor);

        bool prepareMotorsForActivation();
        bool prepareMotorCommand(MotorDevice &motor);
        bool enablePreparedMotors();
        bool disableAllTorque();

        void dynamixelControlCallback(
            const dynamixel_hardware_msgs::msg::DynamixelControlMsgs::SharedPtr msg);

        void controlTimerCallback();
        bool changeOperatingMode(
            MotorDevice &motor,
            uint8_t new_mode);

        void readMotorStatuses();

        std::string device_name_;
        int baud_rate_{0};
        double protocol_version_{2.0};
        double status_rate_hz_{10.0};
        double control_rate_hz_{125.0};
        double control_timeout_sec_{0.5};
        bool command_enabled_{false};
        bool has_control_msg_{false};

        dynamixel::PortHandler *port_handler_{nullptr};
        dynamixel::PacketHandler *packet_handler_{nullptr};

        std::unique_ptr<MotorSetting> motor_setting_;
        std::unique_ptr<MotorStatus> motor_status_;
        std::string control_topic_;
        std::string status_topic_;
        rclcpp::Subscription<
            dynamixel_hardware_msgs::msg::DynamixelControlMsgs>::SharedPtr
            dynamixel_control_subscription_;
        rclcpp_lifecycle::LifecyclePublisher<
            dynamixel_hardware_msgs::msg::CurrentMotorStatus>::SharedPtr
            motor_status_publisher_;
        dynamixel_hardware_msgs::msg::DynamixelControlMsgs::SharedPtr
            latest_control_msg_;
        rclcpp::Time last_control_msg_time_{0, 0, RCL_ROS_TIME};
        mutable std::mutex control_msg_mutex_;

        std::vector<MotorDevice> motors_;

        rclcpp::TimerBase::SharedPtr control_timer_;
        rclcpp::TimerBase::SharedPtr status_timer_;
    };

} // namespace dynamixel_hardware_interface
