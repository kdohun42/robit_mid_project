#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>

#include "dynamixel_hardware_interface/control_table.hpp"
#include "dynamixel_hardware_interface/motor_model.hpp"

namespace dynamixel_hardware_interface
{

    struct MotorStatusData
    {
        uint8_t id{0};

        // Raw data
        int32_t present_position_raw{0};
        int32_t goal_position_raw{0};
        int32_t present_velocity_raw{0};

        // MX-28: Present Load
        // MX-64, XH540: Present Current
        int16_t present_feedback_raw{0};
        FeedbackType feedback_type{FeedbackType::NONE};
 
        uint16_t input_voltage_raw{0};
        uint8_t temperature_c{0};
        uint8_t moving_status{0};
        uint8_t hardware_error_status{0};

        double present_position_rad{0.0};
        double goal_position_rad{0.0};
        double present_velocity_rad_s{0.0};
        double input_voltage_v{0.0};

        double present_current_mA{0.0};
        double present_load_percent{0.0};

        bool communication_ok{false};
    };

    class MotorStatus
    {
    public:
        MotorStatus(
            dynamixel::PortHandler *port_handler,
            dynamixel::PacketHandler *packet_handler,
            rclcpp::Logger logger);

        ~MotorStatus() = default;

        // 한 모터의 전체 상태 읽기
        bool readMotorStatus(
            uint8_t id,
            const MotorModelInfo &model_info,
            MotorStatusData &status);

        // 여러 모터의 전체 상태 읽기
        bool readMotorStatusSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<const MotorModelInfo *> &model_infos,
            std::vector<MotorStatusData> &statuses);

        // Individual Read
        bool getCurrentPosition(uint8_t id, int32_t &position_raw);
        bool getGoalPosition(uint8_t id, int32_t &goal_position_raw);
        bool getCurrentVelocity(uint8_t id, int32_t &velocity_raw);
        bool getInputVoltage(uint8_t id, uint16_t &voltage_raw);
        bool getCurrentTemperature(uint8_t id, uint8_t &temperature_c);
        bool getPresentFeedback(uint8_t id, int16_t &feedback_raw);
        bool getMovingStatus(uint8_t id, uint8_t &moving_status);
        bool getHardwareErrorStatus(uint8_t id, uint8_t &error_status);
        bool getReturnDelayTime(uint8_t id, uint8_t &delay_raw);

        // Sync Read
        bool getCurrentPositionSync(
            const std::vector<uint8_t> &motor_ids,
            std::vector<int32_t> &positions_raw);

        bool getGoalPositionSync(
            const std::vector<uint8_t> &motor_ids,
            std::vector<int32_t> &goal_positions_raw);

        bool getCurrentVelocitySync(
            const std::vector<uint8_t> &motor_ids,
            std::vector<int32_t> &velocities_raw);

        bool getInputVoltageSync(
            const std::vector<uint8_t> &motor_ids,
            std::vector<uint16_t> &voltages_raw);

        bool getCurrentTemperatureSync(
            const std::vector<uint8_t> &motor_ids,
            std::vector<uint8_t> &temperatures_c);

        bool getPresentFeedbackSync(
            const std::vector<uint8_t> &motor_ids,
            std::vector<int16_t> &feedback_values_raw);

        bool getMovingStatusSync(
            const std::vector<uint8_t> &motor_ids,
            std::vector<uint8_t> &moving_status_values);

        bool getHardwareErrorStatusSync(
            const std::vector<uint8_t> &motor_ids,
            std::vector<uint8_t> &error_status_values);

        // 단위 변환
        double positionRawToRadian(int32_t raw) const;
        double velocityRawToRadianPerSecond(int32_t raw) const;
        double voltageRawToVolt(uint16_t raw) const;

        double feedbackRawToMilliAmpere(
            int16_t raw,
            const MotorModelInfo &model_info) const;

        double feedbackRawToLoadPercent(
            int16_t raw,
            const MotorModelInfo &model_info) const;

    private:
        bool readRegister(
            uint8_t id,
            const ControlItem &item,
            uint32_t &value,
            const std::string &status_name);

        bool syncReadRegister(
            const ControlItem &item,
            const std::vector<uint8_t> &motor_ids,
            std::vector<uint32_t> &values,
            const std::string &status_name);

        // Indirect Address/Data로 묶은 상태 블록(19바이트)을 한 번에 읽어
        // statuses의 raw 필드들을 채운다.
        bool syncReadIndirectStatus(
            const ControlItem &item,
            const std::vector<uint8_t> &motor_ids,
            std::vector<MotorStatusData> &statuses);

        dynamixel::PortHandler *port_handler_;
        dynamixel::PacketHandler *packet_handler_;
        rclcpp::Logger logger_;
    };

} // namespace dynamixel_hardware_interface