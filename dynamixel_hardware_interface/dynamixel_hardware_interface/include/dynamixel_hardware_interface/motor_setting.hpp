#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include "dynamixel_hardware_interface/control_table.hpp"
#include "dynamixel_hardware_interface/motor_model.hpp"

namespace dynamixel_hardware_interface
{

    class MotorSetting
    {
    public:
        MotorSetting(
            dynamixel::PortHandler *port_handler,
            dynamixel::PacketHandler *packet_handler,
            rclcpp::Logger logger);

        ~MotorSetting() = default;

        // 단일 모터 제어
        bool setTorque(uint8_t id, bool enable);

        bool setGoalPosition(
            uint8_t id,
            uint32_t position);

        bool setGoalCurrent(
            uint8_t id,
            int16_t current_raw,
            const MotorModelInfo &model_info);

        bool setGoalVelocity(
            uint8_t id,
            uint32_t velocity_raw);

        bool setGoalPwm(
            uint8_t id,
            uint16_t pwm_raw);

        // 여러 모터 제어
        bool setTorqueSync(
            const std::vector<uint8_t> &motor_ids,
            bool enable);

        bool setGoalPositionSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint32_t> &positions);

        bool setGoalCurrentSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<int16_t> &currents_raw,
            const std::vector<const MotorModelInfo *> &model_infos);

        // Operating Mode
        bool setOperatingMode(
            uint8_t id,
            uint8_t mode,
            const MotorModelInfo &model_info);

        // 단일 모터 제한값 설정
        bool setMinPositionLimit(
            uint8_t id,
            uint32_t limit);

        bool setReturnDelayTime(
            uint8_t id,
            uint8_t delay_raw);

        // status_indirect::SOURCE_BYTE_ADDRESSES 순서대로
        // Indirect Address 1~19에 원본 주소를 배선한다.
        bool setStatusIndirectAddressMapping(uint8_t id);

        bool setMaxPositionLimit(
            uint8_t id,
            uint32_t limit);

        bool setMaxVelocityLimit(
            uint8_t id,
            uint32_t limit);

        bool setMaxAccelerationLimit(
            uint8_t id,
            uint32_t limit);

        bool setTemperatureLimit(
            uint8_t id,
            uint8_t limit);

        bool setCurrentLimit(
            uint8_t id,
            uint16_t limit_raw,
            const MotorModelInfo &model_info);

        bool setPwmLimit(
            uint8_t id,
            uint16_t limit);

        bool setProfileVelocity(
            uint8_t id,
            uint32_t velocity_raw);

        bool setProfileAcceleration(
            uint8_t id,
            uint32_t acceleration_raw);

        bool setPositionPidGains(
            uint8_t id,
            uint16_t p_gain,
            uint16_t i_gain,
            uint16_t d_gain);

        bool setVelocityPiGains(
            uint8_t id,
            uint16_t p_gain,
            uint16_t i_gain);

        bool setFeedforwardGains(
            uint8_t id,
            uint16_t feedforward_1st_gain,
            uint16_t feedforward_2nd_gain);

        bool setShutdown(
            uint8_t id,
            uint8_t shutdown_mask);

        // 여러 모터 제한값 설정
        bool setMinPositionLimitSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint32_t> &limits);

        bool setMaxPositionLimitSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint32_t> &limits);

        bool setMaxVelocityLimitSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint32_t> &limits);

        bool setMaxAccelerationLimitSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint32_t> &limits);

        bool setTemperatureLimitSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint8_t> &limits);

        bool setCurrentLimitSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint16_t> &limits_raw,
            const std::vector<const MotorModelInfo *> &model_infos);

        bool setPwmLimitSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint16_t> &limits);

        bool setShutdownSync(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint8_t> &shutdown_masks);

        // Profile Acceleration + Profile Velocity + Goal Position
        bool syncWritePositionProfile(
            const std::vector<uint8_t> &motor_ids,
            const std::vector<double> &positions,
            const std::vector<double> &velocities,
            const std::vector<double> &accelerations);

    private:
        bool writeRegister(
            uint8_t id,
            const ControlItem &item,
            uint32_t value,
            const std::string &status_name);

        bool syncWriteRegister(
            const ControlItem &item,
            const std::vector<uint8_t> &motor_ids,
            const std::vector<uint32_t> &values,
            const std::string &status_name);

        void appendBytes(
            std::vector<uint8_t> &data,
            uint32_t value,
            uint8_t byte_size) const;

        int32_t radianToTick(double radian) const;

        dynamixel::PortHandler *port_handler_;
        dynamixel::PacketHandler *packet_handler_;
        rclcpp::Logger logger_;
    };

} // namespace dynamixel_hardware_interface
