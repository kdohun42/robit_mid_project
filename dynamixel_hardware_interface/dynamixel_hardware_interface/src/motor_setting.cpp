#include "dynamixel_hardware_interface/motor_setting.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace dynamixel_hardware_interface
{

    namespace
    {

        constexpr double PI = 3.14159265358979323846;
        constexpr double POSITION_TICKS_PER_REVOLUTION = 4096.0;
        constexpr double VELOCITY_UNIT_RPM = 0.229;

        constexpr uint8_t POSITION_PROFILE_DATA_LENGTH = 12;
        constexpr uint8_t PROTOCOL2_ALERT_ERROR = 0x80;

        bool shouldLogProtocolAlert()
        {
            static auto last_log_time =
                std::chrono::steady_clock::time_point{};

            const auto now =
                std::chrono::steady_clock::now();

            if (now - last_log_time <
                std::chrono::seconds(2))
            {
                return false;
            }

            last_log_time = now;
            return true;
        }

    } // namespace

    MotorSetting::MotorSetting(
        dynamixel::PortHandler *port_handler,
        dynamixel::PacketHandler *packet_handler,
        rclcpp::Logger logger)
        : port_handler_(port_handler),
          packet_handler_(packet_handler),
          logger_(logger)
    {
    }

    // 각개 모터 제어
    bool MotorSetting::setTorque(uint8_t id, bool enable)
    {
        const uint8_t torque_value =
            enable
                ? control_table::torque_enable::ON
                : control_table::torque_enable::OFF;

        return writeRegister(
            id,
            control_table::ram::TORQUE_ENABLE,
            torque_value,
            "Torque Enable");
    }

    bool MotorSetting::setGoalPosition(
        uint8_t id,
        uint32_t position)
    {
        return writeRegister(
            id,
            control_table::ram::GOAL_POSITION,
            position,
            "Goal Position");
    }

    bool MotorSetting::setGoalCurrent(
        uint8_t id,
        int16_t current_raw,
        const MotorModelInfo &model_info)
    {
        if (!model_info.supports_current_mode)
        {
            RCLCPP_ERROR(
                logger_,
                "ID %u model %s does not support Goal Current",
                id,
                model_info.name);

            return false;
        }

        return writeRegister(
            id,
            control_table::ram::GOAL_CURRENT,
            static_cast<uint16_t>(current_raw),
            "Goal Current");
    }

    bool MotorSetting::setGoalVelocity(
        uint8_t id,
        uint32_t velocity_raw)
    {
        return writeRegister(
            id,
            control_table::ram::GOAL_VELOCITY,
            velocity_raw,
            "Goal Velocity");
    }

    bool MotorSetting::setGoalPwm(
        uint8_t id,
        uint16_t pwm_raw)
    {
        return writeRegister(
            id,
            control_table::ram::GOAL_PWM,
            pwm_raw,
            "Goal PWM");
    }

    // sync방식 여러 모터 제어
    bool MotorSetting::setTorqueSync(
        const std::vector<uint8_t> &motor_ids,
        bool enable)
    {
        if (motor_ids.empty())
        {
            return true;
        }

        const uint32_t torque_value =
            enable
                ? control_table::torque_enable::ON
                : control_table::torque_enable::OFF;

        std::vector<uint32_t> values(
            motor_ids.size(),
            torque_value);

        return syncWriteRegister(
            control_table::ram::TORQUE_ENABLE,
            motor_ids,
            values,
            "Torque Enable");
    }

    bool MotorSetting::setGoalPositionSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint32_t> &positions)
    {
        return syncWriteRegister(
            control_table::ram::GOAL_POSITION,
            motor_ids,
            positions,
            "Goal Position");
    }

    bool MotorSetting::setGoalCurrentSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<int16_t> &currents_raw,
        const std::vector<const MotorModelInfo *> &model_infos)
    {
        if (motor_ids.size() != currents_raw.size() ||
            motor_ids.size() != model_infos.size())
        {
            RCLCPP_ERROR(
                logger_,
                "Goal Current size mismatch: ids=%zu, currents=%zu, models=%zu",
                motor_ids.size(),
                currents_raw.size(),
                model_infos.size());

            return false;
        }

        std::vector<uint32_t> raw_currents;
        raw_currents.reserve(currents_raw.size());

        for (size_t i = 0; i < motor_ids.size(); ++i)
        {
            if (model_infos[i] == nullptr)
            {
                RCLCPP_ERROR(
                    logger_,
                    "Model information is null for ID %u",
                    motor_ids[i]);

                return false;
            }

            if (!model_infos[i]->supports_current_mode)
            {
                RCLCPP_ERROR(
                    logger_,
                    "ID %u model %s does not support Goal Current",
                    motor_ids[i],
                    model_infos[i]->name);

                return false;
            }

            raw_currents.push_back(
                static_cast<uint16_t>(currents_raw[i]));
        }

        return syncWriteRegister(
            control_table::ram::GOAL_CURRENT,
            motor_ids,
            raw_currents,
            "Goal Current");
    }

    // 모터바꿈이
    bool MotorSetting::setOperatingMode(
        uint8_t id,
        uint8_t mode,
        const MotorModelInfo &model_info)
    {
        if (!supportsOperatingMode(model_info, mode))
        {
            RCLCPP_ERROR(
                logger_,
                "ID %u model %s does not support operating mode %u",
                id,
                model_info.name,
                mode);

            return false;
        }

        return writeRegister(
            id,
            control_table::eeprom::OPERATING_MODE,
            mode,
            "Operating Mode");
    }

    // 제한값 설정이(개별)
    bool MotorSetting::setMinPositionLimit(
        uint8_t id,
        uint32_t limit)
    {
        return writeRegister(
            id,
            control_table::eeprom::MIN_POSITION_LIMIT,
            limit,
            "Min Position Limit");
    }

    bool MotorSetting::setReturnDelayTime(
        uint8_t id,
        uint8_t delay_raw)
    {
        return writeRegister(
            id,
            control_table::eeprom::RETURN_DELAY_TIME,
            delay_raw,
            "Return Delay Time");
    }

    bool MotorSetting::setStatusIndirectAddressMapping(uint8_t id)
    {
        for (size_t i = 0;
             i < control_table::status_indirect::SOURCE_BYTE_ADDRESSES.size();
             ++i)
        {
            const ControlItem indirect_address_item{
                static_cast<uint16_t>(
                    control_table::eeprom::INDIRECT_ADDRESS_1.address +
                    i * control_table::eeprom::INDIRECT_ADDRESS_1.length),
                control_table::eeprom::INDIRECT_ADDRESS_1.length};

            if (!writeRegister(
                    id,
                    indirect_address_item,
                    control_table::status_indirect::SOURCE_BYTE_ADDRESSES[i],
                    "Indirect Address"))
            {
                return false;
            }
        }

        return true;
    }
    bool MotorSetting::setMaxPositionLimit(
        uint8_t id,
        uint32_t limit)
    {
        return writeRegister(
            id,
            control_table::eeprom::MAX_POSITION_LIMIT,
            limit,
            "Max Position Limit");
    }

    bool MotorSetting::setMaxVelocityLimit(
        uint8_t id,
        uint32_t limit)
    {
        return writeRegister(
            id,
            control_table::eeprom::VELOCITY_LIMIT,
            limit,
            "Velocity Limit");
    }

    bool MotorSetting::setMaxAccelerationLimit(
        uint8_t id,
        uint32_t limit)
    {
        return writeRegister(
            id,
            control_table::eeprom::ACCELERATION_LIMIT,
            limit,
            "Acceleration Limit");
    }

    bool MotorSetting::setTemperatureLimit(
        uint8_t id,
        uint8_t limit)
    {
        return writeRegister(
            id,
            control_table::eeprom::TEMPERATURE_LIMIT,
            limit,
            "Temperature Limit");
    }

    bool MotorSetting::setCurrentLimit(
        uint8_t id,
        uint16_t limit_raw,
        const MotorModelInfo &model_info)
    {
        if (!model_info.supports_current_limit)
        {
            RCLCPP_ERROR(
                logger_,
                "ID %u model %s does not support Current Limit",
                id,
                model_info.name);

            return false;
        }

        return writeRegister(
            id,
            control_table::eeprom::CURRENT_LIMIT,
            limit_raw,
            "Current Limit");
    }

    bool MotorSetting::setPwmLimit(
        uint8_t id,
        uint16_t limit)
    {
        return writeRegister(
            id,
            control_table::eeprom::PWM_LIMIT,
            limit,
            "PWM Limit");
    }

    bool MotorSetting::setProfileVelocity(
        uint8_t id,
        uint32_t velocity_raw)
    {
        return writeRegister(
            id,
            control_table::ram::PROFILE_VELOCITY,
            velocity_raw,
            "Profile Velocity");
    }

    bool MotorSetting::setProfileAcceleration(
        uint8_t id,
        uint32_t acceleration_raw)
    {
        return writeRegister(
            id,
            control_table::ram::PROFILE_ACCELERATION,
            acceleration_raw,
            "Profile Acceleration");
    }

    bool MotorSetting::setPositionPidGains(
        uint8_t id,
        uint16_t p_gain,
        uint16_t i_gain,
        uint16_t d_gain)
    {
        return writeRegister(
                   id,
                   control_table::ram::POSITION_P_GAIN,
                   p_gain,
                   "Position P Gain") &&
               writeRegister(
                   id,
                   control_table::ram::POSITION_I_GAIN,
                   i_gain,
                   "Position I Gain") &&
               writeRegister(
                   id,
                   control_table::ram::POSITION_D_GAIN,
                   d_gain,
                   "Position D Gain");
    }

    bool MotorSetting::setVelocityPiGains(
        uint8_t id,
        uint16_t p_gain,
        uint16_t i_gain)
    {
        return writeRegister(
                   id,
                   control_table::ram::VELOCITY_P_GAIN,
                   p_gain,
                   "Velocity P Gain") &&
               writeRegister(
                   id,
                   control_table::ram::VELOCITY_I_GAIN,
                   i_gain,
                   "Velocity I Gain");
    }

    bool MotorSetting::setFeedforwardGains(
        uint8_t id,
        uint16_t feedforward_1st_gain,
        uint16_t feedforward_2nd_gain)
    {
        return writeRegister(
                   id,
                   control_table::ram::FEEDFORWARD_1ST_GAIN,
                   feedforward_1st_gain,
                   "Feedforward 1st Gain") &&
               writeRegister(
                   id,
                   control_table::ram::FEEDFORWARD_2ND_GAIN,
                   feedforward_2nd_gain,
                   "Feedforward 2nd Gain");
    }

    bool MotorSetting::setShutdown(
        uint8_t id,
        uint8_t shutdown_mask)
    {
        return writeRegister(
            id,
            control_table::eeprom::SHUTDOWN,
            shutdown_mask,
            "Shutdown");
    }

    // sync방식 모터 제한값 설정
    bool MotorSetting::setMinPositionLimitSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint32_t> &limits)
    {
        return syncWriteRegister(
            control_table::eeprom::MIN_POSITION_LIMIT,
            motor_ids,
            limits,
            "Min Position Limit");
    }

    bool MotorSetting::setMaxPositionLimitSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint32_t> &limits)
    {
        return syncWriteRegister(
            control_table::eeprom::MAX_POSITION_LIMIT,
            motor_ids,
            limits,
            "Max Position Limit");
    }

    bool MotorSetting::setMaxVelocityLimitSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint32_t> &limits)
    {
        return syncWriteRegister(
            control_table::eeprom::VELOCITY_LIMIT,
            motor_ids,
            limits,
            "Velocity Limit");
    }

    bool MotorSetting::setMaxAccelerationLimitSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint32_t> &limits)
    {
        return syncWriteRegister(
            control_table::eeprom::ACCELERATION_LIMIT,
            motor_ids,
            limits,
            "Acceleration Limit");
    }

    bool MotorSetting::setTemperatureLimitSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint8_t> &limits)
    {
        std::vector<uint32_t> values(
            limits.begin(),
            limits.end());

        return syncWriteRegister(
            control_table::eeprom::TEMPERATURE_LIMIT,
            motor_ids,
            values,
            "Temperature Limit");
    }

    bool MotorSetting::setCurrentLimitSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint16_t> &limits_raw,
        const std::vector<const MotorModelInfo *> &model_infos)
    {
        if (motor_ids.size() != limits_raw.size() ||
            motor_ids.size() != model_infos.size())
        {
            RCLCPP_ERROR(
                logger_,
                "Current Limit size mismatch: ids=%zu, limits=%zu, models=%zu",
                motor_ids.size(),
                limits_raw.size(),
                model_infos.size());

            return false;
        }

        for (size_t i = 0; i < motor_ids.size(); ++i)
        {
            if (model_infos[i] == nullptr)
            {
                RCLCPP_ERROR(
                    logger_,
                    "Model information is null for ID %u",
                    motor_ids[i]);

                return false;
            }

            if (!model_infos[i]->supports_current_limit)
            {
                RCLCPP_ERROR(
                    logger_,
                    "ID %u model %s does not support Current Limit",
                    motor_ids[i],
                    model_infos[i]->name);

                return false;
            }
        }

        std::vector<uint32_t> values(
            limits_raw.begin(),
            limits_raw.end());

        return syncWriteRegister(
            control_table::eeprom::CURRENT_LIMIT,
            motor_ids,
            values,
            "Current Limit");
    }

    bool MotorSetting::setPwmLimitSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint16_t> &limits)
    {
        std::vector<uint32_t> values(
            limits.begin(),
            limits.end());

        return syncWriteRegister(
            control_table::eeprom::PWM_LIMIT,
            motor_ids,
            values,
            "PWM Limit");
    }

    bool MotorSetting::setShutdownSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint8_t> &shutdown_masks)
    {
        std::vector<uint32_t> values(
            shutdown_masks.begin(),
            shutdown_masks.end());

        return syncWriteRegister(
            control_table::eeprom::SHUTDOWN,
            motor_ids,
            values,
            "Shutdown");
    }

    // 위치프로파일 설정(sync방식)
    bool MotorSetting::syncWritePositionProfile(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<double> &positions,
        const std::vector<double> &velocities,
        const std::vector<double> &accelerations)
    {
        if (motor_ids.empty())
        {
            return true;
        }

        if (motor_ids.size() != positions.size() ||
            motor_ids.size() != velocities.size() ||
            motor_ids.size() != accelerations.size())
        {
            RCLCPP_ERROR(
                logger_,
                "Position profile size mismatch: ids=%zu, positions=%zu, velocities=%zu, accelerations=%zu",
                motor_ids.size(),
                positions.size(),
                velocities.size(),
                accelerations.size());

            return false;
        }

        dynamixel::GroupSyncWrite sync_write(
            port_handler_,
            packet_handler_,
            control_table::ram::PROFILE_ACCELERATION.address,
            POSITION_PROFILE_DATA_LENGTH);

        constexpr double velocity_unit_rad_s =
            VELOCITY_UNIT_RPM * 2.0 * PI / 60.0;

        for (size_t i = 0; i < motor_ids.size(); ++i)
        {
            std::vector<uint8_t> control_data;
            control_data.reserve(POSITION_PROFILE_DATA_LENGTH);

            const int32_t position_tick =
                radianToTick(positions[i]);

            const uint32_t goal_position =
                static_cast<uint32_t>(
                    std::max<int32_t>(0, position_tick));

            const uint32_t goal_velocity =
                velocities[i] <= 0.0
                    ? 0
                    : static_cast<uint32_t>(
                          velocities[i] / velocity_unit_rad_s);

            const uint32_t goal_acceleration =
                accelerations[i] <= 0.0
                    ? 0
                    : static_cast<uint32_t>(
                          accelerations[i]);

            appendBytes(
                control_data,
                goal_acceleration,
                control_table::ram::PROFILE_ACCELERATION.length);

            appendBytes(
                control_data,
                goal_velocity,
                control_table::ram::PROFILE_VELOCITY.length);

            appendBytes(
                control_data,
                goal_position,
                control_table::ram::GOAL_POSITION.length);

            if (!sync_write.addParam(
                    motor_ids[i],
                    control_data.data()))
            {
                RCLCPP_ERROR(
                    logger_,
                    "Failed to add position profile parameter for ID %u",
                    motor_ids[i]);

                return false;
            }
        }

        const int communication_result =
            sync_write.txPacket();

        if (communication_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(
                logger_,
                "Failed to send position profile packet: %s",
                packet_handler_->getTxRxResult(
                    communication_result));

            return false;
        }

        sync_write.clearParam();

        return true;
    }

    // 쓰기 함수
    bool MotorSetting::writeRegister(
        uint8_t id,
        const ControlItem &item,
        uint32_t value,
        const std::string &status_name)
    {
        uint8_t dxl_error = 0;
        int communication_result = COMM_TX_FAIL;

        switch (item.length)
        {
        case 1:
            communication_result =
                packet_handler_->write1ByteTxRx(
                    port_handler_,
                    id,
                    item.address,
                    static_cast<uint8_t>(value),
                    &dxl_error);
            break;

        case 2:
            communication_result =
                packet_handler_->write2ByteTxRx(
                    port_handler_,
                    id,
                    item.address,
                    static_cast<uint16_t>(value),
                    &dxl_error);
            break;

        case 4:
            communication_result =
                packet_handler_->write4ByteTxRx(
                    port_handler_,
                    id,
                    item.address,
                    value,
                    &dxl_error);
            break;

        default:
            RCLCPP_ERROR(
                logger_,
                "[writeRegister] Unsupported register length: %u",
                item.length);

            return false;
        }

        if (communication_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(
                logger_,
                "[writeRegister] Communication failed - %s, ID %u: %s (%d)",
                status_name.c_str(),
                id,
                packet_handler_->getTxRxResult(
                    communication_result),
                communication_result);

            return false;
        }

        if (dxl_error != 0)
        {
            if (dxl_error == PROTOCOL2_ALERT_ERROR)
            {
                if (shouldLogProtocolAlert())
                {
                    RCLCPP_WARN(
                        logger_,
                        "[writeRegister] Protocol 2.0 alert - %s, ID %u: %s (%u). Continuing.",
                        status_name.c_str(),
                        id,
                        packet_handler_->getRxPacketError(
                            dxl_error),
                        dxl_error);
                }
            }
            else
            {
                RCLCPP_ERROR(
                    logger_,
                    "[writeRegister] Dynamixel error - %s, ID %u: %s (%u)",
                    status_name.c_str(),
                    id,
                    packet_handler_->getRxPacketError(
                        dxl_error),
                    dxl_error);

                return false;
            }
        }

        return true;
    }

    bool MotorSetting::syncWriteRegister(
        const ControlItem &item,
        const std::vector<uint8_t> &motor_ids,
        const std::vector<uint32_t> &values,
        const std::string &status_name)
    {
        if (motor_ids.empty())
        {
            return true;
        }

        if (motor_ids.size() != values.size())
        {
            RCLCPP_ERROR(
                logger_,
                "[syncWriteRegister] Size mismatch for %s: ids=%zu, values=%zu",
                status_name.c_str(),
                motor_ids.size(),
                values.size());

            return false;
        }

        if (item.length != 1 &&
            item.length != 2 &&
            item.length != 4)
        {
            RCLCPP_ERROR(
                logger_,
                "[syncWriteRegister] Unsupported register length: %u",
                item.length);

            return false;
        }

        dynamixel::GroupSyncWrite sync_write(
            port_handler_,
            packet_handler_,
            item.address,
            item.length);

        for (size_t i = 0; i < motor_ids.size(); ++i)
        {
            std::vector<uint8_t> data;
            data.reserve(item.length);

            appendBytes(
                data,
                values[i],
                item.length);

            if (!sync_write.addParam(
                    motor_ids[i],
                    data.data()))
            {
                RCLCPP_ERROR(
                    logger_,
                    "[syncWriteRegister] Failed to add %s parameter for ID %u",
                    status_name.c_str(),
                    motor_ids[i]);

                return false;
            }
        }

        const int communication_result =
            sync_write.txPacket();

        if (communication_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(
                logger_,
                "[syncWriteRegister] Communication failed - %s: %s (%d)",
                status_name.c_str(),
                packet_handler_->getTxRxResult(
                    communication_result),
                communication_result);

            return false;
        }

        sync_write.clearParam();

        return true;
    }

    void MotorSetting::appendBytes(
        std::vector<uint8_t> &data,
        uint32_t value,
        uint8_t byte_size) const
    {
        switch (byte_size)
        {
        case 1:
            data.push_back(
                static_cast<uint8_t>(
                    value & 0xFF));
            break;

        case 2:
            data.push_back(
                DXL_LOBYTE(value));

            data.push_back(
                DXL_HIBYTE(value));
            break;

        case 4:
            data.push_back(
                DXL_LOBYTE(
                    DXL_LOWORD(value)));

            data.push_back(
                DXL_HIBYTE(
                    DXL_LOWORD(value)));

            data.push_back(
                DXL_LOBYTE(
                    DXL_HIWORD(value)));

            data.push_back(
                DXL_HIBYTE(
                    DXL_HIWORD(value)));
            break;

        default:
            break;
        }
    }

    int32_t MotorSetting::radianToTick(double radian) const
    {
        const double clamped_radian =
            std::clamp(radian, -PI, PI);

        const double tick =
            (clamped_radian + PI) *
            ((POSITION_TICKS_PER_REVOLUTION - 1.0) /
             (2.0 * PI));

        return static_cast<int32_t>(std::lround(tick));
    }

} // namespace dynamixel_hardware_interface
