#include "dynamixel_hardware_interface/motor_status.hpp"

#include <chrono>
#include <cmath>

namespace dynamixel_hardware_interface
{

    namespace
    {

        constexpr double PI = 3.14159265358979323846;
        constexpr double POSITION_TICKS_PER_REVOLUTION = 4096.0;
        constexpr double VELOCITY_UNIT_RPM = 0.229;
        constexpr double VOLTAGE_UNIT_VOLT = 0.1;
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

    MotorStatus::MotorStatus(
        dynamixel::PortHandler *port_handler,
        dynamixel::PacketHandler *packet_handler,
        rclcpp::Logger logger)
        : port_handler_(port_handler),
          packet_handler_(packet_handler),
          logger_(logger)
    {
    }

    /* ============================================================
     * 전체 상태 읽기
     * ============================================================ */

    bool MotorStatus::readMotorStatus(
        uint8_t id,
        const MotorModelInfo &model_info,
        MotorStatusData &status)
    {
        status = MotorStatusData{};
        status.id = id;
        status.communication_ok = false;

        if (!getCurrentPosition(
                id,
                status.present_position_raw))
        {
            return false;
        }

        if (!getGoalPosition(
                id,
                status.goal_position_raw))
        {
            return false;
        }

        if (!getCurrentVelocity(
                id,
                status.present_velocity_raw))
        {
            return false;
        }

        if (!getPresentFeedback(
                id,
                status.present_feedback_raw))
        {
            return false;
        }

        if (!getInputVoltage(
                id,
                status.input_voltage_raw))
        {
            return false;
        }

        if (!getCurrentTemperature(
                id,
                status.temperature_c))
        {
            return false;
        }

        if (!getMovingStatus(
                id,
                status.moving_status))
        {
            return false;
        }

        if (!getHardwareErrorStatus(
                id,
                status.hardware_error_status))
        {
            return false;
        }

        status.present_position_rad =
            positionRawToRadian(
                status.present_position_raw);

        status.goal_position_rad =
            positionRawToRadian(
                status.goal_position_raw);

        status.present_velocity_rad_s =
            velocityRawToRadianPerSecond(
                status.present_velocity_raw);

        status.input_voltage_v =
            voltageRawToVolt(
                status.input_voltage_raw);

        status.feedback_type =
            model_info.feedback_type;

        if (model_info.feedback_type ==
            FeedbackType::PRESENT_CURRENT)
        {
            status.present_current_mA =
                feedbackRawToMilliAmpere(
                    status.present_feedback_raw,
                    model_info);

            status.present_load_percent = 0.0;
        }
        else if (
            model_info.feedback_type ==
            FeedbackType::PRESENT_LOAD)
        {
            status.present_load_percent =
                feedbackRawToLoadPercent(
                    status.present_feedback_raw,
                    model_info);

            status.present_current_mA = 0.0;
        }
        else
        {
            status.present_current_mA = 0.0;
            status.present_load_percent = 0.0;
        }

        status.communication_ok = true;

        return true;
    }

    bool MotorStatus::readMotorStatusSync(
        const std::vector<uint8_t> &motor_ids,
        const std::vector<const MotorModelInfo *> &model_infos,
        std::vector<MotorStatusData> &statuses)
    {
        if (motor_ids.size() != model_infos.size())
        {
            RCLCPP_ERROR(
                logger_,
                "Motor/model size mismatch: ids=%zu, models=%zu",
                motor_ids.size(),
                model_infos.size());

            return false;
        }

        if (motor_ids.empty())
        {
            statuses.clear();
            return true;
        }
        statuses.resize(motor_ids.size());

        const ControlItem indirect_status_item{
            control_table::ram::INDIRECT_DATA_1.address,
            control_table::status_indirect::TOTAL_LENGTH};

        if (!syncReadIndirectStatus(
                indirect_status_item,
                motor_ids,
                statuses))
        {
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

            MotorStatusData &status = statuses[i];
            const MotorModelInfo &model_info =
                *model_infos[i];

            status.present_position_rad =
                positionRawToRadian(
                    status.present_position_raw);

            status.goal_position_rad =
                positionRawToRadian(
                    status.goal_position_raw);

            status.present_velocity_rad_s =
                velocityRawToRadianPerSecond(
                    status.present_velocity_raw);

            status.input_voltage_v =
                voltageRawToVolt(
                    status.input_voltage_raw);

            status.feedback_type =
                model_info.feedback_type;

            if (model_info.feedback_type ==
                FeedbackType::PRESENT_CURRENT)
            {
                status.present_current_mA =
                    feedbackRawToMilliAmpere(
                        status.present_feedback_raw,
                        model_info);

                status.present_load_percent = 0.0;
            }
            else if (
                model_info.feedback_type ==
                FeedbackType::PRESENT_LOAD)
            {
                status.present_load_percent =
                    feedbackRawToLoadPercent(
                        status.present_feedback_raw,
                        model_info);

                status.present_current_mA = 0.0;
            }
            else
            {
                status.present_current_mA = 0.0;
                status.present_load_percent = 0.0;
            }

            status.communication_ok = true;
        }

        return true;
    }

    /* ============================================================
     * 개별 읽기
     * ============================================================ */

    bool MotorStatus::getCurrentPosition(
        uint8_t id,
        int32_t &position_raw)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::ram::PRESENT_POSITION,
                value,
                "Present Position"))
        {
            return false;
        }

        position_raw =
            static_cast<int32_t>(value);

        return true;
    }

    bool MotorStatus::getGoalPosition(
        uint8_t id,
        int32_t &goal_position_raw)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::ram::GOAL_POSITION,
                value,
                "Goal Position"))
        {
            return false;
        }

        goal_position_raw =
            static_cast<int32_t>(value);

        return true;
    }

    bool MotorStatus::getCurrentVelocity(
        uint8_t id,
        int32_t &velocity_raw)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::ram::PRESENT_VELOCITY,
                value,
                "Present Velocity"))
        {
            return false;
        }

        velocity_raw =
            static_cast<int32_t>(value);

        return true;
    }

    bool MotorStatus::getInputVoltage(
        uint8_t id,
        uint16_t &voltage_raw)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::ram::PRESENT_INPUT_VOLTAGE,
                value,
                "Present Input Voltage"))
        {
            return false;
        }

        voltage_raw =
            static_cast<uint16_t>(value);

        return true;
    }

    bool MotorStatus::getCurrentTemperature(
        uint8_t id,
        uint8_t &temperature_c)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::ram::PRESENT_TEMPERATURE,
                value,
                "Present Temperature"))
        {
            return false;
        }

        temperature_c =
            static_cast<uint8_t>(value);

        return true;
    }

    bool MotorStatus::getPresentFeedback(
        uint8_t id,
        int16_t &feedback_raw)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::ram::PRESENT_FEEDBACK,
                value,
                "Present Feedback"))
        {
            return false;
        }

        feedback_raw =
            static_cast<int16_t>(
                static_cast<uint16_t>(value));

        return true;
    }

    bool MotorStatus::getMovingStatus(
        uint8_t id,
        uint8_t &moving_status)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::ram::MOVING_STATUS,
                value,
                "Moving Status"))
        {
            return false;
        }

        moving_status =
            static_cast<uint8_t>(value);

        return true;
    }

    bool MotorStatus::getHardwareErrorStatus(
        uint8_t id,
        uint8_t &error_status)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::ram::HARDWARE_ERROR_STATUS,
                value,
                "Hardware Error Status"))
        {
            return false;
        }

        error_status =
            static_cast<uint8_t>(value);

        return true;
    }

    bool MotorStatus::getReturnDelayTime(
        uint8_t id,
        uint8_t &delay_raw)
    {
        uint32_t value = 0;

        if (!readRegister(
                id,
                control_table::eeprom::RETURN_DELAY_TIME,
                value,
                "Return Delay Time"))
        {
            return false;
        }

        delay_raw =
            static_cast<uint8_t>(value);

        return true;
    }

    /* ============================================================
     * Sync Read
     * ============================================================ */

    bool MotorStatus:: getCurrentPositionSync(
        const std::vector<uint8_t> &motor_ids,
        std::vector<int32_t> &positions_raw)
    {
        std::vector<uint32_t> values;

        if (!syncReadRegister(
                control_table::ram::PRESENT_POSITION,
                motor_ids,
                values,
                "Present Position"))
        {
            return false;
        }

        positions_raw.resize(values.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            positions_raw[i] =
                static_cast<int32_t>(values[i]);
        }

        return true;
    }

    bool MotorStatus::getGoalPositionSync(
        const std::vector<uint8_t> &motor_ids,
        std::vector<int32_t> &goal_positions_raw)
    {
        std::vector<uint32_t> values;

        if (!syncReadRegister(
                control_table::ram::GOAL_POSITION,
                motor_ids,
                values,
                "Goal Position"))
        {
            return false;
        }

        goal_positions_raw.resize(values.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            goal_positions_raw[i] =
                static_cast<int32_t>(values[i]);
        }

        return true;
    }

    bool MotorStatus::getCurrentVelocitySync(
        const std::vector<uint8_t> &motor_ids,
        std::vector<int32_t> &velocities_raw)
    {
        std::vector<uint32_t> values;

        if (!syncReadRegister(
                control_table::ram::PRESENT_VELOCITY,
                motor_ids,
                values,
                "Present Velocity"))
        {
            return false;
        }

        velocities_raw.resize(values.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            velocities_raw[i] =
                static_cast<int32_t>(values[i]);
        }

        return true;
    }

    bool MotorStatus::getInputVoltageSync(
        const std::vector<uint8_t> &motor_ids,
        std::vector<uint16_t> &voltages_raw)
    {
        std::vector<uint32_t> values;

        if (!syncReadRegister(
                control_table::ram::PRESENT_INPUT_VOLTAGE,
                motor_ids,
                values,
                "Present Input Voltage"))
        {
            return false;
        }

        voltages_raw.resize(values.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            voltages_raw[i] =
                static_cast<uint16_t>(values[i]);
        }

        return true;
    }

    bool MotorStatus::getCurrentTemperatureSync(
        const std::vector<uint8_t> &motor_ids,
        std::vector<uint8_t> &temperatures_c)
    {
        std::vector<uint32_t> values;

        if (!syncReadRegister(
                control_table::ram::PRESENT_TEMPERATURE,
                motor_ids,
                values,
                "Present Temperature"))
        {
            return false;
        }

        temperatures_c.resize(values.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            temperatures_c[i] =
                static_cast<uint8_t>(values[i]);
        }

        return true;
    }

    bool MotorStatus::getPresentFeedbackSync(
        const std::vector<uint8_t> &motor_ids,
        std::vector<int16_t> &feedback_values_raw)
    {
        std::vector<uint32_t> values;

        if (!syncReadRegister(
                control_table::ram::PRESENT_FEEDBACK,
                motor_ids,
                values,
                "Present Feedback"))
        {
            return false;
        }

        feedback_values_raw.resize(values.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            feedback_values_raw[i] =
                static_cast<int16_t>(
                    static_cast<uint16_t>(values[i]));
        }

        return true;
    }

    bool MotorStatus::getMovingStatusSync(
        const std::vector<uint8_t> &motor_ids,
        std::vector<uint8_t> &moving_status_values)
    {
        std::vector<uint32_t> values;

        if (!syncReadRegister(
                control_table::ram::MOVING_STATUS,
                motor_ids,
                values,
                "Moving Status"))
        {
            return false;
        }

        moving_status_values.resize(values.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            moving_status_values[i] =
                static_cast<uint8_t>(values[i]);
        }

        return true;
    }

    bool MotorStatus::getHardwareErrorStatusSync(
        const std::vector<uint8_t> &motor_ids,
        std::vector<uint8_t> &error_status_values)
    {
        std::vector<uint32_t> values;

        if (!syncReadRegister(
                control_table::ram::HARDWARE_ERROR_STATUS,
                motor_ids,
                values,
                "Hardware Error Status"))
        {
            return false;
        }

        error_status_values.resize(values.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            error_status_values[i] =
                static_cast<uint8_t>(values[i]);
        }

        return true;
    }

    /* ============================================================
     * 단위 변환
     * ============================================================ */

    double MotorStatus::positionRawToRadian(
        int32_t raw) const
    {
        return (static_cast<double>(raw) /
                (POSITION_TICKS_PER_REVOLUTION - 1.0)) *
                   (2.0 * PI) -
               PI;
    }

    double MotorStatus::velocityRawToRadianPerSecond(
        int32_t raw) const
    {
        const double rpm =
            static_cast<double>(raw) *
            VELOCITY_UNIT_RPM;

        return rpm * 2.0 * PI / 60.0;
    }

    double MotorStatus::voltageRawToVolt(
        uint16_t raw) const
    {
        return static_cast<double>(raw) *
               VOLTAGE_UNIT_VOLT;
    }

    double MotorStatus::feedbackRawToMilliAmpere(
        int16_t raw,
        const MotorModelInfo &model_info) const
    {
        if (model_info.feedback_type !=
            FeedbackType::PRESENT_CURRENT)
        {
            return 0.0;
        }

        return static_cast<double>(raw) *
               model_info.current_unit_mA;
    }

    double MotorStatus::feedbackRawToLoadPercent(
        int16_t raw,
        const MotorModelInfo &model_info) const
    {
        if (model_info.feedback_type !=
            FeedbackType::PRESENT_LOAD)
        {
            return 0.0;
        }

        return static_cast<double>(raw) * 0.1;
    }

    /* ============================================================
     * 레지스터 읽기
     * ============================================================ */

    bool MotorStatus::readRegister(
        uint8_t id,
        const ControlItem &item,
        uint32_t &value,
        const std::string &status_name)
    {
        uint8_t dxl_error = 0;
        int communication_result = COMM_TX_FAIL;

        switch (item.length)
        {
        case 1:
        {
            uint8_t raw = 0;

            communication_result =
                packet_handler_->read1ByteTxRx(
                    port_handler_,
                    id,
                    item.address,
                    &raw,
                    &dxl_error);

            value = raw;
            break;
        }

        case 2:
        {
            uint16_t raw = 0;

            communication_result =
                packet_handler_->read2ByteTxRx(
                    port_handler_,
                    id,
                    item.address,
                    &raw,
                    &dxl_error);

            value = raw;
            break;
        }

        case 4:
        {
            uint32_t raw = 0;

            communication_result =
                packet_handler_->read4ByteTxRx(
                    port_handler_,
                    id,
                    item.address,
                    &raw,
                    &dxl_error);

            value = raw;
            break;
        }

        default:
            RCLCPP_ERROR(
                logger_,
                "[readRegister] Unsupported register length: %u",
                item.length);

            return false;
        }

        if (communication_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(
                logger_,
                "[readRegister] Communication failed - %s, ID %u: %s (%d)",
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
                        "[readRegister] Protocol 2.0 alert - %s, ID %u: %s (%u). Continuing.",
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
                    "[readRegister] Dynamixel error - %s, ID %u: %s (%u)",
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

    bool MotorStatus::syncReadIndirectStatus(
        const ControlItem &item,
        const std::vector<uint8_t> &motor_ids,
        std::vector<MotorStatusData> &statuses)
    {
        if (motor_ids.empty())
        {
            return false;
        }

        dynamixel::GroupSyncRead sync_read(
            port_handler_,
            packet_handler_,
            item.address,
            item.length);

        for (const auto id : motor_ids)
        {
            if (!sync_read.addParam(id))
            {
                RCLCPP_ERROR(
                    logger_,
                    "[syncReadRegister] Failed to add ID %u",
                    id);

                return false;
            }
        }

        const int communication_result =
            sync_read.txRxPacket();

        if (communication_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(
                logger_,
                "[syncReadRegister] Communication failed: %s (%d)",
                packet_handler_->getTxRxResult(
                    communication_result),
                communication_result);

            return false;
        }

        for (size_t i = 0; i < motor_ids.size(); ++i)
        {
            const uint8_t id = motor_ids[i];

            if (!sync_read.isAvailable(
                    id,
                    item.address,
                    item.length))
            {
                RCLCPP_ERROR(
                    logger_,
                    "[syncReadRegister] Data unavailable, ID %u",
                    id);

                return false;
            }

            MotorStatusData &status = statuses[i];
            status.id = id;

            status.hardware_error_status =
                static_cast<uint8_t>(
                    sync_read.getData(
                        id,
                        item.address +
                            control_table::status_indirect::HARDWARE_ERROR_STATUS_OFFSET,
                        1));

            status.goal_position_raw =
                static_cast<int32_t>(
                    sync_read.getData(
                        id,
                        item.address +
                            control_table::status_indirect::GOAL_POSITION_OFFSET,
                        4));

            status.moving_status =
                static_cast<uint8_t>(
                    sync_read.getData(
                        id,
                        item.address +
                            control_table::status_indirect::MOVING_STATUS_OFFSET,
                        1));

            status.present_feedback_raw =
                static_cast<int16_t>(
                    static_cast<uint16_t>(
                        sync_read.getData(
                            id,
                            item.address +
                                control_table::status_indirect::PRESENT_FEEDBACK_OFFSET,
                            2)));

            status.present_velocity_raw =
                static_cast<int32_t>(
                    sync_read.getData(
                        id,
                        item.address +
                            control_table::status_indirect::PRESENT_VELOCITY_OFFSET,
                        4));

            status.present_position_raw =
                static_cast<int32_t>(
                    sync_read.getData(
                        id,
                        item.address +
                            control_table::status_indirect::PRESENT_POSITION_OFFSET,
                        4));

            status.input_voltage_raw =
                static_cast<uint16_t>(
                    sync_read.getData(
                        id,
                        item.address +
                            control_table::status_indirect::PRESENT_INPUT_VOLTAGE_OFFSET,
                        2));

            status.temperature_c =
                static_cast<uint8_t>(
                    sync_read.getData(
                        id,
                        item.address +
                            control_table::status_indirect::PRESENT_TEMPERATURE_OFFSET,
                        1));
        }

        sync_read.clearParam();

        return true;
    }

    bool MotorStatus::syncReadRegister(
        const ControlItem &item,
        const std::vector<uint8_t> &motor_ids,
        std::vector<uint32_t> &values,
        const std::string &status_name)
    {
        if (motor_ids.empty())
        {
            values.clear();
            return true;
        }

        dynamixel::GroupSyncRead sync_read(
            port_handler_,
            packet_handler_,
            item.address,
            item.length);

        for (const auto id : motor_ids)
        {
            if (!sync_read.addParam(id))
            {
                RCLCPP_ERROR(
                    logger_,
                    "[syncReadRegister] Failed to add %s for ID %u",
                    status_name.c_str(),
                    id);

                return false;
            }
        }

        const int communication_result =
            sync_read.txRxPacket();

        if (communication_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(
                logger_,
                "[syncReadRegister] Communication failed - %s: %s (%d)",
                status_name.c_str(),
                packet_handler_->getTxRxResult(
                    communication_result),
                communication_result);

            return false;
        }

        values.resize(motor_ids.size());

        for (size_t i = 0; i < motor_ids.size(); ++i)
        {
            const uint8_t id = motor_ids[i];

            if (!sync_read.isAvailable(
                    id,
                    item.address,
                    item.length))
            {
                RCLCPP_ERROR(
                    logger_,
                    "[syncReadRegister] Data unavailable - %s, ID %u",
                    status_name.c_str(),
                    id);

                return false;
            }

            values[i] =
                sync_read.getData(
                    id,
                    item.address,
                    item.length);
        }

        sync_read.clearParam();

        return true;
    }

} // namespace dynamixel_hardware_interface
