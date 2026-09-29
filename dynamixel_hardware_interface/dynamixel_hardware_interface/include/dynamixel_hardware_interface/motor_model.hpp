#pragma once

#include <string>
#include <cstdint>
#include "dynamixel_hardware_interface/control_table.hpp"

namespace dynamixel_hardware_interface
{

    enum class MotorModel
    {
        MX28,
        MX64,
        MX106,
        XH540_W270,
        UNKNOWN
    };

    enum class FeedbackType
    {
        PRESENT_LOAD,
        PRESENT_CURRENT,
        NONE
    };

    struct MotorModelInfo
    {
        MotorModel model;
        uint16_t model_number;
        const char *name;

        bool supports_current_mode;
        bool supports_velocity_mode;
        bool supports_position_mode;
        bool supports_extended_position_mode;
        bool supports_current_based_position_mode;
        bool supports_pwm_mode;
        bool supports_current_limit;

        FeedbackType feedback_type;

        double current_unit_mA;
    };

    inline constexpr MotorModelInfo MX28_INFO{
        MotorModel::MX28,
        30,
        "MX-28",
        false,
        true,
        true,
        true,
        false,
        true,

        false, // Current Limit 미지원

        FeedbackType::PRESENT_LOAD,

        0.0};

    inline constexpr MotorModelInfo MX64_INFO{
        MotorModel::MX64,
        311,
        "MX-64",

        true,
        true,
        true,
        true,
        true,
        true,

        true,

        FeedbackType::PRESENT_CURRENT,

        3.36};

    inline constexpr MotorModelInfo MX106_INFO{
        MotorModel::MX106,
        321,
        "MX-106",

        true,
        true,
        true,
        true,
        true,
        true,

        true,

        FeedbackType::PRESENT_CURRENT,

        3.36};

    inline constexpr MotorModelInfo XH540_W270_INFO{
        MotorModel::XH540_W270,
        1100,
        "XH540-W270",

        true,
        true,
        true,
        true,
        true,
        true,

        true,

        FeedbackType::PRESENT_CURRENT,

        2.69};

    inline constexpr MotorModelInfo UNKNOWN_INFO{
        MotorModel::UNKNOWN,
        0,
        "UNKNOWN",

        false,
        false,
        false,
        false,
        false,
        false,

        false,

        FeedbackType::NONE,

        0.0};

    inline MotorModel motorModelFromString(
        const std::string &model_name)
    {
        if (model_name == "mx28")
        {
            return MotorModel::MX28;
        }

        if (model_name == "mx64")
        {
            return MotorModel::MX64;
        }

        if (model_name == "mx106")
        {
            return MotorModel::MX106;
        }

        if (model_name == "xh540_w270")
        {
            return MotorModel::XH540_W270;
        }

        return MotorModel::UNKNOWN;
    }

    inline MotorModel motorModelFromNumber(uint16_t model_number)
    {
        switch (model_number)
        {
        case MX28_INFO.model_number:
            return MotorModel::MX28;

        case MX64_INFO.model_number:
            return MotorModel::MX64;

        case MX106_INFO.model_number:
            return MotorModel::MX106;

        case XH540_W270_INFO.model_number:
            return MotorModel::XH540_W270;

        default:
            return MotorModel::UNKNOWN;
        }
    }

    inline const MotorModelInfo &getMotorModelInfo(
        MotorModel model)
    {
        switch (model)
        {
        case MotorModel::MX28:
            return MX28_INFO;

        case MotorModel::MX64:
            return MX64_INFO;

        case MotorModel::MX106:
            return MX106_INFO;

        case MotorModel::XH540_W270:
            return XH540_W270_INFO;

        default:
            return UNKNOWN_INFO;
        }
    }

    inline bool supportsOperatingMode(
        const MotorModelInfo &model_info,
        uint8_t mode)
    {
        switch (mode)
        {
        case control_table::operating_mode::CURRENT:
            return model_info.supports_current_mode;

        case control_table::operating_mode::VELOCITY:
            return model_info.supports_velocity_mode;

        case control_table::operating_mode::POSITION:
            return model_info.supports_position_mode;

        case control_table::operating_mode::EXTENDED_POSITION:
            return model_info.supports_extended_position_mode;

        case control_table::operating_mode::CURRENT_BASED_POSITION:
            return model_info.supports_current_based_position_mode;

        case control_table::operating_mode::PWM:
            return model_info.supports_pwm_mode;

        default:
            return false;
        }
    }

} // namespace dynamixel_hardware_interface
