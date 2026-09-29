#pragma once

#include <cstdint>
#include <optional>

#include "dynamixel_hardware_interface/control_table.hpp"
#include "dynamixel_hardware_interface/motor_model.hpp"

namespace dynamixel_hardware_interface
{

    enum class MotorGroup
    {
        UPPER_BODY,
        LOWER_BODY,
        WAIST,
        UNKNOWN
    };

    struct MotorConfig
    {
        uint8_t id{0};

        // 상하체/허리 그룹으로 분류
        MotorGroup group{MotorGroup::UNKNOWN};

        // 자동 인식 결과와 비교할 예상 모델
        // config에 모델을 지정하지 않으면 std::nullopt
        std::optional<MotorModel> expected_model{std::nullopt};

        // 시작 시 Operating Mode
        uint8_t startup_mode{
            control_table::operating_mode::POSITION};

        // Current Mode motor uses this internal command instead of
        // external dynamixel_control position commands.
        int16_t internal_goal_current_raw{0};
    };

    // 그룹이 런타임 모드 변경을 지원하는지 확인
    inline bool supportsRuntimeModeSwitching(MotorGroup group)
    {
        return group == MotorGroup::UPPER_BODY;
    }

    // 항상 Position Mode로 사용하는 그룹인지 확인
    inline bool isFixedPositionGroup(MotorGroup group)
    {
        return group == MotorGroup::LOWER_BODY ||
               group == MotorGroup::WAIST;
    }

} // namespace dynamixel_hardware_interface
