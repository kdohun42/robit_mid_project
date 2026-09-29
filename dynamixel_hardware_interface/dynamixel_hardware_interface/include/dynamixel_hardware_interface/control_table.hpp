#pragma once

#include <array>
#include <cstdint>

namespace dynamixel_hardware_interface
{

    struct ControlItem
    {
        uint16_t address;
        uint8_t length;
    };
    //{주소, 길이(바이트)}

    namespace control_table
    {

        // EEPROM
        namespace eeprom
        {

            inline constexpr ControlItem MODEL_NUMBER{0, 2};
            inline constexpr ControlItem MODEL_INFORMATION{2, 4};
            inline constexpr ControlItem FIRMWARE_VERSION{6, 1};

            inline constexpr ControlItem ID{7, 1};
            inline constexpr ControlItem BAUD_RATE{8, 1};
            inline constexpr ControlItem RETURN_DELAY_TIME{9, 1};

            inline constexpr ControlItem DRIVE_MODE{10, 1};
            inline constexpr ControlItem OPERATING_MODE{11, 1};
            inline constexpr ControlItem SECONDARY_ID{12, 1};
            inline constexpr ControlItem PROTOCOL_TYPE{13, 1};

            inline constexpr ControlItem HOMING_OFFSET{20, 4};
            inline constexpr ControlItem MOVING_THRESHOLD{24, 4};

            inline constexpr ControlItem TEMPERATURE_LIMIT{31, 1};
            inline constexpr ControlItem MAX_VOLTAGE_LIMIT{32, 2};
            inline constexpr ControlItem MIN_VOLTAGE_LIMIT{34, 2};
            inline constexpr ControlItem PWM_LIMIT{36, 2};

            // MX-64랑 XH540에만 지원, 28은 없는 기능
            inline constexpr ControlItem CURRENT_LIMIT{38, 2};

            inline constexpr ControlItem ACCELERATION_LIMIT{40, 4};
            inline constexpr ControlItem VELOCITY_LIMIT{44, 4};
            inline constexpr ControlItem MAX_POSITION_LIMIT{48, 4};
            inline constexpr ControlItem MIN_POSITION_LIMIT{52, 4};

            // XH540에서 사용함
            inline constexpr ControlItem STARTUP_CONFIGURATION{60, 1};

            inline constexpr ControlItem SHUTDOWN{63, 1};

            // Indirect Address 1 : 원본 주소값(2바이트)을 저장하는 포인터 슬롯.
            // Indirect Address 2, 3, ...은 이 주소 + 2, + 4, ...로 이어짐.
            inline constexpr ControlItem INDIRECT_ADDRESS_1{168, 2};

        } // namespace eeprom

        // RAM
        namespace ram
        {

            inline constexpr ControlItem TORQUE_ENABLE{64, 1};
            inline constexpr ControlItem LED{65, 1};
            inline constexpr ControlItem STATUS_RETURN_LEVEL{68, 1};
            inline constexpr ControlItem REGISTERED_INSTRUCTION{69, 1};
            inline constexpr ControlItem HARDWARE_ERROR_STATUS{70, 1};

            inline constexpr ControlItem VELOCITY_I_GAIN{76, 2};
            inline constexpr ControlItem VELOCITY_P_GAIN{78, 2};

            inline constexpr ControlItem POSITION_D_GAIN{80, 2};
            inline constexpr ControlItem POSITION_I_GAIN{82, 2};
            inline constexpr ControlItem POSITION_P_GAIN{84, 2};

            inline constexpr ControlItem FEEDFORWARD_2ND_GAIN{88, 2};
            inline constexpr ControlItem FEEDFORWARD_1ST_GAIN{90, 2};

            inline constexpr ControlItem BUS_WATCHDOG{98, 1};

            inline constexpr ControlItem GOAL_PWM{100, 2};

            // MX-64랑 XH540에만 지원, 28은 없는 기능
            inline constexpr ControlItem GOAL_CURRENT{102, 2};

            inline constexpr ControlItem GOAL_VELOCITY{104, 4};
            inline constexpr ControlItem PROFILE_ACCELERATION{108, 4};
            inline constexpr ControlItem PROFILE_VELOCITY{112, 4};
            inline constexpr ControlItem GOAL_POSITION{116, 4};

            inline constexpr ControlItem REALTIME_TICK{120, 2};
            inline constexpr ControlItem MOVING{122, 1};
            inline constexpr ControlItem MOVING_STATUS{123, 1};
            inline constexpr ControlItem PRESENT_PWM{124, 2};

            // 모델따라 의미가 달라서 네이밍은 중립적으로
            // 28 : Present Load
            // 64, XH540 : Present Current
            inline constexpr ControlItem PRESENT_FEEDBACK{126, 2};

            inline constexpr ControlItem PRESENT_VELOCITY{128, 4};
            inline constexpr ControlItem PRESENT_POSITION{132, 4};
            inline constexpr ControlItem VELOCITY_TRAJECTORY{136, 4};
            inline constexpr ControlItem POSITION_TRAJECTORY{140, 4};
            inline constexpr ControlItem PRESENT_INPUT_VOLTAGE{144, 2};
            inline constexpr ControlItem PRESENT_TEMPERATURE{146, 1};

            // XH540에서 사용
            inline constexpr ControlItem BACKUP_READY{147, 1};

            // Indirect Data 1 : 대응하는 Indirect Address가 가리키는
            // 원본 주소의 값이 실시간으로 복사되는 자리(1바이트씩).
            // Indirect Data 2, 3, ...은 이 주소 + 1, + 2, ...로 이어짐.
            inline constexpr ControlItem INDIRECT_DATA_1{224, 1};

        } // namespace ram

        // 상태 일괄 읽기용 Indirect Address 배선표.
        // Indirect Address 1~19에 순서대로 이 원본 주소들을 써넣으면,
        // Indirect Data 1~19(19바이트)를 한 번만 읽어도 아래 8개
        // 레지스터 값을 전부 얻을 수 있다.
        namespace status_indirect
        {

            inline constexpr std::array<uint16_t, 19> SOURCE_BYTE_ADDRESSES{
                ram::HARDWARE_ERROR_STATUS.address,

                static_cast<uint16_t>(ram::GOAL_POSITION.address + 0),
                static_cast<uint16_t>(ram::GOAL_POSITION.address + 1),
                static_cast<uint16_t>(ram::GOAL_POSITION.address + 2),
                static_cast<uint16_t>(ram::GOAL_POSITION.address + 3),

                ram::MOVING_STATUS.address,

                static_cast<uint16_t>(ram::PRESENT_FEEDBACK.address + 0),
                static_cast<uint16_t>(ram::PRESENT_FEEDBACK.address + 1),

                static_cast<uint16_t>(ram::PRESENT_VELOCITY.address + 0),
                static_cast<uint16_t>(ram::PRESENT_VELOCITY.address + 1),
                static_cast<uint16_t>(ram::PRESENT_VELOCITY.address + 2),
                static_cast<uint16_t>(ram::PRESENT_VELOCITY.address + 3),

                static_cast<uint16_t>(ram::PRESENT_POSITION.address + 0),
                static_cast<uint16_t>(ram::PRESENT_POSITION.address + 1),
                static_cast<uint16_t>(ram::PRESENT_POSITION.address + 2),
                static_cast<uint16_t>(ram::PRESENT_POSITION.address + 3),

                static_cast<uint16_t>(ram::PRESENT_INPUT_VOLTAGE.address + 0),
                static_cast<uint16_t>(ram::PRESENT_INPUT_VOLTAGE.address + 1),

                ram::PRESENT_TEMPERATURE.address};

            // Indirect Data 블록(INDIRECT_DATA_1 기준) 안에서
            // 각 필드가 시작하는 위치(offset, 바이트 단위).
            inline constexpr uint16_t HARDWARE_ERROR_STATUS_OFFSET = 0;
            inline constexpr uint16_t GOAL_POSITION_OFFSET = 1;
            inline constexpr uint16_t MOVING_STATUS_OFFSET = 5;
            inline constexpr uint16_t PRESENT_FEEDBACK_OFFSET = 6;
            inline constexpr uint16_t PRESENT_VELOCITY_OFFSET = 8;
            inline constexpr uint16_t PRESENT_POSITION_OFFSET = 12;
            inline constexpr uint16_t PRESENT_INPUT_VOLTAGE_OFFSET = 16;
            inline constexpr uint16_t PRESENT_TEMPERATURE_OFFSET = 18;

            inline constexpr uint8_t TOTAL_LENGTH = 19;

        } // namespace status_indirect

        // Operating Mode 값
        namespace operating_mode
        {

            inline constexpr uint8_t CURRENT = 0;
            inline constexpr uint8_t VELOCITY = 1;
            inline constexpr uint8_t POSITION = 3;
            inline constexpr uint8_t EXTENDED_POSITION = 4;
            inline constexpr uint8_t CURRENT_BASED_POSITION = 5;
            inline constexpr uint8_t PWM = 16;

        } // namespace operating_mode

        namespace torque_enable
        {

            inline constexpr uint8_t OFF = 0;
            inline constexpr uint8_t ON = 1;

        } // namespace torque

        namespace led
        {

            inline constexpr uint8_t OFF = 0;
            inline constexpr uint8_t ON = 1;

        } // namespace led

        // Hardware Error Status 비트
        namespace hardware_error
        {

            inline constexpr uint8_t INPUT_VOLTAGE = 0x01;
            inline constexpr uint8_t OVERHEATING = 0x04;
            inline constexpr uint8_t MOTOR_ENCODER = 0x08;
            inline constexpr uint8_t ELECTRICAL_SHOCK = 0x10;
            inline constexpr uint8_t OVERLOAD = 0x20;

        } // namespace hardware_error

    } // namespace control_table
} // namespace dynamixel_hardware_interface