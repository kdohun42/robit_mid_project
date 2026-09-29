#include "dynamixel_hardware_interface/dynamixel_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

namespace dynamixel_hardware_interface
{

    namespace
    {

        constexpr uint8_t PROTOCOL2_ALERT_ERROR = 0x80;

        // Return Delay Time register unit is 2us per tick
        // (e.g. raw value 250 => 500us delay).
        constexpr uint8_t RETURN_DELAY_TIME_UNIT_US = 2;
        constexpr uint8_t TARGET_RETURN_DELAY_TIME_US = 20;
        constexpr uint8_t TARGET_RETURN_DELAY_TIME_RAW =
            TARGET_RETURN_DELAY_TIME_US / RETURN_DELAY_TIME_UNIT_US;

        bool isIgnorableHardwareErrorStatus(
            uint8_t hardware_error_status)
        {
            return hardware_error_status ==
                   control_table::hardware_error::INPUT_VOLTAGE;
        }

        bool hasBlockingHardwareErrorStatus(
            uint8_t hardware_error_status)
        {
            return hardware_error_status != 0 &&
                   !isIgnorableHardwareErrorStatus(
                       hardware_error_status);
        }

    } // namespace

    DynamixelHardwareInterface::DynamixelHardwareInterface(
        const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode(
              "dynamixel_hardware_interface",
              options)
    {
        declareParameters();

        RCLCPP_INFO(get_logger(), "Dynamixel hardware interface node created");
    }

    DynamixelHardwareInterface::~DynamixelHardwareInterface()
    {
        command_enabled_ = false;
        closePort();
    }

    DynamixelHardwareInterface::CallbackReturn
    DynamixelHardwareInterface::on_configure(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(
            get_logger(),
            "Configuring Dynamixel hardware interface");

        command_enabled_ = false;

        // 1. 파라미터/모터 목록 읽기
        if (!loadParameters())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to load parameters");

            return CallbackReturn::FAILURE;
        }

        // 2. Dynamixel 포트 열기
        if (!openPort())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to open Dynamixel port");

            return CallbackReturn::FAILURE;
        }

        // 3. 읽기/쓰기 객체
        motor_setting_ =
            std::make_unique<MotorSetting>(
                port_handler_,
                packet_handler_,
                get_logger());

        motor_status_ =
            std::make_unique<MotorStatus>(
                port_handler_,
                packet_handler_,
                get_logger());

        // 4. 설정된 모든 모터 Ping -> 모델 감지
        if (!detectMotors())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to detect required motors");

            motor_status_.reset();
            motor_setting_.reset();

            motors_.clear();
            closePort();

            return CallbackReturn::FAILURE;
        }

        // 5. Torque OFF /시작 Operating Mode 설정
        if (!initializeMotors())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to initialize motors");

            // 초기화 도중 일부 모터 상태가 바뀌었을 수 있으므로
            // 가능한 모든 모터의 Torque를 다시 끈다.
            disableAllTorque();

            motor_status_.reset();
            motor_setting_.reset();

            motors_.clear();
            closePort();

            return CallbackReturn::FAILURE;
        }

        dynamixel_control_subscription_ =
            create_subscription<
                dynamixel_hardware_msgs::msg::DynamixelControlMsgs>(
                control_topic_,
                rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                std::bind(
                    &DynamixelHardwareInterface::
                        dynamixelControlCallback,
                    this,
                    std::placeholders::_1));

        motor_status_publisher_ =
            create_publisher<
                dynamixel_hardware_msgs::msg::CurrentMotorStatus>(
                status_topic_,
                rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        // 6. 컨트롤 타이머 생성 (상태 읽기도 이 사이클 안에서 같이 수행)
        const auto control_timer_period =
            std::chrono::duration<double>(
                1.0 / control_rate_hz_);

        control_timer_ =
            create_wall_timer(
                std::chrono::duration_cast<
                    std::chrono::nanoseconds>(
                    control_timer_period),
                std::bind(
                    &DynamixelHardwareInterface::
                        controlTimerCallback,
                    this));

        // configure 완료 상태는 INACTIVE이므로
        // on_activate() 전까지 타이머를 실행하지 않는다.
        control_timer_->cancel();

        RCLCPP_INFO(
            get_logger(),
            "Dynamixel hardware interface configured: "
            "%zu motors ready with torque disabled. "
            "dynamixel_control commands are matched by motor array order.",
            motors_.size());

        return CallbackReturn::SUCCESS;
    }

    DynamixelHardwareInterface::CallbackReturn
    DynamixelHardwareInterface::on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(
            get_logger(),
            "Activating Dynamixel hardware interface");

        command_enabled_ = false;

        // 1. 모든 모터 상태 확인
        // 2. Hardware Error 확인
        // 3. 현재 모드에 맞는 안전 명령 설정
        if (!prepareMotorsForActivation())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to prepare motors for activation");

            return CallbackReturn::FAILURE;
        }

        // 모든 모터가 준비된 뒤 Torque ON
        if (!enablePreparedMotors())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to enable motor torque");

            // 일부 모터만 켜졌을 수 있으므로 전부 다시 끔
            disableAllTorque();

            return CallbackReturn::FAILURE;
        }

        command_enabled_ = true;

        if (motor_status_publisher_)
        {
            motor_status_publisher_->on_activate();
        }

        if (control_timer_)
        {
            control_timer_->reset();
        }

        RCLCPP_INFO(
            get_logger(),
            "Dynamixel hardware interface activated");

        return CallbackReturn::SUCCESS;
    }

    bool DynamixelHardwareInterface::detectMotors()
    {
        if (port_handler_ == nullptr ||
            packet_handler_ == nullptr)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Dynamixel handlers are not initialized");

            return false;
        }

        if (motors_.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "No Dynamixel motors are configured");

            return false;
        }

        bool all_connected = true;

        for (auto &motor : motors_)
        {
            motor.connected = false;
            motor.model_info = nullptr;
            motor.detected_model = MotorModel::UNKNOWN;
            motor.detected_model_number = 0;

            uint16_t model_number = 0;
            uint8_t dxl_error = 0;

            const int communication_result =
                packet_handler_->ping(
                    port_handler_,
                    motor.config.id,
                    &model_number,
                    &dxl_error);

            if (communication_result != COMM_SUCCESS)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to ping ID %u: %s",
                    motor.config.id,
                    packet_handler_->getTxRxResult(
                        communication_result));

                all_connected = false;
                continue;
            }

            if (dxl_error != 0)
            {
                if (dxl_error == PROTOCOL2_ALERT_ERROR)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "ID %u reported Protocol 2.0 alert during ping. "
                        "Continuing detection and deferring detailed "
                        "Hardware Error Status handling to activation.",
                        motor.config.id);
                }
                else
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Dynamixel error while pinging ID %u: %s",
                        motor.config.id,
                        packet_handler_->getRxPacketError(
                            dxl_error));

                    all_connected = false;
                    continue;
                }
            }

            motor.detected_model_number =
                model_number;

            motor.detected_model =
                motorModelFromNumber(
                    model_number);

            if (motor.detected_model == MotorModel::UNKNOWN)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Unsupported Dynamixel model: ID %u, model number %u",
                    motor.config.id,
                    model_number);

                all_connected = false;
                continue;
            }

            motor.model_info =
                &getMotorModelInfo(
                    motor.detected_model);
            motor.connected = true;

            RCLCPP_INFO(
                get_logger(),
                "Detected motor ID %u: %s, model number %u",
                motor.config.id,
                motor.model_info->name,
                motor.detected_model_number);
        }

        if (!all_connected)
        {
            RCLCPP_ERROR(
                get_logger(),
                "One or more required motors were not detected");

            return false;
        }

        return true;
    }

    DynamixelHardwareInterface::CallbackReturn
    DynamixelHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(
            get_logger(),
            "Deactivating Dynamixel hardware interface");

        command_enabled_ = false;

        if (control_timer_)
        {
            control_timer_->cancel();
        }

        if (motor_status_publisher_)
        {
            motor_status_publisher_->on_deactivate();
        }

        bool success = true;

        for (auto &motor : motors_)
        {
            if (!motor.connected ||
                motor.model_info == nullptr)
            {
                continue;
            }

            switch (motor.current_mode)
            {
            case control_table::operating_mode::POSITION:
            {
                int32_t present_position_raw = 0;

                if (!motor_status_->getCurrentPosition(
                        motor.config.id,
                        present_position_raw))
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to read present position for ID %u",
                        motor.config.id);

                    success = false;
                    continue;
                }

                if (!motor_setting_->setGoalPosition(
                        motor.config.id,
                        static_cast<uint32_t>(
                            present_position_raw)))
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to hold current position for ID %u",
                        motor.config.id);

                    success = false;
                    continue;
                }

                motor.last_goal_position_raw =
                    present_position_raw;

                break;
            }

            case control_table::operating_mode::CURRENT:
            {
                if (!motor_setting_->setGoalCurrent(
                        motor.config.id,
                        0,
                        *motor.model_info))
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to set zero current for ID %u",
                        motor.config.id);

                    success = false;
                }

                break;
            }

            default:
                RCLCPP_WARN(
                    get_logger(),
                    "No safe stop command defined for mode %u, ID %u",
                    motor.current_mode,
                    motor.config.id);

                break;
            }
        }

        if (!disableAllTorque())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to disable torque for one or more motors");

            success = false;
        }

        if (!success)
        {
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(
            get_logger(),
            "Dynamixel hardware interface deactivated");

        return CallbackReturn::SUCCESS;
    }

    DynamixelHardwareInterface::CallbackReturn
    DynamixelHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(
            get_logger(),
            "Cleaning up Dynamixel hardware interface");

        command_enabled_ = false;

        bool success = true;

        dynamixel_control_subscription_.reset();
        motor_status_publisher_.reset();

        {
            std::scoped_lock<std::mutex> lock(control_msg_mutex_);
            latest_control_msg_.reset();
            has_control_msg_ = false;
            last_control_msg_time_ =
                rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }

        // 1. 상태 타이머 정지 및 제거
        if (control_timer_)
        {
            control_timer_->cancel();
            control_timer_.reset();
        }

 

        // 2. Torque OFF 재시도
        if (!disableAllTorque())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to disable torque during cleanup");

            success = false;
        }

        // 3. 읽기/쓰기 객체 제거
        motor_status_.reset();
        motor_setting_.reset();

        // 4. 모터 런타임 정보 초기화
        motors_.clear();

        // 5. 포트 닫기
        closePort();

        if (!success)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Dynamixel hardware cleanup completed with errors");

            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(
            get_logger(),
            "Dynamixel hardware interface cleaned up");

        return CallbackReturn::SUCCESS;
    }

    DynamixelHardwareInterface::CallbackReturn
    DynamixelHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(
            get_logger(),
            "Shutting down Dynamixel hardware interface");

        command_enabled_ = false;

        // 1. 상태 타이머 정지 및 제거
        if (control_timer_)
        {
            control_timer_->cancel();
            control_timer_.reset();
        }

 

        // Keep current motor torque state on process shutdown by request.
        // 2. 객체 제거
        motor_status_.reset();
        motor_setting_.reset();

        {
            std::scoped_lock<std::mutex> lock(control_msg_mutex_);
            latest_control_msg_.reset();
            has_control_msg_ = false;
            last_control_msg_time_ =
                rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }

        motors_.clear();

        // 3. 포트 닫기
        closePort();

        RCLCPP_INFO(
            get_logger(),
            "Dynamixel hardware interface shut down");

        return CallbackReturn::SUCCESS;
    }

    void DynamixelHardwareInterface::declareParameters()
    {
        // TODO(syu): Replace legacy group ID arrays with per-motor config
        // parameters such as motor_ids + motor_<id>.* without breaking
        // existing IRC launch/config integration.
        declare_parameter<std::string>(
            "device_name",
            "/dev/ttyUSB-U2D2");

        declare_parameter<int>(
            "baud_rate",
            1000000);

        declare_parameter<double>(
            "protocol_version",
            2.0);

        declare_parameter<double>(
            "control_rate_hz",
            100.0);
        declare_parameter<double>(
            "control_timeout_sec",
            0.5);

        declare_parameter<std::string>(
            "control_topic",
            "dynamixel_control");

        declare_parameter<std::string>(
            "status_topic",
            "motor_status");

        declare_parameter<int>(
            "upper_body_startup_mode",
            control_table::operating_mode::POSITION);

        declare_parameter<int>(
            "lower_body_startup_mode",
            control_table::operating_mode::POSITION);

        declare_parameter<int>(
            "waist_startup_mode",
            control_table::operating_mode::POSITION);

        declare_parameter<std::vector<int64_t>>(
            "upper_body_ids",
            std::vector<int64_t>{});

        declare_parameter<std::vector<int64_t>>(
            "lower_body_ids",
            std::vector<int64_t>{});

        declare_parameter<std::vector<int64_t>>(
            "waist_ids",
            std::vector<int64_t>{});
    }

    bool DynamixelHardwareInterface::loadParameters()
    {
        std::vector<int64_t> upper_body_ids;
        std::vector<int64_t> lower_body_ids;
        std::vector<int64_t> waist_ids;
        int upper_body_startup_mode =
            control_table::operating_mode::POSITION;
        int lower_body_startup_mode =
            control_table::operating_mode::POSITION;
        int waist_startup_mode =
            control_table::operating_mode::POSITION;

        if (!get_parameter("device_name", device_name_) ||
            !get_parameter("baud_rate", baud_rate_) ||
            !get_parameter(
                "protocol_version",
                protocol_version_) ||
            !get_parameter("control_rate_hz", control_rate_hz_) ||
            !get_parameter(
                "control_timeout_sec",
                control_timeout_sec_) ||
            !get_parameter(
                "upper_body_startup_mode",
                upper_body_startup_mode) ||
            !get_parameter(
                "lower_body_startup_mode",
                lower_body_startup_mode) ||
            !get_parameter(
                "waist_startup_mode",
                waist_startup_mode) ||
            !get_parameter(
                "upper_body_ids",
                upper_body_ids) ||
            !get_parameter(
                "lower_body_ids",
                lower_body_ids) ||
            !get_parameter(
                "waist_ids",
                waist_ids))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to load one or more parameters");

            return false;
        }

        if (!get_parameter("control_topic", control_topic_) ||
            !get_parameter("status_topic", status_topic_))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to load topic parameters");

            return false;
        }

        if (control_topic_.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "control_topic must not be empty");

            return false;
        }

        if (status_topic_.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "status_topic must not be empty");

            return false;
        }

        if (device_name_.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "device_name must not be empty");

            return false;
        }

        if (baud_rate_ <= 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "baud_rate must be greater than zero");

            return false;
        }

        if (protocol_version_ <= 0.0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "protocol_version must be greater than zero");

            return false;
        }

        if (control_rate_hz_ <= 0.0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "control_rate_hz must be greater than zero");

            return false;
        }

        if (control_timeout_sec_ < 0.0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "control_timeout_sec must not be negative");

            return false;
        }

        auto validate_startup_mode =
            [this](int mode, const char *name)
        {
            if (mode < 0 || mode > 255)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "%s must be in [0, 255]",
                    name);
                return false;
            }
            return true;
        };

        if (!validate_startup_mode(
                upper_body_startup_mode,
                "upper_body_startup_mode") ||
            !validate_startup_mode(
                lower_body_startup_mode,
                "lower_body_startup_mode") ||
            !validate_startup_mode(
                waist_startup_mode,
                "waist_startup_mode"))
        {
            return false;
        }

        motors_.clear();

        auto add_motors =
            [this](
                const std::vector<int64_t> &ids,
                MotorGroup group,
                uint8_t startup_mode)
        {
            for (const int64_t id_value : ids)
            {
                if (id_value < 0 || id_value > 252)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Invalid Dynamixel ID: %ld",
                        static_cast<long>(id_value));

                    return false;
                }

                const uint8_t id =
                    static_cast<uint8_t>(id_value);

                for (const auto &motor : motors_)
                {
                    if (motor.config.id == id)
                    {
                        RCLCPP_ERROR(
                            get_logger(),
                            "Duplicated Dynamixel ID: %u",
                            id);

                        return false;
                    }
                }

                MotorDevice motor;

                motor.config.id = id;
                motor.config.group = group;
                motor.config.startup_mode =
                    startup_mode;

                motor.current_mode =
                    startup_mode;

                motors_.push_back(motor);
            }

            return true;
        };

        if (!add_motors(
                upper_body_ids,
                MotorGroup::UPPER_BODY,
                static_cast<uint8_t>(
                    upper_body_startup_mode)) ||
            !add_motors(
                lower_body_ids,
                MotorGroup::LOWER_BODY,
                static_cast<uint8_t>(
                    lower_body_startup_mode)) ||
            !add_motors(
                waist_ids,
                MotorGroup::WAIST,
                static_cast<uint8_t>(
                    waist_startup_mode)))
        {
            motors_.clear();
            return false;
        }

        if (motors_.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "No Dynamixel motors are configured");

            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded parameters: device=%s, baud=%d, protocol=%.1f, "
            "control_rate=%.1f Hz, motors=%zu",
            device_name_.c_str(),
            baud_rate_,
            protocol_version_,
            control_rate_hz_,
            motors_.size());

        return true;
    }

    bool DynamixelHardwareInterface::openPort()
    {
        if (port_handler_ != nullptr)
        {
            RCLCPP_WARN(
                get_logger(),
                "Dynamixel port handler is already initialized");

            return true;
        }

        port_handler_ =
            dynamixel::PortHandler::getPortHandler(
                device_name_.c_str());

        packet_handler_ =
            dynamixel::PacketHandler::getPacketHandler(
                protocol_version_);

        if (port_handler_ == nullptr ||
            packet_handler_ == nullptr)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to create Dynamixel SDK handlers");

            port_handler_ = nullptr;
            packet_handler_ = nullptr;

            return false;
        }

        if (!port_handler_->openPort())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to open Dynamixel port: %s",
                device_name_.c_str());

            port_handler_ = nullptr;
            packet_handler_ = nullptr;

            return false;
        }

        if (!port_handler_->setBaudRate(baud_rate_))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to set Dynamixel baud rate: %d",
                baud_rate_);

            port_handler_->closePort();

            port_handler_ = nullptr;
            packet_handler_ = nullptr;

            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "Opened Dynamixel port: %s, baud rate: %d, protocol: %.1f",
            device_name_.c_str(),
            baud_rate_,
            protocol_version_);

        return true;
    }

    void DynamixelHardwareInterface::closePort()
    {
        if (port_handler_ != nullptr)
        {
            port_handler_->closePort();

            RCLCPP_INFO(
                get_logger(),
                "Closed Dynamixel port");
        }

        port_handler_ = nullptr;
        packet_handler_ = nullptr;
    }

    bool DynamixelHardwareInterface::initializeMotors()
    {
        if (motor_setting_ == nullptr)
        {
            RCLCPP_ERROR(
                get_logger(),
                "MotorSetting is not initialized");

            return false;
        }

        if (motors_.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "No motors to initialize");

            return false;
        }

        for (auto &motor : motors_)
        {
            if (!initializeMotor(motor))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to initialize motor ID %u",
                    motor.config.id);

                disableAllTorque();
                return false;
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "Initialized %zu Dynamixel motors",
            motors_.size());

        return true;
    }

    bool DynamixelHardwareInterface::initializeMotor(
        MotorDevice &motor)
    {
        if (!motor.connected ||
            motor.model_info == nullptr)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor ID %u is not connected or has no model information",
                motor.config.id);

            return false;
        }

        motor.prepared_for_activation = false;
        motor.torque_enabled = false;

        if (!motor_setting_->setTorque(
                motor.config.id,
                false))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to disable torque for ID %u",
                motor.config.id);

            return false;
        }

        if (!motor_setting_->setStatusIndirectAddressMapping(
                motor.config.id))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to configure status Indirect Address mapping for ID %u",
                motor.config.id);

            return false;
        }

        uint8_t return_delay_time_raw = 0;

        if (!motor_status_->getReturnDelayTime(
                motor.config.id,
                return_delay_time_raw))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to read Return Delay Time for ID %u",
                motor.config.id);

            return false;
        }

        if (return_delay_time_raw != TARGET_RETURN_DELAY_TIME_RAW &&
            !motor_setting_->setReturnDelayTime(
                motor.config.id,
                TARGET_RETURN_DELAY_TIME_RAW))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to set Return Delay Time for ID %u",
                motor.config.id);

            return false;
        }

        const uint8_t startup_mode =
            motor.config.startup_mode;

        if (!motor_setting_->setOperatingMode(
                motor.config.id,
                startup_mode,
                *motor.model_info))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to set operating mode for ID %u",
                motor.config.id);

            return false;
        }

        motor.current_mode = startup_mode;

        RCLCPP_INFO(
            get_logger(),
            "Initialized ID %u in startup mode %u with torque disabled",
            motor.config.id,
            startup_mode);

        return true;
    }

    bool DynamixelHardwareInterface::prepareMotorsForActivation()
    {
        if (motor_setting_ == nullptr ||
            motor_status_ == nullptr)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor handlers are not initialized");

            return false;
        }

        for (auto &motor : motors_)
        {
            motor.prepared_for_activation = false;

            if (!motor.connected ||
                motor.model_info == nullptr)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Motor ID %u is not connected or has no model information",
                    motor.config.id);

                return false;
            }

            if (!motor_status_->readMotorStatus(
                    motor.config.id,
                    *motor.model_info,
                    motor.status))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to read status from ID %u",
                    motor.config.id);

                return false;
            }

            if (hasBlockingHardwareErrorStatus(
                    motor.status.hardware_error_status))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Refusing to activate ID %u: hardware error 0x%02X",
                    motor.config.id,
                    motor.status.hardware_error_status);

                return false;
            }

            if (isIgnorableHardwareErrorStatus(
                    motor.status.hardware_error_status))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Ignoring input voltage hardware error for ID %u "
                    "(Hardware Error Status=0x%02X) by request",
                    motor.config.id,
                    motor.status.hardware_error_status);
            }

            if (!prepareMotorCommand(motor))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to prepare safe command for ID %u",
                    motor.config.id);

                return false;
            }

            motor.prepared_for_activation = true;
        }

        return true;
    }

    bool DynamixelHardwareInterface::prepareMotorCommand(
        MotorDevice &motor)
    {
        switch (motor.current_mode)
        {
        case control_table::operating_mode::POSITION:
        {
            const uint32_t present_position =
                static_cast<uint32_t>(
                    motor.status.present_position_raw);

            if (!motor_setting_->setGoalPosition(
                    motor.config.id,
                    present_position))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to set safe goal position for ID %u",
                    motor.config.id);

                return false;
            }

            motor.last_goal_position_raw =
                motor.status.present_position_raw;

            return true;
        }

        case control_table::operating_mode::CURRENT:
        {
            if (!motor_setting_->setGoalCurrent(
                    motor.config.id,
                    0,
                    *motor.model_info))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to set zero goal current for ID %u",
                    motor.config.id);

                return false;
            }

            return true;
        }

        default:
            RCLCPP_ERROR(
                get_logger(),
                "Unsupported activation mode %u for ID %u",
                motor.current_mode,
                motor.config.id);

            return false;
        }
    }

    bool DynamixelHardwareInterface::enablePreparedMotors()
    {
        for (auto &motor : motors_)
        {
            if (!motor.prepared_for_activation)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Motor ID %u was not prepared for activation",
                    motor.config.id);

                return false;
            }
        }

        for (auto &motor : motors_)
        {
            if (!motor_setting_->setTorque(
                    motor.config.id,
                    true))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to enable torque for ID %u",
                    motor.config.id);

                disableAllTorque();
                return false;
            }

            motor.torque_enabled = true;
        }

        return true;
    }

    bool DynamixelHardwareInterface::disableAllTorque()
    {
        if (motor_setting_ == nullptr)
        {
            return true;
        }

        bool success = true;

        for (auto &motor : motors_)
        {
            motor.prepared_for_activation = false;

            if (!motor.connected)
            {
                motor.torque_enabled = false;
                continue;
            }

            if (!motor_setting_->setTorque(
                    motor.config.id,
                    false))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to disable torque for ID %u",
                    motor.config.id);

                success = false;
                continue;
            }

            motor.torque_enabled = false;
        }

        return success;
    }

    void DynamixelHardwareInterface::readMotorStatuses()
    {
        if (!command_enabled_)
        {
            return;
        }

        if (motor_status_ == nullptr)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "MotorStatus is not initialized");

            return;
        }

        std::vector<uint8_t> motor_ids;
        std::vector<const MotorModelInfo *> model_infos;

        motor_ids.reserve(motors_.size());
        model_infos.reserve(motors_.size());

        for (const auto &motor : motors_)
        {
            if (!motor.connected ||
                motor.model_info == nullptr)
            {
                continue;
            }

            motor_ids.push_back(
                motor.config.id);

            model_infos.push_back(
                motor.model_info);
        }

        if (motor_ids.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "No connected motors available for status reading");

            return;
        }

        std::vector<MotorStatusData> statuses;

        if (!motor_status_->readMotorStatusSync(
                motor_ids,
                model_infos,
                statuses))
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Failed to read Dynamixel motor statuses");

            return;
        }

        if (statuses.size() != motor_ids.size())
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Status result size mismatch: ids=%zu, statuses=%zu",
                motor_ids.size(),
                statuses.size());

            return;
        }

        for (size_t i = 0; i < statuses.size(); ++i)
        {
            const uint8_t id =
                motor_ids[i];

            for (auto &motor : motors_)
            {
                if (motor.config.id != id)
                {
                    continue;
                }

                motor.status =
                    statuses[i];

                break;
            }
        }

        if (motor_status_publisher_ == nullptr ||
            !motor_status_publisher_->is_activated())
        {
            return;
        }

        dynamixel_hardware_msgs::msg::CurrentMotorStatus msg;
        msg.position.reserve(statuses.size());
        msg.goal_position.reserve(statuses.size());
        msg.velocity.reserve(statuses.size());
        msg.temperature.reserve(statuses.size());
        msg.torque.reserve(statuses.size());
        msg.input_voltage.reserve(statuses.size());
        msg.moving_status.reserve(statuses.size());
        msg.error_status.reserve(statuses.size());

        for (const auto &status : statuses)
        {
            msg.position.push_back(
                status.present_position_rad);
            msg.goal_position.push_back(
                static_cast<uint32_t>(
                    status.goal_position_raw));
            msg.velocity.push_back(
                static_cast<uint8_t>(
                    std::max<int32_t>(
                        0,
                        std::min<int32_t>(
                            255,
                            status.present_velocity_raw))));
            msg.temperature.push_back(
                status.temperature_c);

            // Compatibility note:
            // MX-28 stores Present Load raw at address 126.
            // MX-64/XH540 stores Present Current raw at address 126.
            msg.torque.push_back(
                static_cast<uint16_t>(
                    status.present_feedback_raw));
            msg.input_voltage.push_back(
                status.input_voltage_raw);
            msg.moving_status.push_back(
                status.moving_status);
            msg.error_status.push_back(
                status.hardware_error_status);
        }

        motor_status_publisher_->publish(msg);
    }

    void DynamixelHardwareInterface::dynamixelControlCallback(
        const dynamixel_hardware_msgs::msg::
            DynamixelControlMsgs::SharedPtr msg)
    {
        if (msg == nullptr)
        {
            return;
        }

        // /dynamixel_control follows the configured motor array order.
        if (msg->motor_control.size() != motors_.size())
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "motor_control size mismatch: received=%zu, required=%zu",
                msg->motor_control.size(),
                motors_.size());

            return;
        }

        {
            std::scoped_lock<std::mutex> lock(control_msg_mutex_);
            latest_control_msg_ = msg;
            has_control_msg_ = true;
            last_control_msg_time_ = now();
        }
    }

    void DynamixelHardwareInterface::controlTimerCallback()
    {
        if (!command_enabled_)
        {
            return;
        }

        if (motor_setting_ == nullptr)
        {
            return;
        }

        // 매 컨트롤 사이클마다 상태를 먼저 읽어(1패킷) 최신 값을
        // hasBlockingHardwareErrorStatus 판단 및 퍼블리시에 반영한다.
        
        readMotorStatuses();

        dynamixel_hardware_msgs::msg::DynamixelControlMsgs::SharedPtr
            latest_control_msg;
        rclcpp::Time last_control_msg_time(
            0,
            0,
            get_clock()->get_clock_type());
        bool has_control_msg = false;

        {
            std::scoped_lock<std::mutex> lock(control_msg_mutex_);
            latest_control_msg = latest_control_msg_;
            last_control_msg_time = last_control_msg_time_;
            has_control_msg = has_control_msg_;
        }

        const bool control_timed_out =
            !has_control_msg ||
            latest_control_msg == nullptr ||
            (control_timeout_sec_ > 0.0 &&
             (now() - last_control_msg_time).seconds() >
                 control_timeout_sec_);

        if (!control_timed_out &&
            latest_control_msg->motor_control.size() != motors_.size())
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Stored control message size mismatch");

            return;
        }

        std::vector<uint8_t> position_ids;
        std::vector<uint8_t> current_ids;
        std::vector<double> goal_positions;
        std::vector<double> profile_velocities;
        std::vector<double> profile_accelerations;
        std::vector<int16_t> goal_currents_raw;
        std::vector<const MotorModelInfo *> current_model_infos;

        position_ids.reserve(motors_.size());
        current_ids.reserve(motors_.size());
        goal_positions.reserve(motors_.size());
        profile_velocities.reserve(motors_.size());
        profile_accelerations.reserve(motors_.size());
        goal_currents_raw.reserve(motors_.size());
        current_model_infos.reserve(motors_.size());

        for (size_t i = 0; i < motors_.size(); ++i)
        {
            auto &motor = motors_[i];

            if (!motor.connected ||
                !motor.torque_enabled ||
                motor.model_info == nullptr ||
                hasBlockingHardwareErrorStatus(
                    motor.status.hardware_error_status))
            {
                continue;
            }

            if (motor.current_mode ==
                control_table::operating_mode::POSITION)
            {
                if (control_timed_out)
                {
                    continue;
                }

                const auto &command =
                    latest_control_msg->motor_control[i];

                position_ids.push_back(
                    motor.config.id);

                goal_positions.push_back(
                    command.goal_position);

                profile_velocities.push_back(
                    command.profile_velocity);

                profile_accelerations.push_back(
                    command.profile_acceleration);

                motor.last_goal_position_raw =
                    motor.status.goal_position_raw;
                continue;
            }

            if (motor.current_mode !=
                control_table::operating_mode::CURRENT)
            {
                continue;
            }

            if (!motor.model_info->supports_current_mode)
            {
                continue;
            }

            current_ids.push_back(
                motor.config.id);
            current_model_infos.push_back(
                motor.model_info);

            const int16_t goal_current_raw =
                control_timed_out
                    ? 0
                    : motor.config.internal_goal_current_raw;

            goal_currents_raw.push_back(
                goal_current_raw);
            motor.last_goal_current_raw =
                goal_current_raw;
        }

        if (control_timed_out)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Control command timed out; stopping new position commands");
        }

        if (!position_ids.empty() &&
            !motor_setting_->syncWritePositionProfile(
                position_ids,
                goal_positions,
                profile_velocities,
                profile_accelerations))
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Failed to write position commands");
        }

        if (!current_ids.empty() &&
            !motor_setting_->setGoalCurrentSync(
                current_ids,
                goal_currents_raw,
                current_model_infos))
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Failed to write current commands");
        }
    }
} // namespace dynamixel_hardware_interface

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<
        dynamixel_hardware_interface::DynamixelHardwareInterface>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
