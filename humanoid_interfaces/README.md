# humanoid interfaces directory

# humanoid_interfaces

ROS 2 custom message definitions used throughout the RoboCup Humanoid League software stack.

## Overview

`humanoid_interfaces` provides all custom ROS 2 message definitions shared between:

* Motion / Inverse Kinematics
* Vision
* Localization
* Team Communication (UDP)
* Sensor Processing
* Motion Control

These interfaces are used to exchange robot state, walking commands, localization information, vision targets, and sensor measurements between subsystems.

---

## Package Structure

| Category                   | Messages                                                                                                        |
| -------------------------- | --------------------------------------------------------------------------------------------------------------- |
| Inverse Kinematics         | `IkAngleSimMsg`, `IkComMsg`, `IkCoordMsg`, `IkEndMsg`, `IkLTCMsg`, `IkPatternMsg`, `Master2IkMsg`, `Tune2IkMsg` |
| Sensor                     | `ImuMsg`, `ZmpMsg`                                                                                              |
| Inter-Module Communication | `Master2vision`, `Master2localization`, `Master2udp`, `Udp2master`                                              |
| Utility                    | `MotionOperator`, `Pidtuning`                                                                                   |

---

## Main Interfaces

### Walking Control

#### Master2IkMsg

Walking command sent from the master node to the inverse kinematics module.

```text
Master
 └── Master2IkMsg
       ├── x_length
       ├── y_length
       ├── yaw
       └── one_step_flag
```

Contains:

* Forward stride length
* Side stride length
* Turning angle
* One-step walking command

---

### IMU Feedback

#### ImuMsg

Provides complete inertial sensor information.

Includes:

* Euler angles (roll, pitch, yaw)
* Quaternion orientation
* Angular velocity
* Linear acceleration
* Integrated velocity
* Integrated position

---

### ZMP Feedback

#### ZmpMsg

Zero Moment Point information used for balance control.

Contains:

* Left foot ZMP
* Right foot ZMP
* Combined ZMP
* Foot contact state

---

### Vision Interface

#### Master2vision

Command message from master to vision.

Contains:

* Vision task mode
* Camera pan angle
* Camera tilt angle

---

### Localization Interface

#### Master2localization

Navigation targets and localization parameters.

Contains:

* Goal position
* Target position
* Position tolerance
* Attack / defense reference positions

---

### Team Communication

#### Master2udp

Robot state broadcast to teammates.

Contains:

* Robot ID
* Robot state
* Localization result
* Ball information
* Team information

#### Udp2master

Robot state received from teammates.

Contains:

* Teammate localization
* Ball observations
* Robot behavior state

---

### Motion Control

#### MotionOperator

Motion playback interface.

```text
motion_num
motion_end
```

Used for predefined motion execution.

---

### PID Tuning

#### Pidtuning

Generic PID parameter message.

```text
kp
ki
kd
```

Used for runtime controller tuning.

---

## Build

```bash
cd ~/colcon_ws

colcon build --packages-select humanoid_interfaces

source install/setup.bash
```

## Dependencies

* ROS 2 Humble (or later)
* std_msgs
* geometry_msgs
* rosidl_default_generators
* rosidl_default_runtime

```

---

## Maintainer

RoboCup Humanoid League Software Team

