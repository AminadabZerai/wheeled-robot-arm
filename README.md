# wheeled-robot-arm

Wheeled robot with manipulator arm — ROS 2, Arduino, Raspberry Pi

A modular robotics platform combining a 4-wheel mecanum base with an articulated
manipulator arm. Built using Raspberry Pi, ROS 2, Arduino Nano 33 IoT, and custom
electronics. Designed for research, education, and portfolio development.

## Table of Contents

- [About](#about)
- [System Architecture](#system-architecture)
- [Project Roadmap](#project-roadmap)
- [Hardware](#hardware)
- [Software](#software)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

## About

This project aims to design and build a wheeled mobile robot equipped with a
manipulator arm. It combines mechanical design, electronics, embedded systems,
and high-level control using ROS 2.

The firmware follows a component-based architecture — each hardware subsystem
(IMU, encoders, motors, PID) is isolated into its own driver module with a
clean API, making the system testable, swappable, and maintainable.

## System Architecture
```
┌─────────────────────────────────────┐
│         Raspberry Pi (ROS 2)        │
│   Navigation · Planning · Comms     │
└──────────────┬──────────────────────┘
               │ Serial / USB
┌──────────────▼──────────────────────┐
│       Arduino Nano 33 IoT          │
│  Main controller — 100Hz loop      │
│                                     │
│  ┌─────────┐ ┌──────────────────┐  │
│  │ LSM6DS3 │ │   TCA9548A Mux   │  │
│  │  IMU    │ │   ┌──┐┌──┐┌──┐┌──┐ │
│  └─────────┘ │   │E1││E2││E3││E4│ │
│              │   └──┘└──┘└──┘└──┘ │
│              │   AS5600 Encoders  │  │
│              └──────────────────────┘│
│  ┌──────────────────────────────┐   │
│  │  TB6612FNG Motor Drivers ×2  │   │
│  │  4× DC Motors (7.4V LiPo)   │   │
│  └──────────────────────────────┘   │
└─────────────────────────────────────┘
```

**Key design decisions:**
- Single MCU (Arduino Nano 33 IoT) controls all 4 wheels instead of per-wheel MCUs
- TCA9548A I2C multiplexer allows 4× AS5600 encoders on the same bus (same address 0x36)
- IMU (LSM6DS3) shares the I2C bus for orientation sensing
- PID + feedforward velocity control runs at 50 Hz per motor
- MATLAB tooling for offline calibration and sensor characterization

## Project Roadmap

### ✅ Section 1: Motor, Encoder, and Odometry Setup
- [x] Power system setup (7.4V LiPo, buck converters)
- [x] Mechanical assembly and wiring validation
- [x] AS5600 encoder integration via TCA9548A I2C mux
- [x] TB6612FNG motor driver wiring (2× drivers, 4 motors)
- [x] PID + feedforward velocity control (tuned, proven baseline)
- [x] Basic serial command interface (`f,150` / `r,90`)

### 🔄 Section 1.5: Firmware Refactoring & Sensor Characterization *(current)*
- [x] IMU driver extraction (`lib/IMU/`)
- [x] LSM6DS3 calibration and characterization (PASS — see journal)
- [x] MATLAB real-time plotter, 3D visualizer, and analysis tooling
- [x] Development journal established (`docs/journal.md`)
- [ ] Fix sampling jitter (interrupt-driven reads)
- [ ] AS5600 driver refactoring to match IMU module pattern
- [ ] Motor/PID module extraction
- [ ] Config/pin definitions centralized (`include/config.h`)
- [ ] On-board complementary filter for orientation

### 🔜 Section 2: Motion Control & Odometry
- [ ] Mecanum kinematic model (wheel velocities ↔ robot twist)
- [ ] Odometry estimation from encoders + IMU fusion
- [ ] Serial protocol for ROS 2 bridge (velocity commands + odom feedback)

### 🔜 Section 3: Manipulator Arm Integration
- [ ] Arm kinematics and mechanical design
- [ ] Servo/stepper driver electronics
- [ ] Dedicated controller or shared bus for arm joints
- [ ] ROS 2 nodes for arm control and planning

### 🔜 Section 4: High-Level Integration
- [ ] Raspberry Pi ROS 2 orchestration
- [ ] Autonomous and manual control modes
- [ ] System-level testing and refinement

## Hardware

| Component | Role | Qty |
|-----------|------|-----|
| Arduino Nano 33 IoT | Main MCU (3.3V logic) | 1 |
| Raspberry Pi | High-level ROS 2 control | 1 |
| AS5600 | 12-bit magnetic encoder | 4 |
| TCA9548A | I2C multiplexer | 1 |
| TB6612FNG | Dual H-bridge motor driver | 2 |
| DC Motors | Mecanum wheels, 7.4V | 4 |
| LiPo Battery | 7.4V 2S | 1 |
| Buck Converters | Voltage regulation | as needed |
| 3D Printed Parts | Brackets, mounts, chassis | various |

## Software

**Firmware (Arduino)**
- Modular C/C++ drivers in `lib/` (IMU, AS5600, MotorWheel, PID)
- 100 Hz deterministic main loop
- PID + feedforward velocity control with anti-windup
- Machine-readable CSV telemetry output

**MATLAB Tooling**
- Real-time data acquisition with infinite streaming and CSV export
- 3D orientation visualization with complementary filter
- Post-capture statistical analysis (noise, PSD, bias, pass/fail)

**ROS 2 (planned)**
- Velocity command interface
- Odometry publisher
- Arm control nodes

## Repository Structure
```
WHEELED-ROBOT-ARM/
├── src/
│   └── main.cpp              # Main firmware
├── include/
│   └── config.h              # Pin definitions and constants
├── lib/
│   ├── IMU/                  # LSM6DS3 driver
│   ├── AS5600/               # Magnetic encoder driver
│   ├── MotorWheel/           # Motor control abstraction
│   ├── PID/                  # PID controller
│   └── MoveBase/             # Kinematics and motion
├── matlab/
│   ├── imu_calibration_and_data.m
│   ├── imu_3d_visualization.m
│   ├── imu_analysis.m
│   └── data/                 # CSV captures
├── docs/
│   └── journal.md                        # Development journal
    └── zerillio_procedures.md            # General Procedure to boot the raspberry pi 3
├── 3D Files/                             # CAD and printable parts
├── Datasheets/                           # Component datasheets
├── ros2_ws/                              # ROS 2 workspace (future)
├── platformio.ini
├── LICENSE
└── README.md
```

## Getting Started

### Prerequisites
- [PlatformIO](https://platformio.org/) (firmware build and upload)
- MATLAB R2020b+ with Signal Processing Toolbox (analysis scripts)
- Arduino Nano 33 IoT board support

### Flash the firmware
```bash
cd WHEELED-ROBOT-ARM
pio run -t upload
```

### Run MATLAB live plotter
1. Connect Arduino via USB
2. Update `port` in the script to match your COM port
3. Run `imu_calibration_and_data.m`
4. Close the figure window to stop — CSV saves automatically

## Documentation

See [`docs/journal.md`](./docs/journal.md) for detailed development logs including
sensor characterization results, architecture decisions, and test data.

Additional resources in [`Datasheets/`](./Datasheets) and
[`Roadmap and References/`](./Roadmap%20and%20References).

## Contributing

Contributions, ideas, and bug reports are welcome.
Please open an Issue or submit a Pull Request.

## License

This project is licensed under the MIT License. See [LICENSE](./LICENSE) for details.