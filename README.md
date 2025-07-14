# wheeled-robot-arm
Wheeled robot with manipulator arm - ROS 2, Arduino, Raspberry Pi project

A modular robotics platform combining a wheeled base with an articulated manipulator arm. Built using Raspberry Pi, ROS 2, Seeed XIAO microcontrollers, and custom electronics. Designed for research, education, and portfolio development.

## Table of Contents
- [About](#about)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Project Roadmap](#project-roadmap)
- [Hardware](#hardware)
- [Software](#software)
- [Getting Started](#getting-started)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

## About
This project aims to design and build a wheeled mobile robot equipped with a manipulator arm. It combines mechanical design, electronics, embedded systems, and high-level control using ROS 2.

The robot is divided into clear development sections, making it an ideal platform to demonstrate skills in mechatronics, embedded software development, and system integration.

## Features
- Wheeled platform with mecanum or differential drive
- Distributed motor controllers using Seeed XIAO or Teensy
- Local encoder reading and motor control per wheel
- High-level command interface via Raspberry Pi (ROS 2)
- ROS 2 nodes for movement, localization, and arm control
- Modular manipulator arm with inverse kinematics
- Detailed documentation and schematics

## System Architecture
> [Insert system block diagram or architecture drawing here]

## Project Roadmap

### ✅ Section 1: Motor, Encoder, and Odometry Setup
- Power system setup
- Mechanical assembly
- Wiring validation
- Arduino Nano 33 IoT firmware development
- Basic odometry tests

### 🔜 Section 2: Distributed Motor Intelligence
- Install XIAO/Teensy microcontroller per motor
- Local encoder reading on each MCU
- Motor PWM/Direction control on XIAO
- Serial/ROS 2 interface for velocity commands
- Odometry publishing back to Pi

### 🔜 Section 3: Manipulator Arm Integration
- Arm kinematics and mechanical design
- Servo/stepper driver electronics
- XIAO or dedicated controller for the arm
- ROS 2 nodes for arm control and planning

### 🔜 Section 4: High-Level Integration and Testing
- Raspberry Pi ROS 2 orchestration
- Autonomous and manual control modes
- System-level testing and refinement

## Hardware
- Raspberry Pi (high-level ROS 2 control)
- Seeed XIAO or Teensy microcontrollers (per-motor controllers)
- Arduino Nano 33 IoT (initial testing)
- Motor driver boards (TB6612FNG, BTS7960)
- AS5600 magnetic encoders
- DC motors with mecanum or standard wheels
- 3D printed brackets and mounts
- Power supply and buck converters

## Software
- Arduino/XIAO firmware for local motor control
- Raspberry Pi ROS 2 nodes for command and telemetry
- Serial communication protocols
- ROS 2 launch files and configurations

## Getting Started

> Work in progress. This section will include:

- Wiring diagrams
- Bill of Materials (BOM)
- Setup instructions
- Microcontroller firmware upload guide
- ROS 2 node installation

## Documentation

See the [docs/](./docs) folder for detailed schematics, block diagrams, and design notes.

Planned additions:

- Full wiring schematics
- Control flow diagrams
- Kinematic calculations

## Contributing

Contributions, ideas, and bug reports are welcome. Please open an Issue or submit a Pull Request.

## License

This project is licensed under the MIT License. See the LICENSE file for details.
