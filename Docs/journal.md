# Development Journal

---

## 2026-04-01 — IMU Calibration Testing & Architecture Refactoring

### Overview
Two major milestones today: (1) completed the first full IMU characterization session
for the LSM6DS3, with MATLAB tooling for real-time acquisition, 3D visualization, and
statistical analysis; (2) began the architectural refactoring of the firmware from a
monolithic "god-file" into a modular, component-based system.

---

### Part 1: IMU Calibration & Characterization

#### Test Setup
- Sensor: LSM6DS3 (I2C @ 100 kHz)
- MCU: Arduino Nano 33 IoT (Serial @ 115200 baud)
- Sampling: 100 Hz target (10 ms interval)
- Calibration: 10-second stationary window at startup
- Capture: ~100 seconds, 10,030 samples
- Condition: Sensor flat on desk, stationary throughout

#### Results Summary

**Accelerometer**

| Metric | X | Y | Z |
|--------|---|---|---|
| Mean | +0.0000 g | -0.0001 g | +1.0003 g |
| Std | 0.0005 g | 0.0005 g | 0.0006 g |
| Noise Density | 0.0001 g/√Hz | 0.0000 g/√Hz | 0.0001 g/√Hz |

- Magnitude: 1.0003 g mean, 0.0006 g std (ideal = 1.0000 g)
- Verdict: **PASS** — Bias negligible, magnitude stable

**Gyroscope**

| Metric | X | Y | Z |
|--------|---|---|---|
| Bias | +0.005 °/s | -0.006 °/s | -0.010 °/s |
| Std | 0.107 °/s | 0.096 °/s | 0.089 °/s |
| Noise Density | 0.0107 (°/s)/√Hz | 0.0096 (°/s)/√Hz | 0.0089 (°/s)/√Hz |
| 60s Drift | 0.31° | 0.38° | 0.63° |

- Verdict: **PASS** — All bias values well under 1 °/s

**Pass/Fail Checklist**

| Check | Threshold | Result |
|-------|-----------|--------|
| Accel XY bias | < 0.05 g | **PASS** |
| Accel Z offset from 1g | < 0.05 g | **PASS** |
| Accel magnitude | ~1.000 g | **PASS** (1.0003 g) |
| Gyro bias all axes | < 1 °/s | **PASS** |

#### Observations & Concerns

1. **Sampling Jitter (Critical)**
   dt std = 24.8 ms, jitter = 204 ms peak, 1,164 missed samples (11.6%).
   Root cause: `Serial.print` blocking and/or I2C read latency in the main loop.
   This is the #1 issue to fix before motor integration — inconsistent dt corrupts
   gyro integration and will cause drift in orientation estimates.
   - Reading 14 bytes (6 Accel, 6 Gyro, 2 Temp) plus I2C overhead takes approximately 1.5ms to 2ms per cycle at 100 kHz. When combined with Serial.print (which takes ~3–5ms for a long CSV string at 115200 baud), you are consuming nearly 70% of your 10ms window just on I2C and Serial comms.
   Possible Action: move to Wire.setClock(400000); to reclaim overhead for the PID math.

2. **Noise Distribution**
   - Acc X and Gyr X: Very tight, narrow Gaussian — lowest noise axes.
   - Acc Y and Gyr Y: Clean Gaussian, slightly wider — normal.
   - Acc Z: Wider with slight asymmetry — expected for Z-axis MEMS structure.
   - Gyr Z: Ragged, non-ideal shape — may indicate vibration or mounting issue.
     Monitor this once mounted on the chassis.

3. **Power Spectral Density**
   - Accelerometer: Low-frequency energy at 3–5 Hz (environmental vibration).
     Visible as slow "breathing" envelope in the magnitude stability plot.
   - Gyroscope: Rising energy toward 50 Hz — likely mains interference (Italy = 50 Hz).
     Action: keep IMU wiring short and routed away from motor power lines.

4. **Magnitude Stability**
   Accel magnitude holds steady at 1.0003 ± 0.0006 g over the full 100s capture.
   Translates to ~±0.05° tilt uncertainty — more than sufficient for robot orientation.

#### Key Numbers for Future Filter Tuning

These noise density values become the Q/R covariance entries when implementing
a Kalman or complementary filter:

| Parameter           | X      | Y      | Z      | Unit      |
|---------------------|--------|--------|--------|-----------|
| Accel noise density | 0.0001 | 0.0000 | 0.0001 | g/√Hz     |
| Gyro noise density  | 0.0107 | 0.0096 | 0.0089 | (°/s)/√Hz |
| Gyro bias           | +0.005 | -0.006 | -0.010 | °/s |

---

### Part 2: Architecture Refactoring

#### The Problem: "God-File" Spaghetti

The previous `main.cpp` contained everything — I2C bus management, register-level
bit manipulation, calibration math, PID control for 4 motors, encoder reading,
serial telemetry, and command parsing — all in a single ~500-line file. The code
was tightly coupled: changing the serial output format could accidentally break
I2C timing. Adding a new sensor meant copy-pasting dozens of lines, doubling the
surface area for bugs.

#### The Solution: Component-Based Architecture

We are transitioning from *imperative spaghetti* (a list of instructions for the CPU)
to *component-based architecture* (a collection of services for the robot).

**Phase 1 — Decoupling the Hardware (Driver Layer)**

Extracted LSM6DS3 logic into `imu.h` / `imu.cpp`, creating an abstraction layer.
`main.cpp` no longer needs to know that gyroscope data starts at register `0x22` —
it calls `lsm6ds3_read_gyro()`. The sensor's identity (address, offsets, status)
is packaged into a struct (`lsm6ds3_t`), which is the foundation of object-oriented
thinking in a C environment.

-By passing the sensor struct as a pointer to every function (e.g., lsm6ds3_read_accel(&imu, ...)), we avoided "Hardcoding" the sensor. This makes the driver Reentrant. If you decide to add a second IMU for safety, you simply declare lsm6ds3_t imu2; and use the same functions.

We hit a "language identity crisis" where the C compiler couldn't read Arduino's
C++ libraries. Solved by renaming to `.cpp` and wrapping with `extern "C"` — keeping
the cleanliness of C function interfaces while using C++ hardware libraries.

**Phase 2 — Implementing a Contract (The API)**

By defining strict function prototypes in the header file, `main.cpp` becomes a
*client* that only cares *what* the functions do (init, read, calibrate), while
`imu.cpp` is the *provider* that handles *how*. If we swap the LSM6DS3 for an
MPU-6050 tomorrow, only the provider changes — `main.cpp` stays untouched because
the contract is the same.

**Phase 3 — Deterministic Execution (The Heartbeat)**

Refactored from "run as fast as possible" to a 10 ms deterministic loop using
`millis()`. Moved serial output from chatty labeled text to machine-readable CSV,
separating the firmware's role (data source) from MATLAB's role (data analyst).

#### Module Roadmap

| Module | Files | Status |
|--------|-------|--------|
| IMU Driver | `lib/IMU/imu.h`, `imu.cpp` | ✅ Complete |
| AS5600 Encoder | `lib/AS5600/as5600.h`, `as5600.c` | 🔄 Next — needs same refactor |
| Motor Control | `lib/MotorWheel/` | 📋 Planned |
| PID Controller | `lib/PID/` | 📋 Planned |
| Move Base | `lib/MoveBase/` | 📋 Planned |
| Config / Pins | `include/config.h` | 🔄 Started |
| Main State Machine | `src/main.cpp` | 🔄 In progress |

#### Impact

This refactor reduced `main.cpp` complexity by ~60%, moved hardware-specific
"magic numbers" into isolated drivers, and established the 100 Hz deterministic
heartbeat required for stable PID control.

- Technical Debt Paydown: > Before today, the "Cost of Change" was high—adding a feature required a full-file audit.
After today, we have Separation of Concerns. The IMU is a "Black Box" that provides a stream of floats. This isolation is what will allow us to scale to 4-wheel independent PID control without the code becoming unmanageable.

---

### Files Added This Commit
```
docs/journal.md                              — This file
matlab/imu_calibration_and_data.m            — Live plotter with CSV export
matlab/imu_3d_visualization.m                — Real-time 3D orientation viewer
matlab/imu_analysis.m                        — Post-capture statistical analysis
matlab/data/imu_data_2026-04-01_22-10-42.csv — Raw calibration capture
```

### Next Steps

- [ ] Fix sampling jitter — move to interrupt-driven IMU reads or reduce Serial blocking
- [ ] Refactor AS5600 encoder driver to match IMU module pattern
- [ ] Run IMU vibration test with motors on to characterize noise under load
- [ ] Implement on-board complementary filter (Arduino side)
- [ ] Extract motor control into `MotorWheel` module
- [ ] Extract PID into reusable `PID` module (per-motor instances)