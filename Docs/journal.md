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


--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## 2026-04-02 — I2C Multiplexing, Encoder Integrity & Hardware POST Implementation

### Overview
Three major threads today: (1) implemented a modular Power-On Self-Test (POST)
framework for hardware safety validation; (2) resolved I2C address collisions using
the TCA9548A multiplexer, including a critical bitmask bug; and (3) advanced the
AS5600 encoder driver toward PID-ready telemetry. The robot now performs a "medical
exam" before entering the control loop and has a clean, validated hardware backbone.

---

### Part 1: Hardware Integrity & POST (Power-On Self-Test)

#### The Strategy: "Safety First" Bootloader

To prevent mechanical runaway on startup, we implemented a modular `post.cpp`
library. Before entering the main control loop, the robot performs a structured
hardware health check — if any sensor reports a sub-optimal condition, the system
enters an **SOS Blink** halt state with a specific diagnostic message rather than
attempting to run.

#### Key Implementations

**Magnetic Field Validation**
Created `post_as5600_health()` to monitor the Status register (`0x0B`) and the
Automatic Gain Control register (`0x1A`). The system distinguishes between a missing
magnet and an air-gap that is too wide, surfacing specific failure strings such as:
`[FAIL] Enc Left-Front - NO MAGNET`.

**Bus Presence Checks**
Before any driver initialisation, the POST verifies both the IMU and MUX are
acknowledging on the main I2C line.

**POST Results Summary**

| Component | Check | Threshold | Result |
|-----------|-------|-----------|--------|
| LSM6DS3 | WHO_AM_I | 0x69 | **PASS** |
| TCA9548A | I2C Probe | Ack @ 0x70 | **PASS** |
| AS5600 | Magnet Detected | MD Bit == 1 | **PASS** |
| AS5600 | Signal Strength | AGC < 200 | **PASS** |

---

### Part 2: I2C Multiplexing & Communication Logic

#### The Problem: Address Collision & "Ghost" Readings

All four AS5600 encoders share a fixed I2C address (`0x36`). The TCA9548A
multiplexer was introduced to isolate each encoder on its own channel. On first
integration, all four channels returned `65535` (2¹⁶ − 1). This was identified as
**"Floating High"** behaviour — the MCU was reading pull-up resistors because the
MUX gate was not actually opening.

#### The Fix: Bitmask Addressing

The TCA9548A control register expects a **bitmask byte**, not a channel index.
Writing a raw integer `0` disables all channels rather than selecting channel 0.

- **Incorrect:** `Wire.write(0);` — disables all channels
- **Correct:** `Wire.write(1 << MUX_CH_LF);` — opens only the target channel

This single-line correction eliminated all ghost readings.

---

### Part 3: MATLAB Integration & Telemetry Issues

#### Refactoring the Visualisation

Updated the MATLAB live-stream script to a **3-subplot layout** (Accelerometer,
Gyroscope, Encoder). The internal buffer was formalised to a strict **8-column
schema**: `[Time, Acc_X, Acc_Y, Acc_Z, Gyr_X, Gyr_Y, Gyr_Z, Encoder_Angle]`.

#### Current Blocker: Stream Stalling

Despite the Arduino transmitting data, MATLAB is not rendering the live stream.

**Root Cause (Suspected):** Data framing mismatch. The Arduino `loop()` was
emitting `raw_angle` on a separate `println`, producing a 6-value line followed by
a 1-value line. MATLAB's `readline()` sees an inconsistent column count and stalls.

**Fix (Pending):** Consolidate all 7 sensor values into a single
`Serial.print(...)` string terminated by one `\n`. This is the same principle as
the CSV framing fix from yesterday's session — one sample, one line, always.

---

### Module Status Roadmap

| Module | Files | Status |
|--------|-------|--------|
| POST / System Health | `lib/POST/post.cpp` | ✅ Functional |
| IMU Driver | `lib/IMU/imu.cpp` | ✅ Stable |
| AS5600 Driver | `lib/AS5600/as5600.cpp` | 🔄 Hardware Test OK |
| MATLAB Tooling | `matlab/robot_live_stream.m` | ⚠️ Stalled (Framing Error) |

---

### Next Steps

- [x] **Finalise CSV Framing** — consolidate `Serial.print` in `main.cpp` to restore
      MATLAB live plotting; one line per sample, 8 columns, single `\n` terminator
- [x] **Rollover Math** — implement 12-bit → 32-bit "infinite rotation" accumulation
      logic in `as5600_update()` to handle multi-turn tracking without wrap-around jumps
- [x] **Angular Velocity** — derive rad/s from the encoder delta (Δangle / Δt) for
      use as the PID D-term input
- [ ] **Vibration Characterisation** — run IMU noise test with motors powered to
      measure the real-world noise floor under load (flagged from yesterday)

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## 2026-04-03 — AS5600 Encoder Driver Completion, Toolchain Debugging & MATLAB Telemetry Suite

### Overview
Three major threads today: (1) diagnosed and resolved a PlatformIO library identity
conflict that was silently blocking the AS5600 driver from compiling; (2) completed
the full encoder driver implementation — infinite rotation tracking, MUX gating,
and velocity derivation — bringing the sensing layer to a synchronized, PID-ready
state; (3) designed and delivered a fully separated MATLAB telemetry and analysis
suite with four independent scripts covering IMU and encoder pipelines. The robot
now streams 8 channels of calibrated sensor data at 100 Hz with graceful session
management and clean CSV export.

---

### Part 1: PlatformIO Compilation Failure — Root Cause & Resolution

#### The Problem: Library Identity Conflict

On first build attempt, the compiler rejected every reference to `as5600_t` and
`as5600_init()` with the following error:

```
src\main.cpp:14:1: error: 'as5600_t' does not name a type; did you mean 'AS5600L'?
```

The suggested alternative `AS5600L` was the diagnostic key — PlatformIO had located
a community AS5600 library (robtillaart/AS5600) and was resolving all encoder
symbols against it instead of the custom driver. The custom header was being
silently ignored entirely.

#### Root Causes Identified

Two independent configuration errors in `platformio.ini` combined to produce the
failure:

**1. `lib_extra_dirs` exposed the entire Arduino library ecosystem**

```ini
lib_extra_dirs =
    C:/Users/Amine/Documents/Arduino/libraries
```

This gave PlatformIO's dependency scanner access to every installed Arduino library,
including the conflicting community AS5600 package.

**2. `lib_ldf_mode = deep` enabled aggressive symbol resolution**

With deep mode active, PlatformIO traverses all reachable directories looking for
any file that can satisfy an unresolved symbol. Combined with `lib_extra_dirs`,
this guaranteed the external library would be found and preferred over the local
driver — no warning, no error, just the wrong library silently winning.

#### The Fix

Stripped `platformio.ini` to the minimum viable configuration:

```ini
[env:nano33iot]
platform = atmelsam
board = nano_33_iot
framework = arduino
monitor_speed = 115200

build_flags =
    -Iinclude
```

External dependencies are now declared explicitly via `lib_deps` only, preventing
any implicit global library resolution. Build succeeded immediately on the next
attempt.

**Architectural Rule Established:** Never use `lib_extra_dirs` pointing to the
Arduino global library folder. This couples the build environment to machine-local
state and creates non-reproducible builds. Any dependency needed must appear in
`lib_deps` by name and version.

---

### Part 2: AS5600 Driver — Implementation & Hardening

#### A. The "Infinite Rotation" Engine — Rollover Math

**The Challenge:** The AS5600 is a 12-bit absolute encoder — its raw output resets
from 4095 back to 0 every 360°. A naive reading of this value would cause the PID
controller to perceive a massive instantaneous jump during every wheel revolution,
corrupting both the position error term and the derivative.

**The Fix:** A shortest-path displacement algorithm operating on signed 16-bit
arithmetic. Each control cycle computes the signed jump between the current and
previous raw angle, then applies a boundary correction:

```cpp
int16_t jump = sensor->last_raw_angle - previous_raw_angle;
if      (jump >  2048) jump -= 4096;  // Crossed 0-mark backwards
else if (jump < -2048) jump += 4096;  // Crossed 0-mark forwards
sensor->total_ticks += jump;
sensor->radians = (float)sensor->total_ticks * RAW_TO_RAD;
```

This converts the 12-bit "face angle" into a 32-bit `total_ticks` accumulator,
giving the robot an unbounded understanding of cumulative angular displacement.
The `RAW_TO_RAD` constant (`2π / 4096`) converts ticks directly to radians.

**Result:** The encoder now tracks infinite multi-turn rotation without sawtooth
artifacts. The sign of `total_ticks` naturally encodes rotation direction, so no
additional direction pin is required.

#### B. MUX Gatekeeper — I2C Routing

All four AS5600 encoders share a fixed I2C address (`0x36`). The TCA9548A
multiplexer isolates each encoder on its own channel. The `select_mux_channel()`
helper was finalized and encapsulated inside `read_regs()`, making channel
selection completely transparent to `main.cpp`:

```cpp
static void select_mux_channel(uint8_t mux_channel) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << mux_channel);  // Bitmask, not index
    Wire.endTransmission();
}
```

The critical detail is the bitmask — a raw index of `0` disables all channels
rather than selecting channel 0. This was the root cause of the "Ghost Readings"
(`65535`) encountered in the previous session. The abstraction prevents this class
of bug from ever surfacing in higher-level code.

#### C. Physical Quantity Derivation — Velocity

Velocity is derived by temporal differentiation of the radians field, synchronised
to the control interval:

```cpp
void as5600_get_velocity(as5600_t *sensor) {
    float prev_radians   = sensor->radians;
    as5600_update(sensor);
    float dt             = (float)CONTROL_INTERVAL_MS / 1000.0f;
    sensor->delta_rad    = sensor->radians - prev_radians;
    sensor->angular_velocity = sensor->delta_rad / dt;
}
```

`delta_rad` was added as a first-class field on `as5600_t`. Logging the per-step
displacement directly avoids downstream numerical differencing errors when replaying
CSV data offline — particularly important for EKF process model tuning where
floating-point accumulation errors in post-processing can distort Q matrix fits.

Because velocity is derived from the signed `jump` logic, negative values are
reported automatically for backward rotation with no additional direction logic.

#### D. Code Hardening — Compiler Warnings Resolved

Two warnings surfaced on the clean build and were addressed:

**`buffer[]` may be used uninitialized** — In `as5600_get_angle()`, the read buffer
was declared without initialisation. If `Wire.available()` returned false during
a failed I2C transfer, the angle assembly would operate on garbage values. Fixed:

```cpp
uint8_t buffer[2] = {0, 0};
```

A failed read now produces a zero-delta in `total_ticks` rather than a random
position jump — deterministic fault behaviour.

**`write_reg` defined but not used** — Retained intentionally. This function will
be used when the `AS5600_PREFERED_CONFIG` register write is implemented during
POST initialisation (hysteresis, power mode, slow filter settings).

**Final Memory Footprint After Full Integration**

| Resource | Used | Available | Utilisation |
|----------|------|-----------|-------------|
| RAM | 4208 B | 32768 B | 12.8% |
| Flash | 26344 B | 262144 B | 10.0% |

Substantial headroom remains for motor control, PID instances, and the EKF state
estimator.

---

### Part 3: MATLAB Telemetry Suite — Full Separation

#### Architecture Decision: Four Independent Scripts

The combined 8-channel stream format from the previous session was identified as
an architectural problem for ongoing development:

- IMU calibration sessions require the robot stationary — encoder data is noise
- Encoder characterisation (slip tests, velocity sweeps) does not need IMU
- Combined analysis scripts make filter tuning harder to isolate per sensor
- Scaling to 4 encoders would make a combined format unmanageable

The suite was therefore divided into four independent, self-contained scripts:

| Script | Serial Columns | CSV Columns | Purpose |
|--------|---------------|-------------|---------|
| `imu_stream.m` | 6 | 7 | IMU live acquisition + export |
| `encoder_stream.m` | 2 | 3 | Encoder live acquisition + export |
| `imu_analysis.m` | — | 7 | Statistics, PSD, histograms, noise density |
| `encoder_analysis.m` | — | 3 | Kinematics, rollover health, velocity noise |

#### Graceful Stop — `try/finally` Pattern

A recurring failure mode with live stream scripts is data loss on Ctrl+C, where
execution never reaches the save block. The acquisition loop was wrapped in a
`try/finally` block ensuring the CSV save always executes regardless of exit cause
— figure close, Ctrl+C, or runtime error:

```matlab
finally
    clear device;
    if i > 0
        % save CSV with timestamp filename
    else
        fprintf('No data collected — nothing saved.\n');
    end
end
```

The `i > 0` guard prevents writing an empty file if the session is aborted before
any samples arrive.

#### Encoder Analysis — Diagnostics Beyond the IMU Script

Two encoder-specific diagnostics were added that have no IMU equivalent:

**Rollover Health Check** — Diffs the `EncRad` column and flags any step exceeding
`π` radians as a potential rollover artifact from `as5600_update()`. Zero detected
jumps confirms the boundary correction logic is operating correctly end-to-end.

**Dual Y-axis Velocity Plot** — Angular velocity is displayed simultaneously in
rad/s (left axis) and RPM (right axis) using `yyaxis`, removing the need for
mental unit conversion during test review sessions.

---

### Part 4: EKF Architecture — Mecanum Kinematic Implications

A design review was held on encoder data requirements ahead of EKF implementation,
with specific attention to how the mecanum drive geometry changes the architecture.

#### Key Architectural Conclusions

**Mecanum introduces a third independently controllable DOF.** Unlike differential
drive where lateral velocity is zero by constraint, mecanum requires `vy` as a
proper state:

```
State vector:  x = [px, py, θ, vx, vy, ω]
```

The dead reckoning update must apply the full 2D rotation matrix to transform
body-frame velocities into world-frame displacements:

```
px_new = px + (vx·cosθ − vy·sinθ) · dt
py_new = py + (vx·sinθ + vy·cosθ) · dt
θ_new  = θ + ω · dt
```

**Forward kinematics mixing matrix** (standard 45° roller layout):

```
vx = ( v_LF + v_RF + v_LR + v_RR) × (r/4)
vy = (-v_LF + v_RF + v_LR - v_RR) × (r/4)
ω  = (-v_LF + v_RF - v_LR + v_RR) × (r / 4(Lx+Ly))
```

**Lateral slip dominates encoder uncertainty.** Mecanum rollers slip laterally by
design — `σ_vy > σ_vx` is expected and must be characterised separately from
forward noise. A multiplicative noise model is more appropriate than additive for
the Q matrix:

```
σ_vy = k_y · |vy| + σ_0
```

Where `σ_0` is the quantisation floor from the static encoder test and `k_y` is
fit from a dedicated strafe calibration run.

**The IMU gyroscope is more valuable on mecanum than on differential drive.**
Wheel slip corrupts encoder-derived `ω` directly. Fusing gyro `ω` with encoder `ω`
in the EKF measurement update produces heading estimates that are robust to roller
slip — an advantage that does not exist on differential drive where encoder
odometry is inherently more reliable.

#### Calibration Runs Required Before EKF Tuning

| Test | Parameters Extracted |
|------|---------------------|
| Pure forward, fixed speeds | `r`, `σ_vx`, scale error |
| Pure strafe, fixed speeds | Lateral slip coefficient, `σ_vy` |
| Pure rotation in place | `Lx + Ly`, `σ_ω` |
| Diagonal motion | Cross-coupling validation |
| Surface comparison | Slip model per surface type |

---

### Module Status Roadmap

| Module | Files | Status |
|--------|-------|--------|
| POST / System Health | `lib/POST/post.cpp` | ✅ Stable |
| IMU Driver | `lib/IMU/imu.cpp` | ✅ Stable |
| AS5600 Encoder Driver | `lib/AS5600/as5600.cpp` | ✅ Complete — streaming live |
| MATLAB IMU Stream | `matlab/imu_stream.m` | ✅ Complete |
| MATLAB Encoder Stream | `matlab/encoder_stream.m` | ✅ Complete |
| MATLAB IMU Analysis | `matlab/imu_analysis.m` | ✅ Complete |
| MATLAB Encoder Analysis | `matlab/encoder_analysis.m` | ✅ Complete |
| Motor Control | `lib/MotorWheel/` | 📋 Planned |
| PID Controller | `lib/PID/` | 📋 Planned |
| Move Base / EKF | `lib/MoveBase/` | 📋 Planned |

---

### Files Added This Session

```
lib/AS5600/as5600.cpp                        — Encoder driver (complete)
lib/AS5600/as5600.h                          — Driver header with as5600_t struct
matlab/imu_stream.m                          — IMU-only live stream + CSV export
matlab/encoder_stream.m                      — Encoder-only live stream + CSV export
matlab/imu_analysis.m                        — IMU offline statistical analysis
matlab/encoder_analysis.m                    — Encoder offline kinematic analysis
```

### Next Steps
- [ ] **Add 3 remaining encoders** — replicate `enc_LF` pattern for RF, LR, RR;
      add mecanum forward kinematics block to `loop()` computing `vx`, `vy`, `ω`
- [ ] **IMU vibration test under motor load** — characterise noise floor increase
      relative to the 2026-04-01 stationary baseline
- [x] **Implement `MotorWheel` module** — extract motor control from `main.cpp`
      following the same driver pattern as IMU and AS5600
- [ ] **Implement `PID` module** — reentrant per-motor instances using the same
      struct pointer pattern as the sensor drivers


--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## 2026-04-04 — Motor Driver Module & Open-Loop System Identification

### Overview
Two major threads today: (1) implemented the `Motor` driver module following the
same component-based architecture established for the IMU and encoder drivers;
(2) designed and executed a structured open-loop characterisation session using a
new MATLAB test suite, extracting the key physical parameters of the motor-encoder
system needed to make an informed first attempt at PID tuning tomorrow.

---

### Part 1: Motor Driver Module

#### Architecture

The motor driver follows the same pattern as the IMU and encoder modules — a
struct holds all hardware state, and a set of functions operate on a pointer to
that struct. This makes the driver reentrant: declaring `motor_t motor_LF` and
`motor_t motor_RF` and calling the same functions on each is all that is needed
to support four independent motors without duplicating any logic.

```c
typedef struct {
    uint8_t motor_pwm_pin;
    uint8_t motor_pin1;
    uint8_t motor_pin2;
    uint8_t motor_enable_pin;
    float   current_power;   // -100.0 to +100.0
    bool    is_inverted;     // software direction flip if wired backwards
} motor_t;
```

#### Key Implementation Details

**Power representation** uses a -100.0 to +100.0 percentage scale rather than
raw PWM (0–255). This decouples the control logic from the hardware, making the
PID output human-readable and portable if the MCU or driver chip ever changes.
The conversion to PWM happens only at the lowest level inside `motor_set_speed()`:

```cpp
uint8_t pwm = (uint8_t)map(fabs(power), 0.0f, 100.0f, 0, 255);
```

**Direction control** is handled by the TB6612FNG H-bridge logic. IN1/IN2 set
the direction, PWM sets the magnitude, and the STANDBY pin enables or disables
the entire driver chip. The four operating states are:

| IN1 | IN2 | PWM | Behaviour |
|-----|-----|-----|-----------|
| HIGH | LOW | >0 | Forward |
| LOW | HIGH | >0 | Reverse |
| LOW | LOW | any | Coast (free spin) |
| HIGH | HIGH | 255 | Brake (short circuit) |

**Braking** applies full PWM with both IN pins HIGH, creating a short-circuit
across the motor terminals that generates maximum resistive braking torque.
This is used for emergency stops, not normal deceleration.

**`is_inverted` flag** is reserved for the case where a motor is physically wired
backwards on the robot chassis. Setting this flag in software flips the direction
without rewiring — important when all four mecanum wheels are mounted and some
face opposite directions.

#### Non-Blocking Serial Command Interface

A key change to `main.cpp` was replacing `Serial.parseFloat()` with a
character-by-character accumulation buffer:

```cpp
String serial_buffer = "";

while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
        if (serial_buffer.length() > 0) {
            float commanded_power = serial_buffer.toFloat();
            motor_set_speed(&motor_LF, commanded_power);
        }
        serial_buffer = "";
    } else {
        serial_buffer += c;
    }
}
```

`parseFloat()` blocks the entire loop for up to the Serial timeout period while
waiting for more digits. During that blocking window, the encoder cannot be read
and no data is streamed. The buffer approach reads one character at a time, acts
only when a newline arrives, and never stalls the control loop. The `length() > 0`
guard prevents a bare newline from triggering a `toFloat()` call that returns 0.0
and silently stops the motor.

---

### Part 2: Open-Loop System Identification — What We Did and Why

Before implementing a PID controller, we need to understand how the physical
system behaves. This section explains the MATLAB test suite in plain terms and
documents what each result means.

#### The Core Question: What Are We Trying to Learn?

A PID controller works by computing an error (the difference between where the
wheel is and where we want it to be) and converting that error into a motor power
command. But to choose the right PID gains, we first need to know three things:

1. **What is the minimum power to make the wheel move at all?** — Deadband
2. **How fast does the wheel spin at each power level?** — Plant Gain K
3. **How quickly does the wheel reach its target speed?** — Time Constant τ

The MATLAB script answers all three automatically by commanding the motor and
reading the encoder response.

#### How the MATLAB-Arduino Communication Works

MATLAB acts as the test controller. Arduino acts as the executor and reporter.
They talk over the USB serial cable:

```
MATLAB sends:   "40.00\n"
Arduino reads:  serial_buffer = "40.00" → motor_set_speed(40.0) → wheel spins

Arduino sends:  "12345,40.00,1.234567,8.910000\n"
                [Time_ms, Power_pct, Angle_rad, Vel_rads]
MATLAB reads:   stores the 4 numbers in a growing data matrix
```

MATLAB stores every line in a buffer, then analyses the numbers after each
segment ends. The motor never needs to be reprogrammed between tests — MATLAB
drives everything dynamically over serial.

#### Test 1: Deadband Identification — "What is the Minimum Effective Power?"

**The concept:** A motor with a gearbox does not move the moment you apply any
power. There is a threshold below which friction in the gearbox exceeds the
motor's torque output. Below this threshold, the motor hums but the wheel stays
still. This threshold is the *deadband*.

**What the test does:** MATLAB commands power levels from 5% to 60% in steps of
5%, holding each for 2.5 seconds. Before each step it stops the motor and flushes
the serial buffer to prevent stale readings contaminating the new segment. After
each segment it computes the mean absolute velocity. When this mean exceeds a
noise threshold of 0.05 rad/s, the wheel has started moving — that power level
is the deadband.

**Why this matters for PID:** If a PID controller outputs 15% power and the
deadband is 30%, the motor does nothing. The integrator keeps accumulating error,
the output keeps climbing, and when the wheel finally breaks free it lurches
violently. Knowing the deadband lets us clamp the PID output to never command
below it when motion is intended.

**Result: Deadband = 30%**

The wheel produced no meaningful motion below 30%. At 30% and above, velocity
rose cleanly and monotonically. This is a relatively high deadband, typical of
brushed DC motors with plastic gearboxes.

#### Test 2: PWM-to-Velocity Map — "How Fast Does the Wheel Spin per Unit of Power?"

**The concept:** Once above the deadband, there is a relationship between how
much power we command and how fast the wheel spins at steady state. If this
relationship is approximately linear, we can describe it with a single number
called the *plant gain K*.

**What the test does:** MATLAB commands fixed power levels from ±30% to ±100%,
holding each for 3 seconds. For each level, it discards the first 1500ms (the
acceleration transient) and computes the mean velocity from the remaining
steady-state tail. It then fits a straight line through all the (power, velocity)
pairs. The slope of that line is K.

**The linear model:**

```
velocity (rad/s) = K × power (%)
K = 0.2291 (rad/s) per % power
```

This means for every 1% of power above the deadband, the wheel gains
approximately 0.23 rad/s. At 100% power the wheel reaches ~23 rad/s
(approximately 220 RPM).

**Forward vs Reverse symmetry:**

| Direction | Velocity at 100% |
|-----------|-----------------|
| Forward | 23.02 rad/s |
| Reverse | 23.95 rad/s |

Only 4% asymmetry — excellent for a brushed motor. Separate PID gains per
direction are not needed.

**Why K matters for PID:** K tells us how sensitive the system is to our
commands. The mathematical starting point for Kp is simply `1/K` — if the system
is less sensitive (small K) we need a larger proportional gain to make up for it.

**Result: K = 0.2291 → suggests Kp ~ 4.37**

#### Test 3: Step Response — "How Quickly Does the Wheel Respond?"

**The concept:** When we suddenly apply a fixed power command, the wheel does not
instantly reach its target speed. It accelerates over a period of time, limited
by the motor's inertia and back-EMF. The characteristic time of this rise is the
*time constant τ*.

**What the test does:** MATLAB commands a sudden jump from 0% to a fixed power
level (40%, 60%, 100%) and records the encoder velocity every 10ms for 4 seconds.

**What τ means:** τ is defined as the time for the velocity to reach 63.2% of
its final steady-state value. This comes from the mathematics of first-order
systems — a motor-gearbox behaves approximately like:

```
velocity(t) = v_steady × (1 − e^(−t/τ))
At t = τ:  velocity = v_steady × 0.632
```

In the MATLAB code this is found by scanning the velocity array for the first
sample that crosses the 63.2% mark:

```matlab
v_ss    = mean(vel(end-10:end));         % steady-state from last 10 samples
tau_idx = find(vel >= 0.632 * v_ss, 1); % first crossing of 63.2%
tau     = t_rel(tau_idx);               % elapsed time at that index
```

**Why τ matters for PID:**
- A small τ means the motor responds quickly
- Ki eliminates steady-state error and should operate on the timescale of τ: `Ki = Kp / τ`
- Kd adds damping proportional to the rate of change: `Kd = Kp × τ × 0.1`

**Result: τ = 0.10 s**

The wheel reaches 63.2% of its steady-state speed in 100ms. With a 10ms control
interval, the PID loop completes 10 cycles within one time constant — more than
sufficient resolution for stable control.

#### Computed PID Starting Points

| Parameter | Formula | Value |
|-----------|---------|-------|
| Kp | 1 / K | 4.37 |
| Ki | Kp / τ | 43.66 |
| Kd | Kp × τ × 0.1 | 0.044 |

These are mathematically derived starting points, not final values. For
tomorrow's tuning session, we start conservatively:

```
Kp = 2.0   — half the computed value to avoid overshoot on first run
Ki = 0.0   — add only after Kp is stable
Kd = 0.0   — add last if oscillation appears
```

**Tuning sequence for tomorrow:**
1. Increase Kp until the wheel tracks a setpoint with acceptable overshoot
2. Add Ki to eliminate the steady-state error that persists when Kp alone is used
3. Add Kd only if the response oscillates or overshoots significantly

#### The Feedforward Insight

Because the deadband is large (30%), a purely reactive PID controller will
struggle at low speeds — the integrator winds up trying to overcome the deadband,
causing a lurch when the motor finally breaks free. A better approach adds a
*feedforward* term that maps the desired velocity directly to a base power level:

```
output = deadband_pct + (desired_velocity / K)
PID corrects only the residual error on top of this baseline
```

This will be implemented as part of the PID module.

---

### Troubleshooting Log — Serial Communication Bugs

Three separate bugs were encountered and resolved during the MATLAB integration:

| Bug | Symptom | Root Cause | Fix |
|-----|---------|------------|-----|
| `parseFloat()` blocking | Motor unresponsive during sweep | parseFloat blocks loop for full timeout | Non-blocking char buffer |
| Double newline | Motor stops instantly after command | `writeline` + `\n` in sprintf = two newlines | Remove `\n` from sprintf |
| Second `flush()` eating data | All velocities read as 0.0 | flush after command discarded real measurements | Remove post-command flush |

These bugs reinforced a key principle: **serial timing bugs are invisible.**
The motor was visibly spinning during most of these failures, which made them
harder to identify. The fix was to treat the serial protocol as a strict contract:
one side sends, the other receives, and nothing flushes the pipe while data is
expected.

---

### Module Status Roadmap

| Module | Files | Status |
|--------|-------|--------|
| POST / System Health | `lib/POST/post.cpp` | ✅ Stable |
| IMU Driver | `lib/IMU/imu.cpp` | ✅ Stable |
| AS5600 Encoder Driver | `lib/AS5600/as5600.cpp` | ✅ Stable |
| Motor Driver | `lib/Motor/motor.cpp` | ✅ Complete |
| MATLAB Open-Loop Suite | `matlab/motor_openloop_test.m` | ✅ Complete |
| PID Controller | `lib/PID/` | 🔄 Next — parameters ready |
| Move Base / EKF | `lib/MoveBase/` | 📋 Planned |

---

### Files Added This Session

```
lib/Motor/motor.cpp                           — Motor driver implementation
lib/Motor/motor.h                             — Motor struct and function prototypes
src/main.cpp                                  — Updated with non-blocking serial parser
matlab/motor_openloop_test.m                  — Full open-loop characterisation suite
matlab/CSVs/openloop_sweep_2026-04-04.csv     — Sweep results (power vs velocity)
matlab/CSVs/openloop_step40_2026-04-04.csv   — Step response at 40%
matlab/CSVs/openloop_step60_2026-04-04.csv   — Step response at 60%
matlab/CSVs/openloop_step100_2026-04-04.csv  — Step response at 100%
```

### System Identification Results Summary

| Parameter | Value | Unit |
|-----------|-------|------|
| Deadband | 30 | % power |
| Plant gain K | 0.2291 | (rad/s)/% |
| Max velocity | 23.95 | rad/s |
| Time constant τ | 0.10 | s |
| Fwd/Rev asymmetry | 4 | % |
| Suggested Kp | 4.37 | — |
| Suggested Ki | 43.66 | — |
| Suggested Kd | 0.044 | — |

### Next Steps

- [ ] **Implement PID module** — reentrant `pid_t` struct with Kp, Ki, Kd,
      integral accumulator, derivative filter, output clamping, and anti-windup
- [ ] **Integrate feedforward** — map desired velocity to base power using K and
      deadband offset; PID corrects residual error only
- [ ] **Closed-loop step test** — command a velocity setpoint, record actual
      velocity, tune gains until response is satisfactory
- [ ] **Extend to 4 motors** — replicate motor_t for RF, LR, RR; add mecanum
      forward kinematics to compute body vx, vy, ω


--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## 2026-04-05 — Plant Modelling, PID Auto-Tuning & Simulink Validation

### Overview
Before writing a single line of PID code, we took a step back and modelled the
motor-encoder system mathematically, then used MATLAB's built-in `pidtune()`
function to compute optimal gains automatically from that model. The gains were
then validated inside a Simulink simulation to confirm the closed-loop behaviour
before any hardware was touched. This session bridges the open-loop measurements
from yesterday to the PID firmware implementation tomorrow.

#### Reference Material
Brian Douglas' *Control System Lectures* playlist on YouTube was used as the
primary reference for understanding PID implementation theory, discretisation,
anti-windup, and filtered derivative concepts:
https://youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y

---

### Part 1: What Is a Transfer Function and Why Do We Need One?

#### The Problem With Guessing PID Gains

A PID controller has three knobs: Kp, Ki, and Kd. If you turn them randomly, the
wheel might overshoot wildly, oscillate forever, or respond too slowly to be
useful. To choose them intelligently, you need a mathematical description of how
the motor physically responds to a command. This description is called the
**transfer function**.

#### The Transfer Function G(s) — In Plain English

Think of the transfer function as the motor's "personality profile." It answers
the question: *if I send this power command, what velocity will I get, and how
quickly?*

For our motor, yesterday's open-loop tests revealed that it behaves like a
**first-order system** — the simplest possible dynamic system. This means:

- It has one source of "sluggishness" (mechanical inertia)
- Its velocity rises smoothly toward a final value, never oscillating on its own
- It is fully described by just two numbers: K and τ

The transfer function is written as:

```
G(s) = K / (τs + 1)
G(s) = 0.2291 / (0.1s + 1)
```

Where:
- **K = 0.2291** is the plant gain — how many rad/s you get per % of power at
  steady state
- **τ = 0.1s** is the time constant — how quickly the motor reaches that speed
- **s** is a mathematical operator (the Laplace variable) that represents
  frequency; it allows us to analyse the system's behaviour across all speeds
  of input change simultaneously, not just for one specific input

In practical terms, this equation says: "apply 40% power, and after about 0.3
seconds (3τ) the wheel will be spinning steadily at 40 × 0.2291 = 9.16 rad/s."
Yesterday's measurement confirmed exactly this — the steady-state velocity at 40%
was 8.95 rad/s, which matches the model to within 2%.

---

### Part 2: The Bode Plot — Reading the Motor's Frequency Response

The Bode plot is a standard engineering tool that shows how well the motor tracks
commands at different speeds of change. There are two panels:

**Magnitude plot (top)** — shows how much of the commanded velocity actually
appears at the output, expressed in decibels (dB). 0 dB means perfect tracking;
negative dB means the output is weaker than the command.

**Phase plot (bottom)** — shows how much the output lags behind the command in
time, expressed in degrees. 0° means no lag; -90° means the output is a quarter
cycle behind the input.

#### What Our Bode Plot Shows

```
Low frequencies (slow commands, < 1 rad/s):
  Magnitude ≈ -13 dB = our gain K expressed logarithmically
  Phase ≈ 0° — the wheel tracks slow commands faithfully with no lag

Corner frequency at 10 rad/s (= 1/τ = 1/0.1):
  This is where the motor starts struggling to keep up
  Magnitude begins rolling off at -20 dB per decade
  Phase starts dropping toward -45°

High frequencies (fast commands, > 100 rad/s):
  Magnitude continues dropping — the motor physically cannot follow
  Phase approaches -90° — the output lags badly behind the command
```

The key insight is that a first-order system never loses more than 90° of phase,
no matter how fast the input changes. This makes it **inherently stable** — you
cannot make it oscillate just by increasing the gain. This is what makes our
motor-gearbox system relatively straightforward to control.

---

### Part 3: How pidtune() Works — Automatic Gain Calculation

`pidtune()` is MATLAB's automatic PID gain calculator. You hand it the transfer
function and the type of controller you want, and it returns the optimal gains.

#### What "Optimal" Means Here

`pidtune()` targets a **phase margin of 60°** by default. Phase margin is a
measure of how far the system is from instability — 0° means it is right on the
edge of oscillating, 60° means it has a comfortable safety buffer. This is an
industry standard for robust control: the system will remain stable even if the
real motor behaves somewhat differently from the model.

#### The Four Controllers Compared

```matlab
C_p    = pidtune(G, 'P');    % proportional only
C_pi   = pidtune(G, 'PI');   % proportional + integral
C_pid  = pidtune(G, 'PID');  % proportional + integral + derivative
C_fast = pidtune(G, 'PID', opts);  % PID with faster target bandwidth
```

Results:

| Controller | Kp | Ki | Kd |
|---|---|---|---|
| P | 8.9170 | — | — |
| PI | 4.5508 | 136.57 | — |
| PID | 6.2497 | 113.28 | 0.0 |
| PID fast | 7.1885 | 132.09 | 0.0 |

#### Why Kd = 0 Is the Correct Answer

`pidtune()` returned `Kd = 0` for both PID variants. This surprises people who
expect PID to always use all three terms, but it is mathematically correct here.

The derivative term amplifies high-frequency changes in the error signal. The
faster the error changes, the larger the derivative output. The problem is that
encoder velocity readings contain quantisation noise — small random jumps that
look like very fast changes. A non-zero Kd would amplify this noise directly into
the motor command, causing jitter and heat.

For our motor with τ = 0.1s, the system is already fast enough that derivative
action provides no meaningful benefit. `pidtune()` correctly identifies this and
sets Kd = 0, making this effectively a PI controller. **Trust the maths.**

---

### Part 4: Simulated Performance — Comparing Controllers

To evaluate each controller before touching hardware, we simulated a step command
— the wheel is asked to jump from 0 to 1 rad/s instantly — and observed how each
controller responds.

#### What Each Metric Means

**Settling time** — how long until the wheel stays within 2% of the target and
stays there. A shorter settling time means faster response.

**Overshoot** — how much the wheel exceeds the target before correcting. Expressed
as a percentage of the target. Some overshoot is normal; too much means the
controller is aggressive and will cause mechanical stress.

**Steady-state error** — the permanent gap between the target and the actual speed
after the transient dies out. For a robot wheel, this must be zero — if you command
5 rad/s and the wheel settles at 4.6 rad/s, your robot will drift.

#### Results

| Controller | Settle | Overshoot | SS Error | Verdict |
|---|---|---|---|---|
| P | 2.000 s | 0.0% | 32.87% | ❌ Unusable — permanent 33% error |
| PI | 0.296 s | 13.8% | 0.00% | ⚠️ Fast but bouncy |
| PID | 0.317 s | 6.1% | 0.00% | ✅ Best balance |
| PID fast | 0.292 s | 6.5% | 0.00% | ✅ Slightly faster, slightly bouncier |

**Why P alone fails:** A proportional controller outputs power proportional to
error. As the wheel approaches the target, the error shrinks, the output shrinks,
and the two reach equilibrium before zero error is achieved. The 32.87% permanent
offset is not a bug — it is a fundamental mathematical property of P control on
this type of plant. The integral term exists specifically to drive this error to
zero by accumulating error over time and continuously pushing until it disappears.

**Why PID was chosen over PID fast:** The standard PID settles in 317ms with 6.1%
overshoot. The fast variant settles in 292ms (25ms faster) but with 6.5% overshoot.
For a wheel velocity controller on a robot, that extra 25ms is not worth the
additional overshoot which would cause the robot to jerk slightly at every speed
change.

---

### Part 5: Simulink Validation — The Final Check Before Hardware

Rather than rebuilding the Simulink model from scratch on every run, a permanent
`motor_pid_simulation_final.slx` model was created and saved. The MATLAB script
loads this model, injects the gains from `pid_gains.mat`, runs the simulation,
and reports the results.

#### What the Simulink Model Contains

```
[Step Setpoint] → [Sum] → [PID Controller] → [Dead Zone] → [Plant G(s)] → [Scope]
                    ↑                                              |
                    └──────────────── feedback ───────────────────┘
```

- **Step Setpoint** — commands the wheel to jump to 10 rad/s at t=0
- **Sum** — computes the error (setpoint minus actual velocity)
- **PID Controller** — applies Kp, Ki, Kd to the error
- **Dead Zone** — models the 30% deadband; any command below ±30% is treated as
  zero, just as the real hardware behaves
- **Plant G(s)** — the transfer function representing the motor-encoder system
- **Feedback loop** — feeds the plant output back to the sum block to close the loop

#### Simulation Results

```
Rise time:     0.126 s   — wheel reaches 63% of 10 rad/s in 126ms
Settling time: 0.329 s   — within 2% of 10 rad/s in 329ms
Overshoot:     3.2%      — wheel briefly hits 10.32 rad/s before settling
Steady-state:  10.0000   — perfect, zero permanent error
```

The overshoot in Simulink (3.2%) is lower than in the lsim comparison (6.1%)
because the Simulink model includes the Dead Zone block. In the early moments of
a step response the PID output is very large, but the Dead Zone clips the
lower-end commands during the settling phase, which naturally reduces overshoot.
The Simulink result is the more realistic prediction of what the real hardware
will do.

329ms settling time with a 10ms control loop means the PID converges in
approximately 33 control cycles — fast enough for smooth tracking, stable enough
not to fight itself.

---

### Part 6: Anti-Windup — A Critical Implementation Requirement

Ki = 113.28 is a large value. This creates a specific risk called **integrator
windup** that must be addressed in the firmware before the PID is used on hardware.

**What windup means:** The integral term accumulates error over time. If the wheel
is blocked or stalled, the PID cannot reach the setpoint, so error keeps
accumulating. With Ki = 113, just one second of stall accumulates 113 units of
integral output. When the wheel finally frees, this accumulated value causes it to
lurch violently far past the setpoint.

**The fix — anti-windup clamping:** The integral accumulator must be hard-limited
to the output range (±100% power). If the output is already saturated, the
integrator stops accumulating. This is the first feature to implement in the
PID library tomorrow, before any other functionality.

```cpp
// Anti-windup: clamp integral accumulator to output limits
integral += Ki * error * dt;
integral  = constrain(integral, output_min, output_max);
```

---

### Final Arduino PID Starting Values

These are the values exported to `pid_gains.mat` and ready to copy directly into
the PID firmware:

```cpp
float Kp       = 6.2497;
float Ki       = 113.2799;
float Kd       = 0.0000;   // Derivative not needed — PI controller
float deadband = 30.0;     // % power — from Test 1
float dt       = 0.010;    // 10ms control interval
```

---

### Module Status Roadmap

| Module | Files | Status |
|--------|-------|--------|
| POST / System Health | `lib/POST/post.cpp` | ✅ Stable |
| IMU Driver | `lib/IMU/imu.cpp` | ✅ Stable |
| AS5600 Encoder Driver | `lib/AS5600/as5600.cpp` | ✅ Stable |
| Motor Driver | `lib/Motor/motor.cpp` | ✅ Stable |
| MATLAB Open-Loop Suite | `matlab/motor_openloop_test.m` | ✅ Complete |
| MATLAB Model & Tune | `matlab/motor_model_and_tune.m` | ✅ Complete |
| Simulink Model | `matlab/motor_pid_simulation_final.slx` | ✅ Complete |
| PID Controller | `lib/PID/` | 🔄 Next — gains ready |
| Move Base / EKF | `lib/MoveBase/` | 📋 Planned |

---

### Files Added This Session

```
matlab/motor_model_and_tune.m                 — Plant identification, pidtune(), simulation
matlab/motor_pid_simulation_final.slx         — Permanent Simulink closed-loop model
matlab/CSVs/pid_gains.mat                     — Exported Kp, Ki, Kd, K, τ, deadband
```

### Key Results Summary

| Parameter | Value | Unit |
|-----------|-------|------|
| Transfer function | 0.2291 / (0.1s + 1) | — |
| Chosen controller | PI (PID with Kd=0) | — |
| Kp | 6.2497 | — |
| Ki | 113.2799 | — |
| Kd | 0.0000 | — |
| Simulated rise time | 0.126 | s |
| Simulated settling time | 0.329 | s |
| Simulated overshoot | 3.2 | % |
| Steady-state error | 0.0000 | rad/s |

### Next Steps

- [ ] **Implement PID library** — `pid_t` struct with Kp, Ki, Kd, integral
      accumulator, anti-windup clamping, output saturation, and dt
- [ ] **Anti-windup first** — with Ki = 113, integrator windup is not optional;
      implement and test clamping before any closed-loop hardware run
- [ ] **Closed-loop step test** — command a velocity setpoint over serial,
      stream actual velocity back to MATLAB, compare against Simulink prediction
- [ ] **Feedforward integration** — combine PID with deadband-offset feedforward
      for improved low-speed tracking
- [ ] **Extend to 4 motors** — replicate motor_t and pid_t for RF, LR, RR


--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## 2026-04-13 — PID Library Implementation & First Closed-Loop Hardware Tests

### Overview
Two major threads today: (1) implemented the PID controller library following the
same component-based pattern as the IMU, encoder, and motor drivers; (2) ran the
first closed-loop velocity control tests on hardware, discovering and resolving
three critical bugs that prevented the controller from working. By end of session
the wheel was tracking velocity setpoints over serial with MATLAB live monitoring.

---

### Part 1: PID Controller Library

#### Architecture

The PID driver follows the same reentrant struct pattern as every other module.
All state and configuration lives in `pid_ctrl_t`, and functions operate on a
pointer to it. Declaring `pid_ctrl_t pid_LF` and `pid_ctrl_t pid_RF` with the
same functions gives four independent controllers without duplicating logic.

```c
typedef struct {
    float kp, ki, kd;             // Gains
    float kff;                    // Feedforward gain
    float deadband;               // % power offset
    float output_min, output_max; // Output limits (-100 to 100)
    float dt;                     // Control interval in seconds
    float setpoint;               // Target velocity (rad/s)
    float last_error;             // For derivative term
    float integral_sum;           // For integral term
} pid_ctrl_t;
```

#### Anti-Windup — Clamping Method

The integral accumulator is only updated when the output is within limits. If the
output saturates, the integrator freezes:

```c
if (output > pid->output_max)      output = pid->output_max;
else if (output < pid->output_min) output = pid->output_min;
else                               pid->integral_sum += error * pid->dt;
```

This prevents the integrator winding up during stall or saturation — a critical
requirement given Ki=113 in the initial gains.

#### `pid_init()` calls `pid_reset()`

Rather than duplicating zeroing logic, `pid_init()` calls `pid_reset()` at the
end to zero all state fields. This ensures the struct is always clean on
initialisation regardless of memory contents:

```c
void pid_init(...) {
    pid->kp = kp; pid->ki = ki; // assign all config...
    pid_reset(pid);              // zero all state
}
```

---

### Part 2: Critical Bugs Found and Fixed

#### Bug 1 — Missing `dt` in Integral Accumulation

The most damaging bug. The integral was implemented as:

```c
pid->integral_sum += error;          // WRONG
i_term = pid->ki * pid->integral_sum;
```

With Ki=113.28 and a 10 rad/s setpoint, after the first cycle:
- `integral_sum = 10` (first error value)
- `i_term = 113.28 × 10 = 1132%` — 11x over the output limit

The motor clamped to 100% instantly and stayed there. The fix:

```c
pid->integral_sum += error * pid->dt;  // CORRECT
```

Now with dt=0.02: `i_term = 113.28 × 0.1 = 11.3%` per second — completely sane.

#### Bug 2 — `motor_set_speed()` Using Wrong Variable

After applying the deadband offset to create `final_power`, all subsequent
direction checks and PWM calculation still used the original `power` variable:

```cpp
float final_power = power + MOTOR_DEADBAND;  // computed correctly
if (power > 0.1) { ... }                     // but direction used 'power' ❌
uint8_t pwm = map(fabs(power), ...);         // PWM also used 'power' ❌
```

Direction was being set from the pre-deadband value while PWM was driven from
the post-deadband value — causing erratic behaviour near the deadband boundary.
All checks changed to use `final_power`.

#### Bug 3 — Motor Running Before Any Command

Arduino enters `loop()` immediately after calibration. With setpoint=0 and
encoder noise producing non-zero `current_velocity`, the error was non-zero and
the motor started spinning before MATLAB connected. Fixed with a gate flag:

```cpp
bool setpoint_received = false;

// Gate: motor only runs after first command arrives
if (setpoint_received) {
    float target_power = pid_compute(&pid_LF, current_velocity);
    motor_set_speed(&motor_LF, target_power);
} else {
    motor_set_speed(&motor_LF, 0.0f);
}
```

---

### Part 3: Serial Protocol Upgrade

The serial interface was upgraded from raw float commands to a structured
prefix protocol, allowing MATLAB to change gains live without reflashing:

```
S:<float>  — set velocity setpoint (rad/s), opens gate
P:<float>  — update Kp, resets integral
I:<float>  — update Ki, resets integral
D:<float>  — update Kd, resets integral
F:<float>  — update Kff, resets integral
R:         — full reset, stop motor, close gate
```

Telemetry expanded from 4 to 7 columns:
```
Time_ms, Setpoint, Velocity, Power, Kp, Ki, Kd
```

Active gains are echoed every cycle so MATLAB can verify commands were received
before logging a segment — this eliminates contamination from transition windows
where old gains are still active.

---

### Part 4: MATLAB Live Monitor

`matlab_live_pid_monitor.m` was rebuilt with a non-blocking UI. The previous
version used `input()` for setpoint entry which blocked the serial loop and
caused Arduino to disconnect every time the user typed. The fix uses a figure
text box and Send button — the serial loop never pauses.

Additional features added:
- **Stop button** — sends `R:` then `S:0.00`
- **Sequence runner** — enter `7,10,13,15` and a hold time; steps through
  setpoints automatically with `pause()` between them
- **Dark mode palette** — blue velocity, amber setpoint, green power, dark axes
- **7-column storage** — buffer now captures Kp, Ki, Kd alongside telemetry
- **CSV save on any exit** — figure close, Ctrl+C, or runtime error

---

### Module Status Roadmap

| Module | Files | Status |
|--------|-------|--------|
| POST / System Health | `lib/POST/post.cpp` | ✅ Stable |
| IMU Driver | `lib/IMU/imu.cpp` | ✅ Stable |
| AS5600 Encoder Driver | `lib/AS5600/as5600.cpp` | ✅ Stable |
| Motor Driver | `lib/Motor/motor.cpp` | ✅ Stable |
| PID Controller | `lib/PID/pid_controller.c` | ✅ Complete |
| MATLAB Live Monitor | `matlab/matlab_live_pid_monitor.m` | ✅ Complete |
| 4-Wheel Integration | `src/main.cpp` | 📋 Next |

### Next Steps

- [ ] **Run automated gain sweep** — MATLAB commands gains over serial,
      logs response per combination, ranks by composite score
- [ ] **Add feedforward** — baseline power from setpoint directly to
      reduce integrator load and improve low-speed tracking
- [ ] **Multi-setpoint validation** — confirm gains hold across 7–15 rad/s
- [ ] **Extend to 4 motors** — replicate motor_t and pid_t for RF, LR, RR



--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## 2026-04-24 — Automated PID Sweep, Feedforward Integration & Final Tuning

### Overview
Three major threads today: (1) built and ran an automated MATLAB gain sweep to
identify optimal PID values empirically from real hardware data; (2) implemented
and debugged a feedforward term that reduced settling time by 14x; (3) ran a full
multi-setpoint validation confirming the controller works cleanly across the entire
7–15 rad/s operating range. Single-wheel closed-loop velocity control is complete.

---

### Part 1: Why the MATLAB Model Gains Failed on Hardware

The gains computed by `pidtune()` on 2026-04-05 were `Kp=6.25, Ki=113.28`.
Seven hardware sessions with these gains all produced violent oscillation between
±23 rad/s — the wheel never settled. Two root causes were identified:

**Root cause 1 — Missing `dt` in integral accumulation (described in previous
entry).** With `integral_sum += error` instead of `integral_sum += error * dt`,
Ki=113 accumulated error 50x faster than the model assumed.

**Root cause 2 — Deadband compensation changes the effective plant gain.** The
linear model `G(s) = 0.2291 / (0.1s + 1)` was identified from open-loop data
where `motor_set_speed()` received raw power percentages. In closed-loop,
`motor_set_speed()` adds 30% to every non-zero output. This shifts the effective
plant gain significantly — a PID output of 10% becomes 40% at the motor, making
the real loop gain ~4x higher than the model assumed. Ki=113 on this modified
plant caused the oscillation the model said would never happen.

**The lesson:** Model-derived gains are starting points. Any unmodelled nonlinearity
in the real plant — deadband, friction, saturation — changes the effective loop
gain and requires empirical correction.

---

### Part 2: Automated Gain Sweep — Finding the Real Working Gains

Rather than manually reflashing for each gain combination, MATLAB was extended to
send gains over serial using the `P:/I:/D:` command protocol. The sweep script
tests every combination in a grid, logs the response, computes metrics, ranks
results, and saves everything to CSV automatically.

#### Sweep 1 — Ki range 0.5 to 5.0 (no feedforward)

Initial hypothesis: Ki must be much lower than 113. First sweep used
`Kp=[4,6,8,10]` and `Ki=[0.5,1,2,5]`.

**Result: All combinations showed zero overshoot and zero oscillation — but also
zero settling.** Every combination was still ramping at the end of the 5-second
window. The integrator was so weak it could not accumulate enough output to
overcome the deadband and reach steady state. Ki=5 was the lowest value that
showed any sign of settling.

**Insight:** With the correct `error * dt` formulation, one second of 5 rad/s
error accumulates `5 × 0.02 × 50 = 5` in the integrator. With Ki=5 that is only
25% power after one second — still not enough to hold 10 rad/s which needs ~74%.
The integrator needs more time or higher Ki.

#### Sweep 2 — Ki range 5 to 30 (no feedforward)

Second sweep: `Kp=[6,8,10,12]`, `Ki=[5,10,20,30]`, `SETTLE_WINDOW=10s`.

**Winner: Kp=10, Ki=30** — 0.84s settling, 8.9% overshoot, 0.1% SS error.

This was a genuine step change. Ki=30 accumulated enough integral action within
the window to drive the output to steady state without oscillating. The settling
time bar chart showed Kp=10, Ki=30 at 0.84s while every other combination was
still above 3s.

#### Multi-setpoint validation without feedforward

Tested Kp=10, Ki=30 at 5, 10, 15, 20 rad/s:
- 5 rad/s: settled at 3.14s, 19.6% overshoot — deadband dominates at low speed
- 10 rad/s: settled at 0.56s, 10.5% overshoot — good
- 15 rad/s: settled at 5.80s — approaching saturation
- 20 rad/s: never settled — physically beyond motor capability at 7.4V

Operating range without feedforward: **8–15 rad/s**.

---

### Part 3: Feedforward — What It Is and Why It Helps

#### The Problem with Pure PI Control

A PI controller starts from zero output and climbs toward the required level
purely through error accumulation. At 10 rad/s the motor needs ~74% power. The
integral must build from 0% to 74% before the wheel reaches target speed. During
this ramp the wheel is below setpoint, the error is large, and the integral keeps
growing — causing overshoot when it finally arrives.

#### The Feedforward Solution

Feedforward computes the expected power needed to reach the setpoint directly
from the plant inverse, bypassing the integrator entirely for the steady-state
component:

```
ff_term = sign(setpoint) × Kff × |setpoint|
Kff = 1/K = 1/0.2291 = 4.366
```

At setpoint = 10 rad/s: `ff_term = 4.366 × 10 = 43.66%`

`motor_set_speed()` then adds the 30% deadband offset: `43.66 + 30 = 73.66%` —
exactly the power the motor needs. The integrator only corrects the small residual
error caused by friction, load variations, and encoder noise.

The total PID+FF output:

```
output = ff_term + Kp×error + Ki×integral_sum + Kd×d_error/dt
```

#### The Double Deadband Bug

First implementation of feedforward included the deadband inside the ff_term:

```c
ff_term = pid->deadband + pid->kff * pid->setpoint;  // WRONG
```

`motor_set_speed()` then added another 30%:
```
total = (30 + 43.66) + 30 = 103.66% → clamped to 100%
```

The motor was commanded to 100% from cycle 1 of every test — causing 22–44%
overshoot across every gain combination regardless of Kp or Ki. The fix removes
deadband from `pid_compute()` entirely:

```c
// CORRECT — motor_set_speed() handles deadband
ff_term = sign * pid->kff * fabsf(pid->setpoint);
```

This also fixes reverse direction — at -10 rad/s: `ff_term = -43.66%`, then
`motor_set_speed()` subtracts 30%: `final = -73.66%`. Correct.

#### Sweep 3 — With Corrected Feedforward

Grid: `Kp=[4,6,8,10]`, `Ki=[2,5,10,15]`, `Kff=4.366` fixed.

**Results were dramatically better across the board:**

| Rank | Kp | Ki | Overshoot | Settling | SS Error |
|---|---|---|---|---|---|
| 1 | 10.0 | 5.0 | 8.2% | 0.14s | 0.06% |
| 2 | 10.0 | 10.0 | 8.2% | 0.06s | 0.11% |
| 3 | 8.0 | 5.0 | 8.2% | 0.06s | 0.29% |

The 8.2% overshoot floor is the physical step response of the motor — it cannot
be reduced by gain tuning alone without a filtered derivative or a slower setpoint
ramp. The feedforward eliminated the need for Ki to carry the steady-state load,
so Ki could drop from 30 to 10 while settling improved from 0.84s to 0.06s.

**Chosen: Kp=10, Ki=10** — same overshoot as Ki=5, settling twice as fast
(0.06s vs 0.14s), negligible SS error difference (0.11% vs 0.06%).

---

### Part 4: Final Multi-Setpoint Validation

Tested Kp=10, Ki=10, Kff=4.366 at 7, 10, 13, 15 rad/s:

| Setpoint | SS Vel | SS Error | Overshoot | Settling | Power Range |
|---|---|---|---|---|---|
| 7 rad/s | 6.995 | 0.07% | 14.0% | 0.42s | [58, 100]% |
| 10 rad/s | 9.829 | 1.71% | 8.9% | **0.04s** | [73, 93]% |
| 13 rad/s | 12.874 | 0.97% | 10.3% | **0.04s** | [84, 100]% |
| 15 rad/s | 15.002 | 0.02% | 11.5% | 1.64s | [89, 100]% |

Every setpoint settled. Every setpoint under 2% SS error. Every setpoint under
15% overshoot. The power range at 10 rad/s [73, 93]% confirms the controller
has full authority in both directions — it never hits 100% saturation, meaning
it can always increase or decrease output to correct error.

**Feedforward vs no feedforward at 10 rad/s:**

| Metric | No FF | With FF | Improvement |
|---|---|---|---|
| Settling | 0.56s | 0.04s | 14x faster |
| Overshoot | 10.5% | 8.9% | Lower |
| SS Error | 0.10% | 1.71% | Marginal change |

---

### Part 5: Composite Scoring — How the Sweep Ranks Results

Each combination is scored as a weighted sum of normalised metrics:

```
score = 0.4 × (overshoot / max_overshoot)
      + 0.4 × (settling  / max_settling)
      + 0.2 × (ss_error  / max_ss_error)
```

Lower score is better. Settling and overshoot are weighted equally at 40% each
because both matter for robot motion quality. SS error gets 20% because the
integrator always drives it to near-zero given enough time. The normalisation
ensures no single metric dominates due to scale differences.

---

### Tuning History Summary

| Session | Gains | Result |
|---|---|---|
| Sessions 1–7 | Kp=6.25, Ki=113 | Violent oscillation ±23 rad/s |
| Sessions 8–9 | dt fixed, Ki=30 | OS=8.9%, Settle=0.84s ✅ |
| Session 10 | FF added (double deadband bug) | OS=22–44% ❌ |
| Session 11 | FF corrected | OS=8.2%, Settle=0.06s ✅ |
| Validation | Multi-setpoint 7–15 rad/s | All settled, all under 2% SSErr ✅ |

---

### Final Production Gains

```cpp
#define PID_KP       10.0f
#define PID_KI       10.0f
#define PID_KD       0.0f
#define PID_KFF      4.366f  // 1/K = 1/0.2291
#define PID_DEADBAND 30.0f
#define VEL_MIN_RAD   7.0f   // below this deadband dominates
#define VEL_MAX_RAD  15.0f   // above this output saturates at 7.4V
```

---

### Module Status Roadmap

| Module | Files | Status |
|--------|-------|--------|
| POST / System Health | `lib/POST/post.cpp` | ✅ Stable |
| IMU Driver | `lib/IMU/imu.cpp` | ✅ Stable |
| AS5600 Encoder Driver | `lib/AS5600/as5600.cpp` | ✅ Stable |
| Motor Driver | `lib/Motor/motor.cpp` | ✅ Stable |
| PID Controller | `lib/PID/pid_controller.c` | ✅ Complete |
| MATLAB Sweep | `matlab/pid_autotune_sweep_ff.m` | ✅ Complete |
| MATLAB Live Monitor | `matlab/matlab_live_pid_monitor.m` | ✅ Complete |
| 4-Wheel Integration | `src/main.cpp` | 🔄 Next |
| Mecanum Kinematics | `lib/MoveBase/` | 📋 Planned |
| EKF Localisation | `lib/EKF/` | 📋 Planned |

---

### Files Added or Modified This Session

```
lib/PID/pid_controller.c              — Feedforward term added and corrected
lib/PID/pid_controller.h              — deadband field added to struct
src/main.cpp                          — S:/P:/I:/D:/F:/R: command protocol
                                        7-column telemetry with active gains
config.h                              — Final gains: Kp=10 Ki=10 Kff=4.366
matlab/pid_autotune_sweep_ff.m        — Feedforward-aware sweep script
matlab/matlab_live_pid_monitor.m      — Dark mode, sequence runner, 7-col stream
matlab/CSVs/pid_ff_sweep_summary_*   — Sweep summary tables
matlab/CSVs/pid_response_*           — Validation session recordings
```

### Next Steps

- [ ] **4-wheel integration** — replicate motor_t and pid_t for RF, LR, RR;
      wire up all four encoders through MUX channels 0–3
- [ ] **Mecanum forward kinematics** — compute vx, vy, ω from four wheel
      velocities using the 4×3 mixing matrix
- [ ] **Body-level velocity commands** — MATLAB sends vx/vy/ω, Arduino
      inverts kinematics to per-wheel setpoints, PID tracks each wheel
- [ ] **IMU vibration test under motor load** — characterise noise floor
      increase relative to stationary baseline from 2026-04-01
- [ ] **EKF localisation** — fuse encoder odometry with IMU gyroscope for
      robust heading estimation robust to mecanum wheel slip

