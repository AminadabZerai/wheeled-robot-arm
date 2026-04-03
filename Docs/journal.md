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

- [ ] **Finalise CSV Framing** — consolidate `Serial.print` in `main.cpp` to restore
      MATLAB live plotting; one line per sample, 8 columns, single `\n` terminator
- [ ] **Rollover Math** — implement 12-bit → 32-bit "infinite rotation" accumulation
      logic in `as5600_update()` to handle multi-turn tracking without wrap-around jumps
- [ ] **Angular Velocity** — derive rad/s from the encoder delta (Δangle / Δt) for
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

- [ ] **Encoder characterisation session** — static noise floor baseline, then
      velocity sweeps at fixed PWM steps to extract `σ_vx` and quantisation floor
- [ ] **Add 3 remaining encoders** — replicate `enc_LF` pattern for RF, LR, RR;
      add mecanum forward kinematics block to `loop()` computing `vx`, `vy`, `ω`
- [ ] **IMU vibration test under motor load** — characterise noise floor increase
      relative to the 2026-04-01 stationary baseline
- [ ] **Implement `MotorWheel` module** — extract motor control from `main.cpp`
      following the same driver pattern as IMU and AS5600
- [ ] **Implement `PID` module** — reentrant per-motor instances using the same
      struct pointer pattern as the sensor drivers
- [ ] **Wheelbase and wheel radius calibration** — straight-line and rotation tests
      to extract geometric parameters before EKF process model is written
