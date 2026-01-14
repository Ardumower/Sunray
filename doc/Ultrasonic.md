# Ultrasonic Sensors (Sunray + owlController)

This document explains wiring and usage of ultrasonic sensors for two variants:

1) owlController (RP2040) with DYP-A22 sensors over CAN.
2) Direct connection on the Sunray board (HC-SR04 compatible).

Note: The CAN variant provides left/right only. A center sensor is not supported there yet.

## Variant A: owlController + DYP-A22 (CAN)

### Wiring
- Sensor: DYP-A22 (I2C).
- An I2C multiplexer is used. MUX channels and addresses are defined in `owlController_RP2040_00/config.h`:
  - `dyp1_I2CMuxChn` (left), `dyp2_I2CMuxChn` (right)
  - I2C addresses: `0x74` and `0x75` in `owlController_RP2040_00/DYP_A22.h`
- Wiring goes to the owlController I2C/MUX pins on the PCB.

### Firmware (owlController)
- Measurement is started in `owlController_RP2040_00/owlController_RP2040_00.ino`.
- Distance values are filtered and sent as CAN values `can_val_ultrasonic_left/right`.

### Usage in Sunray
Enable the CAN ultrasonic obstacle logic in the chosen config:
- `#define CAN_SONAR_TRIGGER_OBSTACLES 1`
- `#define SONAR_ENABLE true` (enables polling)
- `#define SONAR_TRIGGER_OBSTACLES true` (enables obstacle triggers)
- Thresholds: `SONAR_LEFT_OBSTACLE_CM`, `SONAR_RIGHT_OBSTACLE_CM`
- Poll interval: `SONAR_POLL_INTERVAL_MS`

If the distance drops below the threshold and `CAN_SONAR_TRIGGER_OBSTACLES` is enabled,
the obstacle logic triggers (see `Sunray/sunray/robot.cpp`).

## Variant B: Direct connection (HC-SR04 compatible)

### Wiring
- Sensor: HC-SR04 or compatible.
- Pins are defined via `pinSonarLeftTrigger`, `pinSonarLeftEcho`,
  `pinSonarCenterTrigger`, `pinSonarCenterEcho`,
  `pinSonarRightTrigger`, `pinSonarRightEcho` in the chosen config.
- Sensors use trigger/echo lines. Ensure 3.3V levels if required by your board.

### Usage in Sunray
Enable in the chosen config:
- `#define SONAR_INSTALLED 1`
- `#define SONAR_ENABLE true`
- `#define SONAR_TRIGGER_OBSTACLES true`
- Thresholds: `SONAR_LEFT_OBSTACLE_CM`, `SONAR_CENTER_OBSTACLE_CM`, `SONAR_RIGHT_OBSTACLE_CM`

The logic lives in `Sunray/sunray/sonar.cpp` and is used by obstacle detection
(`Sunray/sunray/robot.cpp`).

## Validation notes
- For CAN sonar, the log prints "can sonar obstacle!" when triggered.
- For local sonar, the log prints "sonar obstacle!".
- The counter `statMowSonarCounter` is incremented in both variants.
