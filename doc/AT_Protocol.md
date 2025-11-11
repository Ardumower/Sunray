Sunray Firmware AT Protocol

Overview
- Purpose: Text-based control/telemetry protocol used between the Sunray firmware and clients (Sunray App via BLE/WiFi HTTP, and Sunray Cloud via WebSocket).
- Implementation: `sunray/comm.cpp` (dispatcher) with helpers across the codebase.
- Transports: BLE UART, WiFi HTTP server, WiFi relay, WebSocket client. All share the same ASCII protocol line format.

Framing
- Encoding: ASCII text.
- Line format: `AT+<GROUP>[<DETAILS>][,<PAYLOAD>],0xHH` followed by CRLF (`\r\n`) on serial/BLE. For HTTP/WS, the request body or text frame contains the same line; CRLF is optional.
- Responses: Always end with `,0xHH\r\n`. Content begins with a short type tag (e.g., `V`, `S`, `T`, `OK`).

CRC
- Type: 8‑bit additive checksum of every character before the last comma.
- Suffix: Append `,0xHH` with two hexadecimal digits (uppercase or lowercase accepted). Example for `AT+V`: `65+84+43+86 = 278 = 0x16`, so send `AT+V,0x16\r\n`.
- Validation: Required on BLE/WiFi/WS. The interactive console (`CON`) does not require CRC.

Authentication and Encryption
- Handshake: Client sends `AT+V` (with CRC). Firmware replies with version and, when compiled with `ENABLE_PASS`, an encryption mode and challenge.
- Response format: `V,<ver>,<encrypt_mode>,<challenge>,<board>,<driver>,<mcu_fw_name>,<mcu_fw_ver>,<robot_id>,0xHH`.
  - `<ver>`: Build string (e.g., `Sunray,1.0.331`).
  - `<encrypt_mode>`: `0` (off) or `1` (enabled for subsequent commands).
  - `<challenge>`: Random integer 0..199 (nonzero when encryption enabled).
  - `<board>`: Board ID or Linux + model.
  - `<driver>`: `SR` (SerialRobot), `CR` (CanRobot), `AM` (Ardumower), `XX` (unknown).
  - `<mcu_fw_name>`, `<mcu_fw_ver>`: From robot MCU.
  - `<robot_id>`: Unique robot identifier.
- Key derivation: `key = PASS % challenge`, constrained to 1..94 (challenge is regenerated until the result is in range).
- Cipher (requests only): Caesar‑style shift over printable ASCII 32..126 with wrap. For each byte c in that range:
  - Encrypt: `c = 32 + ((c - 32 + key) % 95)`
  - Decrypt: `c = 32 + ((c - 32 - key + 95) % 95)`
- Notes: `AT+V` is never encrypted; all subsequent commands should be encrypted when `encrypt_mode == 1`. Responses are not encrypted.

Command Reference

Version and Capabilities
- `AT+V` → Version/handshake.
  - Request: `AT+V,0xHH`
  - Response: `V,<ver>,<encrypt_mode>,<challenge>,<board>,<driver>,<mcu_fw_name>,<mcu_fw_ver>,<robot_id>,0xHH`

Summary and Obstacles
- `AT+S` → Summary telemetry.
  - Response: `S,<bat_v>,<x>,<y>,<delta>,<gps_sol>,<op>,<mow_idx>,<dgps_age_s>,<sensor>,<tgt_x>,<tgt_y>,<gps_acc>,<sv>,<amps_or_chg>,<sv_dgps>,<map_crc>,<lateral_err>,<tt_day>,<tt_hour>,0xHH`
    - `<bat_v>`: Battery voltage (V).
    - `<x> <y>`: Pose (m).
    - `<delta>`: Heading (rad).
    - `<gps_sol>`: GNSS solution enum (invalid/float/fix; see code).
    - `<op>`: Operation (`0=IDLE,1=MOW,2=CHARGE,3=ERROR,4=DOCK`).
    - `<mow_idx>`: Current mowing point index.
    - `<dgps_age_s>`: DGPS age (s).
    - `<sensor>`: Sensor/error code (see Sensor enum in `sunray/types.h`).
    - `<tgt_x> <tgt_y>`: Current target (m).
    - `<gps_acc>`: Reported accuracy (m).
    - `<sv>` / `<sv_dgps>`: Satellites total / DGPS.
    - `<amps_or_chg>`: Motor current (A) or `-<chg_current>` in charge op.
    - `<map_crc>`: Map CRC.
    - `<lateral_err>`: Lateral tracking error (m).
    - `<tt_day>,<tt_hour>`: Next timetable stop/start: if mowing → autostop day,hour; if charging → autostart day,hour; else `-1,0`.
- `AT+S2` → Obstacles.
  - Response: `S2,<poly_cnt>,[0.5,0.5,1,<n>,x1,y1,...]...,0xHH` (polygons with RGB=(0.5,0.5,1) and their vertices).

Control and Tuning
- `AT+M,<linear>,<angular>` → Set linear (m/s) and angular (rad/s) speed.
  - Ack: `M,0xHH`
- `AT+C,<mow>,<op>,<spd>,<fixTO>,<restart>,<perc>,<skip>,<sonar>,<pwm>,<height>,<dock>` → High‑level control.
  - Params (use `-1` to leave unchanged where applicable):
    - `<mow>`: `0/1` off/on cutter motor.
    - `<op>`: Operation (`0=IDLE,1=MOW,2=CHARGE,3=ERROR,4=DOCK`). `0` has special behavior to immediately stop all motors.
    - `<spd>`: Target mowing speed (m/s).
    - `<fixTO>`: GNSS fix timeout (s).
    - `<restart>`: `0/1` continue after finish.
    - `<perc>`: Set mowing progress percent (restarts current op).
    - `<skip>`: If `>0`, skip next mowing point (restarts current op).
    - `<sonar>`: `0/1` enable sonar/near‑obstacle sensors.
    - `<pwm>`: Max cutter PWM.
    - `<height>`: Cutter height (mm), if supported.
    - `<dock>`: `0/1` dock after finish.
  - Ack: `C,0xHH`
- `AT+CT,<param_idx>,<value>` → Online tuning of control parameters.
  - Indices:
    - `0`: stanleyTrackingNormalP
    - `1`: stanleyTrackingNormalK
    - `2`: stanleyTrackingSlowP
    - `3`: stanleyTrackingSlowK
    - `4/5/6`: motor PID Kp/Ki/Kd (both wheels)
    - `7`: motor low‑pass Tf (both wheels)
    - `8`: motor PID output_ramp
    - `9`: `motor.pwmMax`
  - Ack: `CT,0xHH`

Map Upload
- `AT+N,<peri>,<excl>,<dock>,<mow>,<free>` → Predeclare element counts for each way type.
  - Ack: `N,0xHH`
- `AT+W,<start_idx>,x1,y1,x2,y2,...` → Upload waypoint coordinates in chunks (perimeter, exclusions, dock, mow, free; implicit by sequence configured via `AT+N`).
  - Ack: `W,<next_idx>,0xHH`
- `AT+X,<start_idx>,c1,c2,...` → Upload exclusion polygon lengths (counts per polygon) starting at index.
  - Ack: `X,<next_idx>,0xHH`

Position Source
- `AT+P,<enable>,<lon>,<lat>` → Configure external absolute position source (e.g. app provides GNSS lat/lon).
  - Ack: `P,0xHH`

Statistics
- `AT+T` → Read statistics.
  - Response: `T,<idle_s>,<charge_s>,<mow_s>,<mow_float_s>,<mow_fix_s>,<float_to_fix>,<mow_dist_m>,<max_dgps_age_s>,<imu_recov>,<tmin_C>,<tmax_C>,<gps_chk_err>,<dgps_chk_err>,<max_ctl_cycle_s>,<serial_buf_sz>,<mow_invalid_s>,<mow_invalid_recov>,<mow_obstacles>,<free_mem>,<reset_cause>,<gps_jumps>,<sonar_cnt>,<bumper_cnt>,<gps_motion_to_cnt>,<mow_motor_recovery_s>,<lift_cnt>,<gps_no_speed_cnt>,<tof_cnt>,<diff_imu_wheel_yaw_cnt>,<imu_no_rot_speed_cnt>,<rot_timeout_cnt>,0xHH`
- `AT+L` → Clear statistics. Ack: `L,0xHH`

WiFi (Linux targets)
- `AT+B1` → Scan SSIDs. Response: `B1,<ssid0>,<ssid1>,...,0xHH`
- `AT+B2,<ssid>,<pass>` → Configure WiFi. Ack: `B2,0xHH`
- `AT+B3` → Report local IP. Response: `B3,<ip>,0xHH`

Firmware Update (Linux targets)
- `AT+U1,<url>` → Trigger firmware update via installer script. Ack: `U1,0xHH`

Camera (Linux targets)
- `AT+CAM,<enable>,<index>,<width>,<height>,<fps>[,<quality>]`
  - Starts/stops MJPEG streaming via cloud WebSocket when enabled.
  - Defaults and clamps: width 160..640, height 120..480, fps 1..10, quality 10..95.
  - Ack: `OK,0xHH`

Diagnostics and Developer Tools
- `AT+E` Motor test; `AT+Q` Motor plot; `AT+F` Sensor test. Acks: `E`, `Q`, `F`.
- `AT+O` Simulate obstacle; `AT+O2` simulate rain; `AT+O3` simulate low battery. Acks: `O`, `O2`, `O3`.
- `AT+G` Toggle GNSS solution (invalid→float→fix) for testing. Ack: `G`.
- `AT+K` Kidnap test (zero pose). Ack: `K`.
- `AT+Z` Pathfinding stress test. Ack: `Z`.
- `AT+Y` Trigger watchdog reboot; `AT+Y2` GNSS reboot; `AT+Y3` power off (where supported). Acks: `Y`, `Y2`, `Y3`.

Operation and Sensor Enums
- Operations (`<op>` in `AT+C` and `<op>` in `AT+S`): `0=IDLE`, `1=MOW`, `2=CHARGE`, `3=ERROR`, `4=DOCK`.
- Sensor/error codes (`<sensor>` in `AT+S`): See `sunray/types.h` for full list (e.g., `SENS_NONE`, `SENS_OBSTACLE`, `SENS_GPS_FIX_TIMEOUT`, `SENS_IMU_TILT`, `SENS_RAIN`, ...).

Example Flow (BLE/WS)
- 1) Version/handshake: `AT+V,0x16\r\n` → `V,Sunray,1.0.331,1,73,Linux Raspberry Pi 5,SR,<mcu_fw>,<mcu_ver>,<id>,0xHH\r\n`
- 2) Encrypt subsequent requests using key `PASS % challenge` (printable ASCII shift), append CRC, then send.
- 3) Poll `AT+S` for telemetry and/or `AT+T` for statistics; issue `AT+C` to control operation.

Notes
- Timetable upload (`AT+TT`) exists for mowing hours and may change; see `cmdTimetable` for details of 24 hour day masks.
- All numeric units are those used internally (meters, seconds, radians, amps, degrees C), unless noted.
- For chunked uploads (`AT+W`, `AT+X`), the ack returns the next start index to continue from.

