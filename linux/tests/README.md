# Cloud/control isolation regression tests

## Linux monotonic control timers

Run `sh linux/tests/run-monotonic-time-tests.sh` on Linux. This compiles the
production `wiring_main.cpp` with hardware initialization disabled. Process-local
clock wrappers simulate forward/backward system-clock steps (including the
observed +2281 s NTP correction); the system clock itself is never changed.
Checks cover concurrent first use, the shared millis/micros epoch, precision,
GPS/PID deadlines, long uptime, and a separate run with the real monotonic clock.

Linux `millis()` and `micros()` use `CLOCK_MONOTONIC` with one immutable epoch,
initialized at first use and retained through main(). They no longer follow
wall-clock jumps. The unsigned-long API and its platform-specific wrap width are
unchanged. Calendar time and MCU implementations are unaffected. CLOCK_MONOTONIC
does not count system suspend; ordinary NTP frequency slewing can still adjust
its rate slightly, but cannot introduce a discontinuous wall-clock step.

## Cloud/control tests

On Linux with a C++14 compiler and OpenSSL development libraries:

```sh
linux/tests/run-cloud-tests.sh
sh linux/tests/run-gps-cloud-tests.sh
```

The worker test blocks connect, receive, send, close and camera transfer while
performing 1,000 control-side polls. No network operation may execute on the
control thread. It also checks request expiry, session identity and cached board
identification. Each 1,000-poll group must complete within 50 ms.

The TLS test uses the production BridgeSecureClient and a local TCP peer that
starts, but never finishes, a TLS record. Connection setup must time out while
control-side calls continue. This exercises the distinction between readable TCP
data and a complete TLS record, without connecting to a robot or cloud service.

Observed on Orange Pi 5 (2026-10-02): about 2.1–2.2 seconds transport stall,
more than 400 control polls, worst observed poll 1 ms. These are measurements,
not a hard real-time guarantee or a complete field WiFi-outage test.

## Architecture and scope

Linux CloudWorker is the only caller of WebSocket/TLS operations. It exchanges
one request/response at a time with the control thread using try-locks. Commands
still execute on the control thread; no robot state is accessed by the network
thread. Pending requests expire after 500 ms, and session state is discarded on
reconnection. Camera traffic retains only the latest frame (maximum 1 MiB).
Network socket operations use a two-second timeout; SIGPIPE is blocked in the
worker. DNS resolution is isolated but may take longer than a socket timeout.

The version command caches a bounded device-tree file read, replacing a shell
process plus Stream::readString, which waits 1,000 ms at EOF.

This change covers the Linux cloud WebSocket path. Direct HTTP, BLE, NTRIP and
MCU network implementations are not converted to this worker. Explicit robot
commands (including Stop) and sensor safety decisions continue to apply.

The separate Linux WiFi LED/status probe is also isolated from the control
thread by `LinuxWifiStatus`. Its test blocks the probe for 3.4 seconds while
polling the cached state, checks state transitions, and exercises the real shell
timeout/parser using a temporary fake `wpa_cli` (no WiFi changes).

## GPS during a stalled cloud connection

`run-gps-cloud-tests.sh` compiles the production UBLOX parser and CloudWorker in
an isolated temporary source tree, without changing the robot's `config.h`.
An in-memory serial receiver supplies checksummed UBX-NAV-RELPOSNED packets at
5 Hz while connect, receive, send, close and camera transfer are each stalled
for 3.4 seconds (longer than the GPS watchdog). FIX and advancing iTOW must be
preserved in every phase. No physical device or external network is used.

The test also checks receiver-reported INVALID/FLOAT, corrupt checksums, silence
past the GPS timeout and recovery. It uses the real SparkFun library but disables
hardware configuration (`GPS_CONFIG=false`); it does not verify receiver/radio
configuration, the complete robot loop, or physical WiFi loss.

## Cooperative u-blox startup and reconnect

Run `sh linux/tests/run-gps-startup-tests.sh` on Linux. It builds the production
UBLOX parser/configuration and Motor implementation with `config_xlmower.h` in
an isolated directory. UART, clock, motor driver and cloud transport are fakes;
no robot is controlled. Checks cover absent/late receivers, cloud request
handling during initialization, all ten RAM-only configuration groups, radio
RTCM settings, fallback baud, optional MON-VER timeout, NAK/missing/invalid ACKs,
bounded RX/TX, clock rollover, position watchdog, USB generation changes,
silence-triggered recovery, and motor/brake inhibition including stale commands.

`GPS_CONFIG=true` now schedules configuration in `UBLOX::run()`. Each poll writes
at most 32 bytes and parses at most 1024 received bytes. Probe/ACK/TX timeouts
are 2 seconds; configuration groups have two attempts, with 300 ms between
successful groups. A failed setup retries after 10 seconds while the main loop
continues. Configuration replies share the navigation parser; there is no
second UART reader/thread. `configure()` reports acceptance, not completion;
`isConfiguring()` reports the pending state. Navigation remains INVALID until
setup finishes and a fresh solution arrives. Motion and cutter output are
inhibited during setup, and old motor commands are cleared.

On Linux, a detected serial disconnect/reopen changes `connectionGeneration()`
and restarts configuration. Ten seconds without a checksummed UBX message also
triggers a retry; loss of RTK FIX with continuing UBX traffic does not. With
`GPS_CONFIG=false`, startup and reconnect send no automatic configuration.
The existing GPS/cloud test verifies that disabled-config behavior explicitly.
Raw/RTCM writes are suppressed during setup so they cannot split a CFG packet.
The former UART1 TIMEUTC-disable call mistakenly used the timeout as the value;
the replacement packet explicitly sends zero.

Validated on an Orange Pi 5 Pro, 2026-10-03: startup/reconnect/motor
regressions, GPS/cloud regression (five 3.4-second stalls; 17 positions each,
worst measured poll 1 ms), and complete XL-mower firmware build passed.
Receiver responses in the regression suite are emulated. A physical F9P USB
unplug/replug also passed: disconnect at 18:18:55.705, reopen at 18:19:04.707,
and configuration complete at 18:19:08.817 (Europe/Berlin). iTOW advanced after
recovery and control-duration samples remained 0.02 s. This verifies USB/setup
recovery, not an RTK FIX or mowing/navigation behavior. A live cloud handshake
also completed while GPS configuration was still in progress (18:43:25 versus
18:43:27). Cloud availability still requires a working network and valid,
unique connection credentials.
