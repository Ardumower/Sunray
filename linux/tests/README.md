# Cloud/control isolation regression tests

On Linux with a C++14 compiler and OpenSSL development libraries:

```sh
linux/tests/run-cloud-tests.sh
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
