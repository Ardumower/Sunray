# Direct browser camera and joystick link

Linux Sunray can use an encrypted WebRTC data connection alongside its existing
Cloud connection. The browser automatically probes it, using authenticated Cloud
signaling. No IP scanning, local HTTP certificate exception, STUN or TURN relay is
required. Only reachable interface candidates are used (LAN or a routed VPN).
Guest/client-isolated Wi-Fi, blocked UDP, or browser restrictions can prevent a
direct connection; normal Cloud control and camera streaming remain available.

The Cloud UI uses this transport for both Operate and the separate Joystick page.
A small radio icon overlays an active robot camera image only when the direct
connection is healthy. Other operations, including telemetry, map uploads,
Start/Stop and Dock, continue to use Cloud. Browser reload may be needed after an
update. The helper is separate from the RP2040/CAN firmware.

## Installation

Use Python 3.10+ on the Linux robot and a Sunray config with
`#define DIRECT_LINK_ENABLE true` (enabled in `config_xlmower.h`, inherited by the
Cloud simulator; absent/false in other configs leaves the feature inactive).
Build that configuration as usual, then as the normal robot user run:

```sh
bash linux/service.sh direct-setup sunray
```

For the simulator use `direct-setup sunray-sim3d`; the main simulator setup action
already does this automatically. All setup remains in `linux/service.sh`.
The helper installs pinned Python requirements into `tools/direct_link/.venv` and
creates `sunray-direct.service`. A systemd dependency starts it whenever the chosen
Sunray service starts. There is no new browser extension or client installation.

The helper is restarted on failure and retries the local Sunray connection.
Stopping Sunray closes the local bridge and invalidates peer sessions. A helper
failure leaves the existing Cloud connection available. Check:

```sh
systemctl status sunray-direct --no-pager
journalctl -u sunray-direct -n 50 --no-pager
```

The bridge listens only on `127.0.0.1:18891`, signaling RPC only on
`127.0.0.1:18892`. WebRTC uses dynamic UDP ports on the robot's interfaces. Local
processes are trusted; the bridge is not an API to expose on the LAN. RPC rejects
browser-origin requests and requires the Sunray IPC header. Remote access is
negotiated through the existing authenticated, ownership-checked Cloud session;
DTLS fingerprints bind the WebRTC peer to that negotiation.

## Simultaneous local and remote clients

Routing is per browser, not a global robot setting. Up to four direct peers can
coexist with Cloud viewers. Camera subscriptions/configuration still go through
the Cloud's existing channel arbitration. If **any** active camera viewer uses
Cloud, Sunray retains one camera upload for those viewers while delivering frames
to the direct helper too. If all active viewers use direct, the camera upload is
suppressed. Both the browser direct flag and firmware suppression have short
leases, so missing renewals restore the default Cloud route.

This does not add independent camera configurations per viewer: the existing
single-stream/channel-owner rules and raw/texture auxiliary-view support remain.
Profiles and requested JPEG quality/FPS are not automatically increased for local
clients, since that could affect simultaneous external viewers. Camera JPEGs are
split into bounded WebRTC data messages, retaining only the latest pending frame.
Fine-grained queue limits leave room for control/heartbeat traffic.

## Control and fallback

- Control and image data use separate unordered data channels without retransmits.
- Robot-generated heartbeat challenges expire after five seconds to allow the
  outdoor WLAN round trip; drive sequence numbers reject reordered/replayed commands. The local bridge also checks monotonic age.
- One direct peer owns the manual drive lease at a time. Camera-only viewers never
  send a drive command. The Sunray bridge stops motion after three seconds without
  a fresh drive command; disconnect also queues zero. Commands execute on the robot control thread
  through the usual manual-speed/slowdown path. The motor watchdog uses the same
  three-second deadline for direct commands; other command paths retain their
  existing timeout. Local IPC still rejects queued commands older than 250 ms.
- During a direct manual-control lease, delayed Cloud joystick packets are ignored;
  operation-level Stop and other normal Cloud commands are not filtered.
- A transport switch while driving requires the joystick to return to neutral;
  a held direction is never silently replayed over the new route.
- The camera link tolerates up to six seconds without heartbeat; drive validation
  and the three-second motor watchdog remain independent. Short ICE interruptions
  do not force renegotiation, and route changes preserve the last camera image.
  Failed ICE or a stale subscribed camera falls back to Cloud.
  Direct negotiation retries automatically. Signaling/permission renewals use the
  Cloud; this is not a standalone offline app. Peer authorization expires after
  two minutes without renewal.

## Validation

The simulator on Twenty was tested from a Mac browser: direct images without
Cloud duplicates, a simultaneous external Cloud viewer, helper shutdown causing
Cloud fallback, and automatic direct reconnection after helper restart. Tests
use an isolated bridge port. An additional end-to-end WebRTC test on Twenty
verified positive and negative rotation and stopping through the helper, AT
parser and simulated wheels. A two-second command gap preserved rotation;
watchdog expiry stopped it. No physical robot was driven. Direct motor commands
include a validated AT checksum so the parser retains the angular argument.

```sh
# Pure control-bridge tests on Linux, isolated from the production bridge port:
c++ -std=c++14 -pthread -DDIRECT_LINK_PORT=18893 -I linux/src \
  linux/tests/direct-link-test.cpp linux/src/DirectLink.cpp -o /tmp/direct-link-test
/tmp/direct-link-test
# Helper validation, expiry, peer ownership, frame bounds, and RPC-origin tests:
cd tools/direct_link
.venv/bin/python -m unittest test_server
```

Cloud tests cover per-client signaling, simultaneous local/external camera
routing, lease expiry, spectator behavior, and neutral-before-fallback control.
