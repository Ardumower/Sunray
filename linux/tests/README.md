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


## Camera JPEG error recovery

Run `linux/tests/run-jpeg-decode-tests.sh` on Linux with g++, libjpeg development headers, and AddressSanitizer/UndefinedBehaviorSanitizer support. The test covers zero-filled camera frames, malformed headers, every truncation of a generated JPEG, oversized dimensions, RGB/grayscale input, and repeated valid decoding after failures. Corrupt frames must return failure with empty output instead of invoking libjpeg's default process-exit handler.

### Native three-reflector docking

`./linux/tests/run-reflector-strips-tests.sh` builds the Linux camera detector
in isolation (C++11, no image libraries). It covers equal vertical triples,
exposure, camera roll, missing/extra strips, mismatched geometry, ambiguity,
acquisition and expiry, close markers crossing the ROI, short cable occlusions,
large occlusions, clipped near markers and almost full-height charging-contact
markers wider than the former 8% image-width cap, and distant dim strips with
a narrow valid threshold interval between background attachment and loss. Optional `CXXFLAGS` enables AddressSanitizer/UBSan.
For a diagnostic RGB24 frame, append `image.rgb width height`.

The marker consists of three equal-length, equal-width vertical white reflective
strips with equal spacing on a dark backing. Steering follows the middle strip.
All three strips must be identifiable; close-range clipping exceptions are described below. Their lower ends normally lie in
the lower image region; close strips may extend above the ROI anchor. Short
vertical gaps up to 4% of image height are bridged for component connectivity
(e.g. a thin cable), while contrast is checked on the original image. Horizontal
gaps between strips are never closed. Large occlusions and clipping outside those exceptions remain grounds for rejection.
No single-strip fallback is used. Missing or ambiguous markers stop camera-guided
motion; three consistent frames are needed to reacquire. The detector recognizes
the visual pattern, not the physical reflectivity of a material.

Configure `DOCK_REFLECTOR_STRIPS` and its camera, ROI, geometry and steering
settings in the robot configuration. Docking stops within 1 m of the penultimate
dock point (`DOCK_REFLECTOR_STRIPS_HANDOFF_RADIUS`), waits at least 500 ms and
requires a stable marker before moving. There is no rotating search when absent.
Undocking keeps the camera facing the marker until within the same 1 m radius,
then latches back to GPS at the penultimate point and retains the preceding route.
RTK fix and lower-weight float correct position during camera control;
a travel limit stops an undock that misses its exit. Loss/jumps stop both drive
and rotation, including contact advance; reacquisition requires three frames
and at least 500 ms since stopping. The 500 ms stale-image limit still applies.

`DOCK_STOP_ON_OBSTACLE=true` on mscmower bypasses bumper dead time and reverse/
final-dock exceptions and latches Error instead of avoidance. Charger and other
automatic operation transitions cannot release this latch; an explicit operator
command can. The option defaults to false for other configurations.

`sh linux/tests/run-reflector-docking-tests.sh` exercises the pure handoff policy
and extracts the production camera-motion and obstacle/operation methods into
hardware-free fixtures. Tests cover stop/reacquisition, direction, the penultimate-point 1 m entry/exit,
invalid routes, travel limit, timer wrap, charger events, explicit release and
option-disabled behavior. No robot commands are sent by these tests.

The LED detector/configuration and
legacy ROS/LiDAR reflector package have been removed. Real camera recordings
remain outside the repository. No automatic driving test is part of the runner.

### Docking heading

`./linux/tests/run-docking-heading-tests.sh` runs the production
`Map::getDockingPos` method in a hardware-free fixture (Python 3 and C++11).
It covers duplicate and nearly identical endpoints, indexed lookups, all four
heading quadrants, the mscmower route, and invalid routes preserving caller state.

### Operator-confirmed camera entry

`AT+CD` explicitly confirms that the operator has placed the robot at the camera
entry point. Accepted only from Idle/Error with a valid dock route; it starts
Dock and consumes a one-shot flag. It skips GPS approach, holds for 500 ms and
requires stable three-strip detection. Missing markers keep it stationary.
A normal Dock command still uses the mapped first point. The confirmation is
not stored in the map or persisted, and another operation cancels it.

The mscmower supervised test configuration currently uses `DOCK_LINEAR_SPEED=0.02`,
`DOCK_MAX_WHEEL_SPEED=0.02` (m/s per commanded wheel, preserving curvature), and
`DOCK_TEST_TIMEOUT_MS=300000` (local stop in Idle after five minutes). These are
command limits, not a guarantee of physical wheel speed. The camera detector is
compiled with `-O2` even in a Debug build so fresh observations can arrive within
the unchanged 500 ms deadline. On the Orange Pi, a saved first-point frame took
about 268 ms at `-O0` versus 72 ms at `-O2` (12 repetitions, decoding excluded).

### Low-speed fractional motor control

The motor startup fixture also exercises the production controller at +/-0.02 m/s
with stationary encoder feedback. Integer PWM accumulation previously truncated
every sub-unit PID increment: with the XL-mower gains the output remained at
+/-1 indefinitely. The controller now keeps its PWM state as float, clamps without
integer casts, and converts only at the driver boundary. The regression checks
forward/reverse buildup, saturation, response to simulated overspeed and zero
reset, in addition to the existing coasting/pivot/timeout/GPS-interlock tests.
This is an offline control test; friction, physical speed and low-speed stability
still require a separate supervised test on the robot.

### Reflectors touching the frame at charging contact

The detector permits top/bottom clipping only when each of the three visible
strips spans at least 70% of the image height. Outside the close-range case below, side clipping is rejected;
width, slope, common overlap, spacing, dark gaps, ambiguity, three-frame
acquisition and immediate loss handling remain checked. Synthetic tests cover
clipped ends, cable interruption, missing/extra strips, short segments and side
clipping. A 20-frame original MJPEG capture at charging contact on 2026-10-17
reproduced five rejected frames before this fix and zero afterwards. The prior
20-frame first-point capture and saved near/contact images still pass, while the
old single-LED image remains rejected. Stream overlays are not detector input;
use original captures for replay rather than annotated cloud screenshots.

At even closer range, fully visible outer strips can have less dark margin than
the preferred 0.6-strip-width contrast offset. Contrast samples now clamp to
the available image border; ordinary components remain at least
two pixels inside both sides (partial outer strips use the close-range rule below). This fixes the second 20-frame contact capture
(0/20 before, 20/20 after) without changing width/aspect/spacing or acquisition
thresholds. Synthetic narrow-margin, missing-middle, side-clipped and missing
outer dark-edge cases cover this boundary condition.

### Supervised undock-only test configuration

`UNDOCK_TEST_TIMEOUT_MS=300000` temporarily blocks blade activation in
`Motor::setMowState` and stops MowOp before tracking when the deadline expires,
camera undocking completes, or the route is no longer an undocking route.
`DOCK_MAX_WHEEL_SPEED` also caps each wheel during undocking. The production
stop block is exercised by the docking safety fixture; the production Motor
fixture checks the reverse wheel cap and blade block. Remove/disable the
undock test macro to restore normal mowing after validation.

Reverse encoder travel is calculated as the shorter unsigned modular difference
before conversion to float. Subtracting a larger starting count previously
underflowed and immediately tripped the undock distance bound. The gate fixture
covers forward/reverse one-tick motion, no motion and wrap in both directions.

An ordinary Mow start resumes camera undocking beyond the 1 m exit radius when
the robot is within 0.75 m of the final nondegenerate dock-route segment and either has
the dock route retained or faces the station within 45 degrees. Idle preserves
an unfinished undock route. Geometry allows recovery after a restart without a
special command. Missing markers never select GPS as a fallback: the normal
camera acquisition gate holds the robot stationary. Normal path planning still
prepares the continuation from the first dock point. The corridor/exit/facing
policy and duplicate-segment rejection are covered by the gate tests.

### Partially visible outer reflector at extreme close range

A close strip spanning at least 85% of image height may be up to 22% of image
width with a 2.5:1 aspect ratio. Exactly one outer strip may touch a side border
when all three span 85% of height and at least 60% of the opposite outer strip's
width remains visible. Its missing width is inferred from that opposite strip
for spacing checks; steering continues to use the fully visible middle strip.
A clipped middle, missing third strips and ambiguous four-strip groups remain rejected.
The full-height exception for both outer sides is described below. Contrast is required on the visible inner
edge; common overlap, dark gaps and temporal/loss guards remain active.

The 20-frame original capture after contact re-touch now passes 20/20, alongside
64 earlier replay images (including the rejected single-LED negative). Synthetic
left/right clipping, insufficient visible width, missing/extra-strip cases also
pass with address/undefined-behavior sanitizers. XL-mower keeps
`DOCK_RETRY_TOUCH` enabled as described below; its legacy Charge recovery is
independent of the camera docking guards.

### Leaving the side border and dim lighting

Extreme close triples (all three visible heights >=85%) compare the outer
widths with each other and the middle width with their mean, using the existing
1.6 ratio. This handles perspective/lens distortion consistently when an outer
strip becomes fully visible during undocking; previously that transition fell
back to min/max width and lost the marker after about 1 cm. Shorter markers
retain the original min/max-width rule. Synthetic tests cover the transition
at multiple brightness levels and reject unequal outer/excessive middle widths.

The absolute brightness/threshold/edge-contrast floors are now 12/6/4 instead
of 20/12/8 (8-bit intensity); relative contrast, geometry, ambiguity and temporal
checks remain required. Tests include dim triples with deterministic noise,
pure sensor noise and insufficient light. Simulated first-point images at 10%
brightness with noise sigma 2 pass; sigma 5 at that brightness is still rejected.
This is not a substitute for real twilight validation with camera exposure,
motion blur and actual reflector illumination.

### Original failure-frame recording

`CameraStreamer` records original camera JPEGs (before resize/overlay) in
`linux/camera_failures/`. Each image has JSON metadata with wall/monotonic time,
source dimensions, detection result and requested linear/angular velocity.
A found-to-missing transition stores one failure image (at most 4 Hz).
Startup without a marker and persistent loss/night darkness write no images.
A separate writer thread uses a
four-frame queue, dropping diagnostics rather than blocking detection. There
are 128 fixed slots, at most 2 MiB per JPEG (256 MiB plus small metadata), reused
across restarts. No upload occurs. Missing/unwritable storage does not interrupt
camera processing. `sh linux/tests/run-reflector-failure-recorder-tests.sh`
checks original bytes, transition-only recording, startup/night silence, metadata,
rotation, restart and errors.
The 60-frame stationary capture after the 2026-10-17 contact flicker passes
60/60; the moving failure itself needs this recorder to establish its cause.

### Contact-motion false ambiguity (recorded 2026-10-17)

The recorded docking run reached Charge after 244.6 s. Eight failed frames at
contact replayed as ambiguous: bright scenery at the left image border plus
left/middle reflectors produced a second triple at a low threshold. Side-clipped
outer candidates now require ALL three strips to touch a top/bottom image edge,
in addition to the existing 85% span rule. This preserves the extreme close
clipped marker while excluding border scenery beside fully visible reflectors.
Synthetic mirrored bright-border scenes at multiple brightness levels cover
this distinction; the four-strip ambiguity and genuine clipped-marker tests
remain active. No temporal grace period or motion gate was relaxed.

Validation: all 177 original positive frames (including 17 recorder images)
and four older positive/negative snapshots match expectations. All eight newly
recorded losses now pass. The 17 recorder images also pass all 153 combinations
of brightness 50/20/10% and independent Gaussian RGB noise sigma 0/2/5 (8-bit,
seed 71, original decoded RGB, no additional JPEG encoding). This simulates
reduced signal and noise, not actual twilight exposure or motion blur; the
previous far-marker limitation at 10% brightness with sigma 5 remains.

### Contact recovery re-enabled

After the successful reflector round trip, XL-mower re-enables
`DOCK_RETRY_TOUCH=true` at the operator's explicit request. The existing Charge
retry behavior is intentionally independent of the camera docking/undocking
rules: front-side 0.05 m/s (rear-side -0.03 m/s), at most five seconds, retry stops
on restored contact. The separate first-contact 0.10 m advance remains enabled.
No Charge retry motion/safety behavior was changed by this config-only update.

### Ignore charger reconnect while Mow/Undock is active

A bounced charging contact during undocking previously invoked the base
`Op::onChargerConnected`, switching Mow to Charge. That also exposed the robot
to Charge-only retry recovery on later contact loss. The base handler now
ignores charger-connected events throughout OP_MOW, including undocking;
normal Idle-to-Charge and Dock contact advance remain unchanged. Production
handler tests cover both ordinary mowing and undocking, plus Idle charging.

### Both outer reflectors clipped after retry touch

The 2026-10-17 retry-touch recording fills the frame with all three reflectors,
but clips both outer sides. A new strictly full-height case permits that only
when all three components touch both top and bottom, each outer visible width
is at least half the middle width, and outer widths agree within the existing
size ratio. Effective outer width is bounded below by their visible widths and
middle-width/size-ratio; only the middle's directly measured centre steers.
Single-side clipping also permits the observed outer width to be larger than
the opposite width within the same ratio (perspective), without shrinking it.
All dark-gap, spacing, slope, ambiguity, acquisition and immediate-loss checks
remain in place. Existing tests that rejected valid full-height triples solely
because both sides touched the border now expect detection; new tests reject
tiny outer remnants, shorter middle components, missing/extra strips and width
mismatch. The earlier bright-background contact ambiguity remains covered.

Validation: 239 original positive frames plus four older positive/negative
snapshots pass, including all 62 retry-recorder images. All 558 simulated dim
variants of the retry series pass: brightness .5/.2/.1, Gaussian RGB noise sigma
0/2/5, seed 71. The 20 far-point images pass 20/20 at brightness .2 at each noise
level and at .1 with no noise; at .1 with sigma 2 they pass 19/20, with sigma 5
2/20. These are simulated limits, not a guarantee for real twilight conditions.
Synthetic tests pass ASAN/UBSAN and the configured double-literal size ratio.

Supervised round trip on 2026-10-17 with this detector: undock completed in
242.7 s, dock reached Charge in 223.1 s, with no recorded detection losses.
The 0.10 m contact advance completed; subsequent retry touch restored contact
after three seconds without detection loss. Final Charge and zero wheel PWM
remained stable. No physical bumper actuation was part of this round trip.

### Native Linux build only

The ROS workspace, ROS-only LiDAR drivers, relocalization operation and AprilTag
input path have been removed. Localization and event IDs retain their historical
numbers. Linux audio helpers now live in `linux/scripts/`; the service wrapper
and event logger use those paths. Native camera reflector guidance is unchanged.

### Keep mowing progress across map uploads

`sh linux/tests/run-map-upload-progress-tests.sh` exercises production upload
methods with hardware-free storage fixtures. Before the first upload point
clears the map, the current mowing index and an ordered 64-bit fingerprint of
mowing coordinates (centimetre protocol precision) are saved. Only a completed
upload with the same mowing count and fingerprint restores the index. Changes
to dock/perimeter/exclusion points do not reset an unchanged mowing route.
Changing/reordering/removing mowing points, invalid prior indices, explicit map
clears and abandoned uploads reset progress. No stable waypoint IDs are sent by
the protocol, so moved mowing points cannot safely retain progress merely because
the count is unchanged. Tests cover uploads with and without exclusions and the
final WAY_FREE count, which must not reset the restored index after completion.

Validated on mscmower (2026-10-17): a full re-upload of the active 376-point
mowing route retained index 165 (43.9%). Operation remained Charge, wheel PWM
stayed zero and encoder counts stayed unchanged. The build and offline tests
also passed with AddressSanitizer/UndefinedBehaviorSanitizer.

### GPS position during camera guidance

`run-reflector-docking-tests.sh` also tests `CameraGpsPosition`: three consecutive
fresh solutions of the same quality are required; invalid values, gaps over one second
and implausible successive jumps restart acquisition. Fixes more than 2 m from
the estimated position are rejected. Accepted fixes correct XY at up to 0.1 m/s (maximum 2.5 cm per sample).
Float corrects at up to 0.05 m/s, with a one-second smoothing time constant.
Both are independent of camera steering and IMU yaw. Quality changes restart
three-sample acquisition; float does not refresh the last-fix timer.
Without accepted fixes, encoder/IMU localization continues. The penultimate-point
undocking exit uses this corrected position. Charging-contact localization is
unchanged. Logs report `gpsPos=0/1/2` for inactive/float/fix correction.

Live mscmower validation before the penultimate-point and float changes (2026-10-17,
2 cm/s): undock stopped at the 2 m
first-point radius after 158.5 s, without the former decimetre-scale position
jump at handoff. Camera docking reached Charge after 158.3 s. Logs contain 307
marker samples, no detection losses or Error transitions, and 249 samples with
RTK correction active. Float disabled RTK correction near the station. Fixed
docking telemetry agreed with the reported GPS XY within 1 cm (this is fusion
agreement, not a measurement of physical accuracy). Contact remained present
after the 10 cm advance; wheel PWM returned to zero. Mowing index stayed 165.
One excessive encoder delta was rejected by the existing odometry check.

### Penultimate docking handoff

Both automatic handoffs now use point `dockPoints.numPoints-2` and the shared
`DOCK_REFLECTOR_STRIPS_HANDOFF_RADIUS=1.0` m setting. Passing an earlier waypoint
no longer forces camera entry. After camera undocking, GPS resumes at that point
and follows preceding docking points. Explicit operator-confirmed camera entry
remains available. The supervised undock test still stops at handoff instead of
continuing into GPS driving/mowing. Offline tests cover entry/exit boundaries,
two-point routes, ignoring the first point, stable transitions and marker loss.

### Camera heading correction

Camera-mode yaw is no longer permanently isolated from GPS. A separate estimator
uses fresh, accepted RTK fixes only, at least 25 cm GPS displacement and 20 cm
encoder travel, consistent forward/reverse direction, and a straight track.
Each independent track corrects yaw by at most 30 degrees. Float, standstill,
turns, GPS jumps and stale data reset acquisition. Camera steering still uses
only image error; GPS yaw is not fed into that control law. At charge contacts,
the final dock-segment heading remains authoritative. Offline tests cover the
observed wrong initial heading, reverse travel, angle wrap and rejection cases.

### Automatic camera entry during docking

An ordinary Dock command, including retries from Error and commands deferred
until startup calibration finishes, can select camera acquisition without
routing back through the approach points. The same decision is checked every
Dock approach cycle, after the bumper stop check. It requires a fresh, stable
three-strip detection and a position within 2 m of the finite final dock segment
(penultimate to last point). Image position supplies the orientation reference;
drifting map yaw must not reject a visible marker. Entry stops immediately and
retains the configured acquisition interval. At the image edge, the existing
camera controller rotates without translating until the marker is centered.
An active acquisition/loss gate or blocked state is never reset by this check.

Mow/undock accepts the same 2 m marker-based corridor outside the 1 m handoff
radius around the penultimate point. Existing narrow 0.75 m corridor recovery
without a visible marker waits for camera acquisition. Camera losses and bumper
stops remain unchanged. The XL configuration now disables both temporary test
timeouts; normal RTK/mowing continuation is enabled, with camera speed still
limited to 0.02 m/s.

The GPS translation watchdog refreshes its baseline during commanded pure
rotation, which does not require GPS displacement. Forward travel still has a
full observation interval and triggers on insufficient motion. The separate
rotation detector configuration is unchanged (disabled on XL).

`run-reflector-docking-tests.sh` exercises the production entry decisions and GPS
motion watchdog, including 2 m boundaries, late marker acquisition, image-edge
and invalid observations, map-yaw disagreement, undock handoff, stop/loss gates,
three minutes of pure rotation and subsequent blocked/moving translation.
No test sends real robot movement commands.

### Marker search versus GPS approach

Without a visible marker, Dock starts an in-place scan only when the position
projects onto the final segment between handoff and station and is within 2 m
laterally. Otherwise it plans the ordinary route to the first dock point. Search
has zero commanded translation, a 500 ms initial stop, and a 2 cm/s wheel cap.
A fresh stable marker stops the scan and enters the existing camera acquisition.
It stops in Idle after one measured positive turn, 15 seconds without angular
progress, a yaw discontinuity, or a five-minute upper bound. Bumper and physical
obstacle checks precede scanning; no automatic retry occurs after a failed scan.
The scan is not restarted on camera tracking loss.

XL uses `DOCK_APPROACH_MAX_WHEEL_SPEED=0.10` m/s for GPS approach/exit and retains
`DOCK_MAX_WHEEL_SPEED=0.02` m/s for camera guidance/search. Previously the camera
wheel cap unintentionally applied to the entire GPS approach, reducing forward
progress especially during strong steering. This change keeps the GPS no-motion
watchdog enabled for translation. Tests execute the production search path and
wheel-limit selection in addition to the search angle/time/geometry policy.

The automatic 1 m handoff also requires the route to have reached the
penultimate target index. Earlier docking points inside the same circle do not
trigger camera acquisition. At this route handoff, a missing marker starts one
bounded search even on the approach side within the tolerance circle. Search
availability is consumed by a scan or valid camera acquisition, so subsequent
tracking loss only stops/reacquires. Regression coordinates reproduce the XL
first/second points overlapping the penultimate point's 1 m radius.

### Distant marker minimum height and background bridging (2026-10-17)

The flapping far-view marker had thresholded strip heights of 18/19 pixels at
640x360, below the previous 5.5% floor (~20 pixels). The floor is now 4% (~15
pixels). A second pass with at most 2 pixels of vertical gap filling runs only
when the ordinary 4%-height closing finds no marker. It avoids connecting small
reflectors to scenery below. Ambiguous ordinary detections do not invoke the
fallback. Geometry, contrast, three-frame acquisition and loss stops are intact.

The preserved 128-frame ring snapshot improves from 52 to 119 detections,
including all 114 frames of the reported stable far-view scene. Nine other
frames from the turning sequence remain rejected; this does not claim universal
recognition of blurred/edge views. All 239 older originals and four additional
positive/negative snapshots pass. The older 62-frame twilight set passes all
nine gain/noise variants. Synthetic tests cover threshold-shortened distant
strips, excessive shortening, background bridging and the existing negative
patterns. Strongly darkened distant frames still lose usable contrast; stop-on-
loss is deliberately retained.

Original JPEGs, ring metadata and replay results are preserved externally in
`reflector-datasets/2026-10-17/distant-marker`. The ring was copied while recording,
so JPEG/metadata pairs spanning an overwrite must not be treated as synchronized.

Camera navigation overlay (no robot commands):

```sh
python3 linux/tests/test-navigation-overlay.py
```

Exercises the production snapshot publisher for RTK fix/float, missing GPS,
marker acquisition/search/loss, camera drive, odometry and non-automatic modes.
Renders all labels at 160/320/640 pixels with address/undefined-behavior sanitizers.
An optional existing output directory saves PPM previews.

Asymmetric contact cropping: both outer strips may be clipped by different
amounts when all three span the full image height. At least one outer segment
must retain half the centre width and the other 18%. For this full-height, both-sides-clipped case only,
outer/centre slope differences up to 0.20 are accepted (normally 0.12). Contrast is measured
on visible cores; reconstructed centres are used only for spacing. Synthetic
cases include mirrored views and 10/20/100% brightness, with missing-strip,
tiny-remnant and ambiguous-four-strip negatives retained.

Dock approach regression: command-time camera entry requires the station side
of the penultimate point. Once RTK approach has started, only the route-aware
handoff gate may switch to camera; the marker-visible shortcut is excluded.

Passive light telemetry: see [CAMERA_LIGHT_LOGGING.md](CAMERA_LIGHT_LOGGING.md)
for storage, field definitions, limits and the Linux logger test command.


### Shaded reflectors at dawn (2026-10-18)

Bright scenery beside the charging station pushed the old minimum threshold
(25% of the brightest neutral pixel) above the shaded strips. A third pass
tries 5%, 7.5%, 10%, 15% and 20% only when both existing passes find no marker.
The absolute brightness/contrast floors, colour-neutrality, triple geometry,
ambiguity handling, three-frame acquisition and immediate stop-on-loss remain.
Existing or ambiguous ordinary detections never enter the new fallback.

The saved morning ring improves from 0/127 to 127/127 readable images. One
additional JPEG was truncated while copying the live ring and is excluded;
metadata pairing across the live overwrite is not assumed atomic. A replay of
440 older JPEGs improves 291 to 294 detections with zero previous detections
lost. Synthetic tests include dim strips next to a bright patch, incomplete or
ambiguous triples, coloured objects and darkness/noise; docking safety tests
also pass. This is offline image replay, not a physical docking test.

Morning originals and replay results are retained outside version control in
`diagnostics/reflector-dawn-20261018/` with a provenance manifest. The independent
camera light logger remains disabled on MSCMower.


### Dawn chroma fragmentation while undocking (2026-10-18)

The next 46 loss-event JPEGs show a blue cast and chroma noise splitting the
middle reflector. The low-light fallback additionally classifies the pooled
RGB sample, permitting up to 50% channel spread only below intensity 128.
The two ordinary passes retain their per-pixel 35% neutrality rule. The fallback
checks all exposure levels and requires side-clipped triples to span both image
boundaries vertically, preventing bright edge scenery from creating a second
triple. No missing stripe is inferred; ordinary ambiguity remains a rejection.

All 46 saved loss images change from no detection to a valid triple in offline
replay. Synthetic tests cover blue-cast triples with two/three/four strips and
strongly coloured distractors. Originals and results are stored separately in
`diagnostics/reflector-dawn-20261018/moving-loss/`. These are copied image/metadata
pairs checked for unchanged metadata during the copy, not annotated screenshots.

The final fallback keeps all previous detections across 568 archived/initial
morning images (one known corrupt file excluded) and detects one additional
older distant view. On the Orange Pi, replaying the 1920x1080 loss frame takes
127.4 ms per detection on average, below the unchanged 500 ms freshness limit.

### u-blox signal diagnostics

Run `sh linux/tests/run-ublox-signal-tests.sh` for NAV-SIG C/N0 statistics,
strongest-signal selection, no-reception and malformed-message checks.
The existing `run-gps-startup-tests.sh` builds the full UBLOX parser.

Periodic MON-COMMS output now includes `UBX-SIGNAL C/N0[dB-Hz]`:
`nonzero` counts signal entries with C/N0 > 0 (not unique satellites),
`used` counts pseudorange-used entries, and `avgNonzero` excludes zero entries.
`top8[gnss:sv/sig=cno]` lists at most eight strongest satellite/frequency signals.
`ageMs` describes the last valid NAV-SIG reception; age > 10000 ms is marked
`STALE`. Missing NAV-SIG is explicitly unavailable. No navigation thresholds
are changed. Statistics are reset on receiver initialization/configuration.

On Linux, the same interval also logs `WLAN-SIGNAL iface=... rssi[dBm]=...`
for each interface in `/proc/net/wireless`, using read-only driver statistics
(`SIOCGIWSTATS`). Noise is included when valid. Missing interfaces, disconnected
links and unsupported/invalid statistics are explicitly unavailable. No Wi-Fi
scan, external command or additional library is needed; driver-reported values
may be averaged or cached by the driver.
