# Passive camera light diagnostics

While native camera capture is running, firmware appends one JSON record about
every second to `linux/camera_light/YYYY-MM-DD.jsonl` (UTC date). Each camera
session restarts its rolling averages. Original MJPEG reference images are
saved at session start and approximately every five minutes; `reference_jpeg`
links the image to the record. These are separate from the existing bounded
`camera_failures` ring and are not automatically overwritten or deleted.

Disk writes run on a separate thread with an eight-record queue. New diagnostic
records are skipped if the queue is full, and disk writes are suspended below
256 MiB free space. A typical 250 KiB JPEG every five minutes uses about 70 MiB
per day, plus roughly 60–100 MiB/day of JSON. Archive this directory periodically.
No log is produced while capture is disabled or cannot open the camera. During
an open stream, a one-second select timeout produces an image-invalid record.

## Stop and restart logging

Create `linux/camera_light/disabled` and restart firmware while stationary to
stop this logger without disabling camera capture or marker detection. Remove
the file and restart firmware to collect a new session. Existing data remains.

## Fields (schema 1)

- `unix_ms`: UTC wall clock for comparison with saved failure images;
  `monotonic_ms`: monotonic timestamp for within-boot timing.
- `camera_index`, `nav_mode`: CameraRegistry index and NavigationOverlay enum
  (0 hidden/non-auto, 1 RTK fix, 2 float, 3 GPS wait, 4 camera, 5 camera wait,
  6 camera search, 7 camera stop, 8 odometry, 9 guidance).
- `window_s`: actual measurement interval, which can exceed one second.
- `capture_fps`: all dequeued buffers / interval (including invalid buffers).
  This is the application's received rate, not the requested/cloud frame rate.
- `buffer_timestamp_fps`: reciprocal mean positive interval between capture
  buffer timestamps. Missing sequence numbers still affect this observed rate;
  this is not a guaranteed measurement of the sensor's internal frame rate.
- `max_frame_gap_ms`, `sequence_gaps`, `invalid_frames`, `select_timeouts`:
  diagnostics to distinguish reduced light from capture/processing trouble.
  `invalid_frames` counts both invalid buffers and attempted JPEG decode failures.
- `decoded_frames`, `detection_count`, `found_count`: per-window processing
  counts. Divide found_count by detection_count for detection success; a zero
  denominator is unavailable, not a failed detection. `confidence` is the latest
  observation (zero if detection is disabled).
- `capture_fps_ema60`, `luma_ema60`: exponential averages with a 60-second time
  constant. Startup needs a settling period. Missing images do not update luma.
- `exposure_raw_100us`, `gain_raw`, `auto_exposure`, `dynamic_framerate`: values
  returned by V4L2 G_CTRL, null if unavailable. The corresponding `_flags`
  preserve QUERYCTRL status, including INACTIVE (0x10) and VOLATILE (0x80).
  **An inactive/manual exposure or gain value is not verified automatic telemetry.**
  Exposure raw units are 100 microseconds; gain units are camera-specific, not
  calibrated linear amplification. Do not infer lux from these values.
- `image_valid`: statistics came from a successfully decoded original image.
  Image statistics are absent on image-invalid records, rather than a stale
  brightness being mistaken for a fresh measurement.
- `luma_mean`, `p10`, `p50`, `p90`: sampled JPEG luma, 0–255, computed before
  overlays. This is gamma-processed image brightness, not linear sensor light.
- `dark_fraction` (<24), `bright_fraction` (>239): proportions from 0 to 1.
- `regions_2x3`: six mean luma values, top row left-to-right then bottom row.
  The regions are not automatically masked for sky or reflectors.
- `queue_dropped`: cumulative records dropped by a full/busy queue in this
  session. Disk-space/write failures are reported separately in the journal.
- `reference_jpeg`: filename only if the reference image write succeeded.

The logger does not change exposure, gain, camera frame rate or navigation.
Use the trends together with image content and camera settings; shade and
station walls cannot be distinguished from twilight by brightness alone.

Linux verification: `sh linux/tests/run-camera-light-logger-tests.sh`.

## Live `TOO DARK` warning

The warning stays active independently of file logging. Once per second it
measures the median brightness of the three lower image regions. A five-second
exponential filter suppresses frame noise. Below 85/255 for 20 seconds enables
the warning; above 115/255 for 45 seconds clears it. Gaps longer than three
seconds reset the persistence timers. This is a conservative scene-brightness
warning, not an astronomical dusk detector and not a motion interlock.

Replay of the 2026-10-17 recording: first warning at 18:10:09 CEST, subsequent
warnings at 18:19:34 and 18:24:12, with recovery on brighter views; continuously
active from 18:30:49 until recording ends. Thresholds are calibrated to this
camera/recording, not universal lux or marker-recognition limits.

Hardware-free classifier test:

```sh
c++ -std=c++11 -Isunray linux/tests/image-darkness-test.cpp -o /tmp/image-darkness-test
/tmp/image-darkness-test
```
