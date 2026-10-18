# Grass segmentation model

`grass.rknn` is included in the project and copied to `build/models/grass.rknn` by CMake when RKNN is available. It requires a Rockchip RK3588-family NPU. No separate model download is needed.

- SHA-256: `23d58d619897ac9e8177084130fa67e0f88b260f75cbb2f1def87aed3e2571eb`
- Size: 12,876,463 bytes.
- Build settings: RKNN Toolkit2 2.3.2, target RK3588, optimization level 3, FP16 without quantization.
- Input: RGB UINT8, NHWC, 640 × 360. Resize the entire camera image; do not swap RGB/BGR or normalize again.
- Compiled normalization: mean `[123.675,116.28,103.53]`, standard deviation `[58.395,57.12,57.375]`.
- Output: five score planes, NCHW `[1,5,45,80]`; choose the largest score at each cell.
- Display colors: 0 obstacle red, 1 grass green, 2 excluded region purple, 3 non-grass orange, 4 restricted region blue. Except grass, names describe inferred semantic roles rather than verified training labels.

Camera index **100** is the grass overlay. Physical camera indices 0–3 retain their existing role. Overlay uses camera 0, or the already active reflector-detection camera when that detector owns capture.

Example existing protocol command (with normal transport authentication/checksum):

    AT+CAM,1,100,320,240,5,60

Select camera index 0 for the ordinary image. Index 100 runs inference only while streaming is requested. The cloud app provides a dedicated Grass overlay option in both camera selectors. No motor/navigation behavior uses these predictions.

Inference runs in a separate thread with one pending frame (overwritten by newer input) and one result. Each displayed overlay uses its matching input image and capture timestamp. The latest result is drawn with 80% opacity (20% camera image). Results older than one second are not displayed; `GRASS WAIT` indicates missing/stale results. The model stays loaded until the capture worker exits. This avoids reinitialization on each frame. Inference uses NPU core 0; capture and reflector detection continue independently.

`SUNRAY_GRASS_MODEL` can override the model path. The default is relative to the running executable, not its working directory. Non-RKNN Linux builds still compile and show GRASS WAIT for this view.

## Hardware verification

2026-10-18, mscmower: RK3588S, runtime 1.5.2, driver 0.9.6. Firmware built successfully. The dedicated overlay test submits 50 frames, verifies eventual delivery of the latest timestamp, and compares every output RGB pixel against an independently calculated NPU result. The overlay worker measured approximately 43 ms including input/output conversion on this test. Existing runtime and robot services are used without replacement of system libraries.

Test (from repository root, requires RKNN headers/library and input/reference fixtures):

    g++ -std=c++11 -O2 -pthread -DSUNRAY_RKNN -I/usr/include/rknn -Isunray/src linux/tests/grass-overlay-test.cpp sunray/src/camera/GrassOverlay.cpp -lrknnrt -o /tmp/grass-overlay-test
    SUNRAY_GRASS_MODEL="$PWD/linux/models/grass.rknn" /tmp/grass-overlay-test input.rgb reference.fp32

The input fixture must contain 640×360 RGB bytes; the reference contains 5×45×80 float32 scores. Test data is not needed for firmware operation.

Deployment configuration on mscmower: CMake uses `linux/config_xlmower.h` explicitly. The undock test mode is disabled (`UNDOCK_TEST_TIMEOUT_MS=0`); RTK docking uses `DOCK_RTK_SPEED=0.5` m/s and camera docking retains its separate low-speed limit.
