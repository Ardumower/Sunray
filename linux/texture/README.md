# Robot-local training (camera 101)

The cloud Train panel relays authenticated, per-robot requests. It does not
store training images or models. Raw RPCs are handled by the network thread,
not the motion/control loop. They cannot issue motion commands.

The model `../models/texture-features.rknn` comes from
`owl_wido/robot/models/full_image_segformer_l2_student_residual_2048_stretch_norm_e60_fp.rknn`.
The feature/MLP implementation is adapted from that prototype's
`texture-train-profile.py`. The frozen RKNN backbone runs on NPU core 1;
the small classifier is trained locally with NumPy. This does not retrain or
convert the RKNN backbone. No RKNN Python SDK is required: the small C ABI
uses the installed librknnrt and rknn_api.h. Python requires NumPy and OpenCV.

Default ROI: width 60%, start Y 30%, stop Y 65%, patch 32px, red threshold 25%.
All patches refer to the fixed 640x360 analysis image. A patch is red at
classifier probability >= 0.5; the overlay reports obstacle when the red
fraction inside the ROI reaches the configured threshold. Camera 101 colors
patches across the entire image, including partial edge patches. FULL reports
the whole-image red percentage; ROI reports the unchanged decision region.
The feature map is computed once and shared by both evaluations. Untrained, training, unavailable
and stale output are explicitly indicated. Nothing feeds navigation or motors.

Storage: `/home/orangepi/.local/share/sunray/training/` (override
`SUNRAY_TRAINING_DATA`). Images retain capture-time ROI/settings; thumbnails
have red/green ROI borders. Deleting a sample removes its image and thumbnail;
the current classifier stays active until Train replaces it. Maximum 500
images, 12 thumbnails per page; training uses up to 64 evenly sampled patches
per image to bound memory. Training loss is not an independent accuracy test.
Settings and classifier survive restart. Model hash prevents loading a
classifier trained with another feature backbone.

`/dev/shm/sunray-texture/` holds only the latest clean frame and overlay,
atomically replaced in RAM. Firmware publishes at 2fps while video is streamed;
there is no automatic image archive. Both carry monotonic capture timestamps.
Save rejects images older than 2 seconds; stale overlays are never displayed.
Service binds only 127.0.0.1:18741; robot cloud authorization remains required.
The service and firmware must share a user/runtime directory.

Install on MSCMower (paths in service unit are explicit):

```sh
cd /home/orangepi/Sunray/linux/texture
g++ -O2 -shared -fPIC -I/usr/include/rknn rknn_features.cpp -lrknnrt -o librknn_features.so
sudo cp sunray-texture.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now sunray-texture
```

Build/restart the Sunray firmware with the associated camera/network changes.
Test storage/training in temporary directories: `python3 test_service.py`.
