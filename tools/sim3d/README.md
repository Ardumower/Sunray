# Sunray cloud 3D simulator

`linux/config_sim_ws.h` selects simulated hardware and the mscmower navigation and
camera settings from `config_xlmower.h`. `config_sim.h` retains the legacy simulator.
Godot owns physical position, collision, encoders, and camera rendering. Sunray
runs its normal estimator, path planner, reflector detector, and Cloud interface.
The initial known spawn pose seeds the estimator once; rendering never uses the
estimated pose. No Cloud connection is required for local simulation to continue.

## Twenty installation

The deployed source/build is `/home/orangepi/Sunray-sim3d`, separate from the
original `/home/orangepi/Sunray` checkout. Build on Linux with the normal Sunray
dependencies:

```sh
sh tools/sim3d/build.sh
```

`systemd/` contains the three units deployed on Twenty. They use the existing
isolated GPU prerequisites in `/home/orangepi/godot-camera-test`:

- Official Godot 4.7.2 Linux ARM64 executable.
- Extracted Rockchip Xorg 1.20.11 (`xserver-xorg-core_1.20.11-1_arm64.deb`,
  Radxa rockchip-xserver release `rk3528_linux_release-linux-5.10-gen-rkr6`).
  SHA256: `a13593a292a63b6e976ab3b22d60804309acadeeb41a4909b59ef9f773f9dcff`.
- Modesetting/glamor Xorg config, 1280x720 screen, DRI2, private Xauthority file.
- Installed Mali userspace driver and connected HDMI dummy.

These are machine-specific service templates, not a generic GPU installer.
They do not replace the system Xorg packages. Display :88 disables TCP listening.
The world renders with OpenGL ES on Mali-G610, not Godot's headless dummy renderer.

```sh
sudo cp tools/sim3d/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now sunray-sim3d-display sunray-sim3d-world sunray-sim3d
journalctl -u sunray-sim3d -f
cat /run/sunray-sim3d/status.json
```

Stop Sunray before running the independent world integration test, which moves
only the virtual robot and returns it to its starting pose:

```sh
sudo systemctl stop sunray-sim3d
python3 tools/sim3d/test_world.py
sudo systemctl start sunray-sim3d
```

Artificial connection loss is disabled by default. `--test-disconnect` explicitly
opts into that fault-injection test; do not use it during normal operation.
No background task deliberately interrupts Cloud or local simulator connections.
Actual loss of local control commands stops simulated motors after 500 ms; stale
world feedback/camera frames are rejected. Cloud reconnects run independently.

## Scene and camera

`map.json` is the selected simulator Cloud map `msc_flugfeld_komplett` (map 5).
Cloud uploads update Sunray's navigation map; they do not teleport the robot or
replace the scene geometry. After changing the ground plan, export the matching
map JSON here and restart both world and Sunray while idle.

`scene.json` contains editable dimensions: robot 0.8 x 1.0 m, camera position,
horizontal FOV, dock table, marker strips, and spawn distance. Dock table sides
are closed and its entrance is open. Ground is flat; the runway exclusion is a flat surface, not a collision wall.
These are provisional dimensions, not a surveyed reconstruction. FOV is currently
75 degrees, explicitly uncalibrated; close-up photographs alone do not uniquely
determine intrinsics. Camera image is 640x360 at approximately 10 fps. Sunray's
existing output scaling, timestamp overlays, and marker channels remain in use.
The build helper requires RKNN headers/runtime and copies the included grass model.

The final dock waypoint is the robot-center charging pose. The panel is ahead by
`camera.forward_m + dock.camera_gap_m`; currently the camera remains 0.5 m from it
in charging position. Contact requires distance below 0.12 m and heading error
below 0.25 rad. Charging is a simple voltage/contact model, not a battery model.

The local UDP protocol listens only on 127.0.0.1:19091. Atomic camera files contain
`SRF1`, little-endian sequence and Unix milliseconds (two uint64 values), then JPEG.
Do not run two Sunray instances or the integration test against one world at once.

## Safety nets

`scene.json` contains two open polylines along the field-facing perimeter of
`msc_vorbereitungsfeld` (Cloud map 35), with a gap on the dock approach. The three
single-point exclusions in that map all duplicate (44.61, 34.53); only this one
distinct post location is evidenced. Remaining post spacing (20 m), height (3 m),
and centerline between samples are provisional. Nets are currently rolled up at the top: only posts and narrow top rolls have
collision. Setting `rolled_up` false adds transparent cutout meshes and continuous
collision panels. They are independent of
the navigation map, so adding scenery does not alter mowing routes.

## Surroundings

`site_environment.gd` builds the hut, ground patches, and deterministic low-poly
tree groups from `scene.json`. Site placement uses the satellite origin stored
for Cloud simulator 1 (52.3768191763, 8.7041398510), verified against the visible
airfield. The real robot 14 currently stores latitude 52.3486, which displays a
different place; neither Cloud origin was modified. Satellite images were used
as reference only and are not distributed as textures. Trees, building heights,
additional table spacing and field boundaries are approximations, not surveyed
geometry. The long runway exclusion remains excluded in navigation but has no
artificial perimeter walls. Rolled nets permit passage beneath them; their posts
remain solid obstacles.

## Molehills and grass recognition

Twelve deterministic low earth mounds are configured in `scene.json`, inside the
mowing perimeter and outside the runway exclusion. Heights are 14–23 cm, radii
28–43 cm. Each has irregular brown geometry and a convex collision shape. They
are not added to the navigation map: perception-based avoidance remains a future
step. Collision reports through the existing simulated bumper interface.

The one-metre grass albedo tile is generated with the built-in image_gen tool;
the complete prompt is in `assets/grass_albedo.prompt.txt`. `grass.gdshader` maps
it in world coordinates with mipmaps/anisotropic sampling. No simulator labels
are passed to the detector; `GrassOverlay` performs ordinary RKNN inference on
the rendered RGB images, exactly as with the real camera.

Twenty initially lacked the RKNN include path and its service lacked the render
group. `build.sh` selects the existing rknpu2 headers (or `RKNN_INCLUDE_DIR`) and
requires RKNN support; the service has `SupplementaryGroups=render`. The installed
runtime is `/usr/lib/librknnrt.so` (2.3.2). First live inference measured 28.3 ms.
Cloud camera index 100 was verified to deliver classified images.

Controlled comparison, camera (east=28,height=0.4,north=0) aimed at
(30,0.1,0), 640x360, identical geometry and lighting, 2026-10-28:
- Lower image third, 1200 segmentation cells containing unobstructed lawn:
  plain surface 84/1200 grass (7%); textured surface 1200/1200 grass (100%).
- Mound bounding ROI (includes some grass): 34/60 cells classified as obstacle
  after texturing; the plain scene incorrectly classified the surrounding lawn
  as obstacle as well.
These are measurements for one view, not a general detection-accuracy guarantee.
