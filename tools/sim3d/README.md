# Sunray cloud 3D simulator

`linux/config_sim_ws.h` selects simulated hardware and the mscmower navigation and
camera settings from `config_xlmower.h`. `config_sim.h` retains the legacy simulator.
Godot owns physical position, collision, encoders, and camera rendering. Sunray
runs its normal estimator, path planner, reflector detector, and Cloud interface.
The initial known spawn pose seeds the estimator once; rendering never uses the
estimated pose. No Cloud connection is required for local simulation to continue.

## Installation and updates through service.sh

On the Orange Pi, run as the normal user (not with `sudo`):

```sh
cd /home/orangepi/Sunray-sim3d
bash linux/service.sh
```

Choose **3D simulator menu (Orange Pi)**, then **Set up / update simulator**.
There are only four actions: setup/update, start, stop, and status/diagnostics.
The noninteractive equivalent is:

```sh
bash linux/service.sh simulator setup
```

For a fresh installation, first install the board's **Ubuntu 22.04 ARM64 image
with vendor kernel 6.1 for Orange Pi RK3588/RK3588S**, create a normal sudo-capable
user, connect an HDMI dummy or monitor, and clone this repository. For example:

```sh
sudo apt-get update
sudo apt-get install -y git
git clone https://github.com/Ardumower/Sunray.git ~/Sunray-sim3d
cd ~/Sunray-sim3d
bash linux/service.sh
```

Setup checks the architecture, OS, SoC, `/dev/mali0`, and HDMI connection before
changing the installation. Mainline/Panfrost images and other Orange Pi families
are not supported by this vendor driver bundle. The base image must provide
working Mali and RKNPU kernel drivers; setup does not replace the kernel or device
tree. If missing GPU firmware is installed, a reboot and another setup run are
required. Internet access is needed to install packages/download components.

Setup automatically:

1. Stops the simulator and installs build/display dependencies (no full OS upgrade).
2. Downloads pinned Godot 4.7.2 ARM64, Rockchip Xorg 1.20.11, Mali G610 for Linux
   6.1, RGA, and RKNN 2.3.2 headers/runtime, verifying SHA-256 checksums.
3. Extracts vendor libraries into `~/godot-camera-test`, separate from the system
   Xorg/Mesa packages. Configures modesetting/glamor, DRI2, display `:88`, a private
   Xauthority cookie, and GPU/NPU device permissions. Xorg TCP listening is disabled.
4. Builds natively with `linux/config_sim_ws.h` and mandatory RKNN grass inference,
   then imports the Godot project and installs all three systemd services with
   paths and user adjusted to the current checkout/account.
5. Starts the world and verifies a fresh JPEG and a Mali-G610 renderer. Runs a
   grass-model inference on the NPU under the service user and device groups.
6. Starts Sunray and enables all three services at boot only after successful checks.

Setup/update resets the world to its spawn pose. It rebuilds the **current local
source**; it does not pull Git changes. Use a separate simulator checkout because
CMake writes the selected configuration to `sunray/config.h`. Twenty uses
`/home/orangepi/Sunray-sim3d`; its original `/home/orangepi/Sunray` is separate.
A different board also needs its own allowed Cloud robot identity/MAC; successful
local setup does not automatically register a new device in the Cloud.

Downloads are cached and checksum-verified when setup is repeated. Errors stop
setup instead of reporting success with software rendering or missing NPU support.
An interrupted/failed setup can be rerun. Consult **Status / diagnostics** for
service state and recent logs. The HDMI dummy must remain connected after setup.

Vendor sources (versions and full hashes are pinned in `linux/service.sh`):

- [Godot official builds](https://github.com/godotengine/godot-builds/releases/tag/4.7.2-stable)
- [Radxa Rockchip Xorg](https://github.com/radxa-pkg/rockchip-xserver/releases/tag/rk3528_linux_release-linux-5.10-gen-rkr6)
- [Orange Pi Mali/RGA packages](https://github.com/orangepi-xunlong/rk-rootfs-build/tree/adaf2dc40407566abee889deec7b6a8b5aaab133/common)
- [Rockchip RKNN runtime](https://github.com/airockchip/rknn-toolkit2/tree/42aa1d426c0a9e0869b6374edba009f7208a1926/rknpu2/runtime/Linux/librknn_api)

The automated setup was tested on Twenty (Orange Pi 5 Pro, Ubuntu 22.04.5,
vendor kernel 6.1.43) on 2026-10-28: native build, downloaded/extracted GPU stack,
Mali-G610 rendering, fresh camera frames, NPU inference, and service autostart all
passed, including a second setup run using the verified download cache. This was
an existing OS installation, not a freshly flashed SD card.
Command-routing, build failure, and download-checksum tests pass on Mac and Twenty:

```sh
python3 linux/tests/test_simulator_service.py
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

## Direct camera and joystick connection

Simulator setup also installs the optional Linux WebRTC helper. Local browsers
can receive camera images and send joystick input directly while external clients
continue through Cloud. See [direct-link setup and behavior](../direct_link/README.md).

## Everyday operation on Twenty

All simulator operations are available in **`linux/service.sh` → “3D simulator
menu (Orange Pi)”**. Launch it on Twenty as the normal `orangepi` user:

```sh
cd /home/orangepi/Sunray-sim3d
bash linux/service.sh
```

The menu's setup action handles the entire build/install/start sequence. For
advanced scripting, `bash linux/service.sh simulator ACTION` also accepts `build`,
`start`, `stop`, `restart`, `status`, and `log`. The direct `build` action requires
Sunray to be stopped (`sudo systemctl stop sunray-sim3d`); the world can remain
running to preserve its pose. It never restarts services itself.
`RKNN_INCLUDE_DIR`, `RKNN_LIBRARY`, and `BUILD_JOBS` override build defaults.
`start`/`stop` leave the boot setting unchanged; the advanced `enable`/`disable`
actions change boot autostart without starting/stopping services. The entry point
also works from other working directories when invoked with an absolute path.

After the installation above, start the simulator from a Mac terminal with:

```sh
ssh twenty.local 'bash /home/orangepi/Sunray-sim3d/linux/service.sh simulator start'
```

Starting Sunray also starts its world and display dependencies if they are stopped.
If already logged into Twenty, use `sudo systemctl start sunray-sim3d` directly.
The enabled services start automatically when the Orange Pi boots; the HDMI
dummy must remain connected for this GPU setup.

Check all three services and follow the Sunray log:

```sh
ssh twenty.local 'bash /home/orangepi/Sunray-sim3d/linux/service.sh simulator status'
ssh twenty.local 'bash /home/orangepi/Sunray-sim3d/linux/service.sh simulator log'
```

Restart only Sunray (leave the virtual world running):

```sh
ssh twenty.local 'bash /home/orangepi/Sunray-sim3d/linux/service.sh simulator restart'
```

Stop the entire simulator, including camera rendering and its dedicated display:

```sh
ssh twenty.local 'bash /home/orangepi/Sunray-sim3d/linux/service.sh simulator stop'
```

Stopping only `sunray-sim3d` leaves the world and display running. The world's
command watchdog stops the simulated motors after 500 ms. An explicit stop does
not disable autostart at the next boot.

### Position and restart behavior

- Restarting only Sunray preserves the physical virtual pose while the world
  remains running. Whether navigation resumes depends on Sunray's saved operation
  and navigation state; a service start is not itself a mowing command.
- Restarting `sunray-sim3d-world`, or rebooting the Orange Pi, resets the physical
  pose. It is not automatically saved and restored across world restarts.
- The current spawn is **2.5 m before the dock, facing the marker**, not inside
  the dock. Change `spawn.distance_before_dock_m` in `scene.json` to adjust it.
- Cloud unavailability does not stop the local simulation. Loss of the local
  Sunray/world connection is handled separately by the watchdog described above.

## Scene and camera

`map.json` is the selected simulator Cloud map `msc_flugfeld_komplett` (map 5).
Cloud uploads update Sunray's navigation map; they do not teleport the robot or
replace the scene geometry. After changing the ground plan, export the matching
map JSON here and restart both world and Sunray while idle.

`scene.json` contains editable dimensions: robot 0.8 x 1.0 m, camera position,
horizontal FOV, dock table, marker strips, and spawn distance. Dock table sides
are closed and its entrance is open. Ground is flat; the runway exclusion is a flat surface, not a collision wall.
The runway uses matte grey asphalt (`asphalt.gdshader`) with procedural centimetre-scale
aggregate and subtle broad weathering. Fine grain fades below pixel size to avoid
shimmering in the moving camera. It does not use the grass texture.
These are provisional dimensions, not a surveyed reconstruction. FOV is currently
75 degrees, explicitly uncalibrated; close-up photographs alone do not uniquely
determine intrinsics. Camera image is 640x360 at approximately 10 fps. Sunray's
existing output scaling, timestamp overlays, and marker channels remain in use.
The simulator build action requires RKNN headers/runtime and copies the included grass model.

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
group. `service.sh simulator setup` now installs its own pinned RKNN headers/runtime
in `~/godot-camera-test/rknn` and assigns `SupplementaryGroups=render video`.
The separate `build` action also accepts `RKNN_INCLUDE_DIR`/`RKNN_LIBRARY`.
The previous system runtime `/usr/lib/librknnrt.so` remains untouched.
First live inference in the original setup measured 28.3 ms.
Cloud camera index 100 was verified to deliver classified images.

Controlled comparison, camera (east=28,height=0.4,north=0) aimed at
(30,0.1,0), 640x360, identical geometry and lighting, 2026-10-28:
- Lower image third, 1200 segmentation cells containing unobstructed lawn:
  plain surface 84/1200 grass (7%); textured surface 1200/1200 grass (100%).
- Mound bounding ROI (includes some grass): 34/60 cells classified as obstacle
  after texturing; the plain scene incorrectly classified the surrounding lawn
  as obstacle as well.
These are measurements for one view, not a general detection-accuracy guarantee.

## Distant woods

Additional forest outlines were approximated from the wider satellite view
(zoom 15, same reference origin and center). Six groups add 6,896 distant trees
to the west/southwest, north/northwest, south, southeast, northeast and far north.
The camera far plane is configurable as `camera.far_m` (1800 m); the background
ground extends 4 km across. Open farmland seen in the aerial reference remains
open, including gaps toward the east.

Distant woods use two MultiMesh draws per group (crowns and trunks), with no
individual scene nodes, dynamic shadows or collision bodies. Nearby trees retain
their existing collision geometry. Four cardinal-direction camera previews on
Twenty each maintained the configured 10 fps preview limit. These are approximate
background silhouettes; terrain elevations and individual trees are not surveyed.
