#pragma once
// Cloud-connected Linux 3D simulator with mscmower navigation/camera settings.
// config_sim.h remains the legacy simulator for the old app/Bluetooth.
#include "../linux/config_xlmower.h"
#undef DRV_CAN_ROBOT
#define DRV_SIM_ROBOT 1
#define SIM3D_ENABLE true
#define SIM3D_FRAME_FILE "/run/sunray-sim3d/camera.frame"
#undef WS_ROBOT_CONNECT_KEY
#define WS_ROBOT_CONNECT_KEY "99bb86e57429815531ce96aea6389f99"
#undef BLE_NAME
#define BLE_NAME "Sunray 3D simulator"
#undef WIFI_HEATMAP_ENABLE
#define WIFI_HEATMAP_ENABLE false
#undef IOEXT_LIGHTS_ENABLE
#define IOEXT_LIGHTS_ENABLE false
#undef ENABLE_NTRIP

// SimStopButtonDriver reports logical state, not the active-low hardware pin.
#undef BUTTON_INVERT
#define BUTTON_INVERT false
