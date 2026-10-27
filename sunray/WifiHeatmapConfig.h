#pragma once
// Linux only. Disabled means no worker, no probes and no log file access.
#ifndef WIFI_HEATMAP_ENABLE
#define WIFI_HEATMAP_ENABLE false
#endif
#ifndef WIFI_HEATMAP_FILE
#define WIFI_HEATMAP_FILE "wifi-heatmap.csv"
#endif
#ifndef WIFI_HEATMAP_INTERFACE
#define WIFI_HEATMAP_INTERFACE "wlan0"
#endif
#ifndef WIFI_HEATMAP_MIN_DISTANCE_M
#define WIFI_HEATMAP_MIN_DISTANCE_M 1.0
#endif
#ifndef WIFI_HEATMAP_MAX_AGE_DAYS
#define WIFI_HEATMAP_MAX_AGE_DAYS 30
#endif
