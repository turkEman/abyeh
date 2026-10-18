#pragma once

// ═══════════════════════════════════════════════════════════════════════
//   wifi_bridge_config.h  —  EDIT THIS FILE
//
//   Only file you need to touch for network configuration.
// ═══════════════════════════════════════════════════════════════════════

// ── Network ─────────────────────────────────────────────────────────────
// Your laptop's hotspot SSID and password
#define WB_WIFI_SSID        "turk"
#define WB_WIFI_PASS        "1234567800"

// Laptop IP — when using Windows laptop hotspot this is almost always
// 192.168.137.1 (the hotspot gateway). Verify in: Settings → Mobile Hotspot
// → "Properties" or run `ipconfig` and look at the "Local Area Connection*"
// adapter with IP 192.168.137.x
#define WB_SERVER_IP        "192.168.137.1"
#define WB_SERVER_PORT      5000

// ── Variable Limits (compile-time) ──────────────────────────────────────
// Maximum number of vars you can register. Only affects RAM.
// Actual count is set by how many wifiBridge_addTelemetry() calls you make.
#define WB_MAX_TELEM_VARS   32
#define WB_MAX_CTRL_VARS    32

// ── FreeRTOS Task ────────────────────────────────────────────────────────
#define WB_TASK_STACK_SIZE  8192  // bytes — do not go below 4096
#define WB_TASK_PRIORITY    1     // keep lower than your FC task
#define WB_TASK_CORE        1     // always Core 1, FC runs on Core 0
