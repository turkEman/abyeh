#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════
//   wifi_bridge.h  —  Public API
//
//   All functions are safe to call from your FC loop on Core 0.
//   WiFi, TCP, and JSON parsing live entirely on Core 1.
//   The FC loop NEVER blocks waiting on network state.
//
//   Setup pattern:
//       wifiBridge_addTelemetry(...)   // register your float vars
//       wifiBridge_addControl(...)
//       wifiBridge_init()              // call LAST in setup()
//
//   Loop pattern (call based on micros() timers):
//       wifiBridge_sendTelemetry()     // at 100–200 Hz
//       wifiBridge_receiveControl()    // as often as you like, ~0 cost
// ═══════════════════════════════════════════════════════════════════════

/**
 * Start the WiFi bridge task on Core 1.
 * Call LAST in setup(), after all addTelemetry / addControl calls.
 */
void wifiBridge_init();

/**
 * Register a float variable for the telemetry stream (ESP32 → Laptop).
 *
 * @param name  Label that appears as the JSON key on the Python/browser side.
 * @param ptr   Pointer to your float. Bridge reads it; never writes it.
 *
 * Call in setup() BEFORE wifiBridge_init().
 * The ORDER of calls here must match TELEM_NAMES in bridge.py.
 */
void wifiBridge_addTelemetry(const char* name, float* ptr);

/**
 * Register a float variable to receive from the laptop (Laptop → ESP32).
 *
 * @param name  Must match the JSON key the laptop sends, e.g. "kp_roll".
 * @param ptr   Pointer to your float. Bridge writes it when a new value arrives.
 *              Initialised from the variable's current value at registration time,
 *              so partial JSON updates (only some keys) leave others unchanged.
 *
 * Call in setup() BEFORE wifiBridge_init().
 */
void wifiBridge_addControl(const char* name, float* ptr);

/**
 * Snapshot all registered telemetry variables and wake the WiFi task to send.
 *
 * Uses a seqlock — Core 0 never waits, never touches network state.
 * Call from loop() at your target telemetry rate (e.g. every 10 000 µs = 100 Hz).
 */
void wifiBridge_sendTelemetry();

/**
 * Apply any pending control values received from the laptop.
 *
 * Non-blocking. Returns true if at least one variable was updated this call.
 * Typically costs < 1 µs when there is nothing new.
 * Call from loop() at whatever rate suits you; checking every FC cycle is fine.
 */
bool wifiBridge_receiveControl();

/** True when the TCP connection to the laptop bridge is active. */
bool wifiBridge_isConnected();
