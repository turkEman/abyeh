// Runs on Core 1.  The DRDY ISR latches esp_timer_get_time() at the rising
// edge; the task wakes every 2 ms, checks for a pending timestamp, then bursts
// 6 I2C bytes and publishes one MagPacket to the depth-1 overwrite queue.
// Core 0 consumes the queue at 100 Hz in fc_update_attitude_mag().

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include "QMC5338.h"

#ifndef PIN_QMC_DRDY
#define PIN_QMC_DRDY 36    // input-only on ESP32; no internal pull-up in GPIO matrix
#endif

// ── ISR ↔ task handshake ──────────────────────────────────────────────────
// Protected by a spinlock rather than a mutex (ISR context).
// Value 0 = no unread event.

static portMUX_TYPE  s_drdy_mux     = portMUX_INITIALIZER_UNLOCKED;
static int64_t       s_drdy_time_us = 0;

// ── Exported ──────────────────────────────────────────────────────────────
QueueHandle_t magDataQueue = nullptr;

// ── Private ───────────────────────────────────────────────────────────────
static SemaphoreHandle_t s_i2cMutex = nullptr;

static constexpr uint32_t    MAG_INTERVAL_MS    = 10;   // 100 Hz hard ceiling
static constexpr uint32_t    MAG_HEARTBEAT_MS   = 2;    // poll period
static constexpr uint32_t    MAG_STACK_BYTES    = 3072;
static constexpr UBaseType_t MAG_PRIORITY       = 5;    // below IMU task (≥10)
static constexpr BaseType_t  MAG_CORE           = 1;
static constexpr uint32_t    MAG_MUTEX_TIMEOUT_MS = 15;

// ── DRDY ISR ──────────────────────────────────────────────────────────────
// IRAM_ATTR is mandatory on ESP32: ISRs must live in IRAM to avoid
// cache-miss stalls during flash reads that would corrupt the timestamp.

static void IRAM_ATTR drdyISR(void)
{
    portENTER_CRITICAL_ISR(&s_drdy_mux);
    s_drdy_time_us = esp_timer_get_time();
    portEXIT_CRITICAL_ISR(&s_drdy_mux);
}

// ── Task ──────────────────────────────────────────────────────────────────

static void magTaskFn(void *)
{
    uint32_t lastReadMs = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(MAG_HEARTBEAT_MS));   // yield first

        const uint32_t now = (uint32_t)millis();
        if ((now - lastReadMs) < MAG_INTERVAL_MS)
            continue;   // enforce 100 Hz ceiling even if DRDY fires faster

        // ── Atomically consume the DRDY timestamp ─────────────────────────
        portENTER_CRITICAL(&s_drdy_mux);
        int64_t drdy_time = s_drdy_time_us;
        if (drdy_time > 0)
            s_drdy_time_us = 0;         // consume before releasing lock
        portEXIT_CRITICAL(&s_drdy_mux);

        if (drdy_time == 0)
            continue;   // mid-conversion; retry on next heartbeat (well within 10 ms)

        // ── I2C burst-read ─────────────────────────────────────────────────
        if (!xSemaphoreTake(s_i2cMutex, pdMS_TO_TICKS(MAG_MUTEX_TIMEOUT_MS)))
        {
            // Bus contention: re-arm the timestamp so the sample isn't lost.
            portENTER_CRITICAL(&s_drdy_mux);
            if (s_drdy_time_us == 0)    // don't overwrite a newer ISR event
                s_drdy_time_us = drdy_time;
            portEXIT_CRITICAL(&s_drdy_mux);
            continue;
        }

        Vector3f mag;
        const bool ok = qmc_read_data(&mag);
        xSemaphoreGive(s_i2cMutex);

        if (ok)
        {
            MagPacket pkt;
            pkt.mag          = mag;
            pkt.timestamp_us = drdy_time;   // ISR edge — not post-I2C
            xQueueOverwrite(magDataQueue, &pkt);
            lastReadMs = now;
        }
        // On read failure lastReadMs is NOT updated — retry in 2 ms.
        // The old queue value remains valid (last good reading).
    }
}

// ── Public entry point ────────────────────────────────────────────────────
void magTask_start(SemaphoreHandle_t i2cMutex)
{
    s_i2cMutex   = i2cMutex;
    magDataQueue = xQueueCreate(1, sizeof(MagPacket));
    configASSERT(magDataQueue);

    // QMC CTRL2.INT_ENB must be 1 (set in qmc_init) for this to fire.
    // Using a named function instead of a lambda gives it storage class for
    // IRAM_ATTR to take effect.
    attachInterrupt(digitalPinToInterrupt(PIN_QMC_DRDY), drdyISR, RISING);

    xTaskCreatePinnedToCore(
        magTaskFn, "mag",
        MAG_STACK_BYTES, nullptr,
        MAG_PRIORITY, nullptr,
        MAG_CORE);
}