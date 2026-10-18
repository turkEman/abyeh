#include "wifi_bridge.h"
#include "wifi_bridge_config.h"

#include <WiFi.h>
#include <ArduinoJson.h>
#include <atomic>
#include <string.h>

// ═══════════════════════════════════════════════════════════════════════
//   Debug logging
//   ─────────────────────────────────────────────────────────────────────
//   Enable by adding a build flag — do NOT hardcode it here so you can
//   flip it per-build without touching source.
//
//   PlatformIO (platformio.ini):
//       build_flags = -DWB_DEBUG                  ← normal debug
//       build_flags = -DWB_DEBUG -DWB_DEBUG_VERBOSE  ← per-packet noise too
//
//   Arduino IDE: add the following line BEFORE your #include "wifi_bridge.h"
//       #define WB_DEBUG
//       #define WB_DEBUG_VERBOSE   // optional
//
//   WB_LOG   → always printed when WB_DEBUG is defined
//              (WiFi events, TCP events, errors, first-time state changes)
//   WB_LOGV  → only when WB_DEBUG_VERBOSE is also defined
//              (per-packet send, per-key ctrl update, seqlock read details)
// ═══════════════════════════════════════════════════════════════════════
#ifdef WB_DEBUG
#define WB_LOG(fmt, ...) \
    Serial.printf("[WB %8lums] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
#ifdef WB_DEBUG_VERBOSE
#define WB_LOGV(fmt, ...) \
    Serial.printf("[WB:V %8lums] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
#else
#define WB_LOGV(fmt, ...) \
    do                    \
    {                     \
    } while (0)
#endif
#else
#define WB_LOG(fmt, ...) \
    do                   \
    {                    \
    } while (0)
#define WB_LOGV(fmt, ...) \
    do                    \
    {                     \
    } while (0)
#endif

// ── Wire protocol constants ──────────────────────────────────────────────
static constexpr uint8_t WB_MAGIC_0 = 0xFC;
static constexpr uint8_t WB_MAGIC_1 = 0x1A;
static constexpr uint8_t WB_VERSION = 0x01;
static constexpr uint8_t WB_TYPE_TELEM = 0x01;
static constexpr uint16_t WB_HDR_SIZE = 16; // bytes before float payload
// Max packet: 16-byte header + 32 floats × 4 bytes = 144 bytes
static constexpr uint16_t WB_MAX_PKT = WB_HDR_SIZE + WB_MAX_TELEM_VARS * sizeof(float);
static constexpr uint8_t WB_TYPE_CTRL_RESP = 0x02;

// ── Registered variable tables ───────────────────────────────────────────
static const char *telem_names[WB_MAX_TELEM_VARS];
static float *telem_ptrs[WB_MAX_TELEM_VARS];
static uint16_t telem_count = 0;

static const char *ctrl_names[WB_MAX_CTRL_VARS];
static float *ctrl_ptrs[WB_MAX_CTRL_VARS];
static uint16_t ctrl_count = 0;

// ── Telemetry seqlock  (Core 0 writes, Core 1 reads) ────────────────────
struct TelemetrySnap
{
    float values[WB_MAX_TELEM_VARS];
    uint16_t count;
    uint32_t timestamp_us;
};
static std::atomic<uint32_t> telem_seq{0};
static TelemetrySnap telem_snap; // guarded by telem_seq

// ── Control seqlock  (Core 1 writes, Core 0 reads) ──────────────────────
struct ControlSnap
{
    float values[WB_MAX_CTRL_VARS];
    uint16_t count;
};
static std::atomic<uint32_t> ctrl_seq{0};
static ControlSnap ctrl_snap;      // guarded by ctrl_seq
static uint32_t last_ctrl_seq = 0; // private to Core 0

// ── WiFi task handles ────────────────────────────────────────────────────
static TaskHandle_t wb_task_handle = nullptr;
static WiFiClient wb_client;
static std::atomic<bool> wb_connected{false};
static uint32_t wb_pkt_seq = 0; // monotonic packet counter

// ── Receive buffer for newline-delimited JSON ────────────────────────────
static char rx_buf[512];
static int rx_len = 0;

// ── Packet buffer (static → lives in BSS, not on the task stack) ─────────
static uint8_t pkt_buf[WB_MAX_PKT];

// Response packet buffer — sized for JSON payload of all control vars.
// 32 vars × ~40 chars/var ≈ 1300 bytes; 2 KB is comfortable.
static uint8_t resp_buf[2048];

static void send_ctrl_response();

// ═══════════════════════════════════════════════════════════════════════
//   Seqlock helpers
//   seq_cst fences are the most conservative option and unambiguous on
//   any architecture including Xtensa LX6 (in-order, weakly cached).
// ═══════════════════════════════════════════════════════════════════════

static inline void seqlock_write_begin(std::atomic<uint32_t> &seq)
{
    seq.fetch_add(1, std::memory_order_relaxed); // seq → odd (write open)
    std::atomic_thread_fence(std::memory_order_seq_cst);
}

static inline void seqlock_write_end(std::atomic<uint32_t> &seq)
{
    std::atomic_thread_fence(std::memory_order_seq_cst);
    seq.fetch_add(1, std::memory_order_relaxed); // seq → even (write closed)
}

// Returns true if a clean snapshot was obtained; false if retries exhausted.
// snap_dst: destination buffer, src: protected source, sz: bytes to copy.
static bool seqlock_read(std::atomic<uint32_t> &seq,
                         void *snap_dst, const void *src, size_t sz,
                         uint32_t &seq_out)
{
    uint32_t s1, s2;
    for (uint8_t attempt = 0; attempt < 8; attempt++)
    {
        s1 = seq.load(std::memory_order_relaxed);
        if (s1 & 1)
        {
            // Writer is mid-write — yield and retry
            WB_LOGV("seqlock_read: seq odd (0x%08X) on attempt %u — yielding", s1, attempt);
            taskYIELD();
            continue;
        }
        std::atomic_thread_fence(std::memory_order_seq_cst);
        memcpy(snap_dst, src, sz);
        std::atomic_thread_fence(std::memory_order_seq_cst);
        s2 = seq.load(std::memory_order_relaxed);

        if (s1 == s2)
        {
            seq_out = s1;
            return true; // clean read
        }
        // seq changed while we were copying — writer snuck in, retry
        WB_LOGV("seqlock_read: torn read (s1=0x%08X s2=0x%08X) on attempt %u — retrying",
                s1, s2, attempt);
    }
    // All 8 attempts failed — this is very unusual
    WB_LOG("seqlock_read: EXHAUSTED 8 retries (last seq=0x%08X) — snapshot skipped",
           seq.load(std::memory_order_relaxed));
    return false;
}

// ═══════════════════════════════════════════════════════════════════════
//   Packet packing
//   Explicit byte-by-byte layout — no struct padding, no endian surprises.
//
//   Wire format:
//     [0] 0xFC  magic byte 0
//     [1] 0x1A  magic byte 1
//     [2] version  (uint8)
//     [3] type     (uint8, 0x01 = telemetry)
//     [4..7]  seq         (uint32 LE)
//     [8..11] timestamp   (uint32 LE, microseconds since boot)
//     [12..13] var_count  (uint16 LE)
//     [14..15] payload_len (uint16 LE, = var_count × 4)
//     [16..]  floats      (IEEE 754 LE, var_count × 4 bytes)
// ═══════════════════════════════════════════════════════════════════════

static uint16_t pack_telem(const TelemetrySnap &snap, uint8_t *buf, uint32_t seq)
{
    const uint16_t payload_len = snap.count * sizeof(float);

    buf[0] = WB_MAGIC_0;
    buf[1] = WB_MAGIC_1;
    buf[2] = WB_VERSION;
    buf[3] = WB_TYPE_TELEM;
    memcpy(buf + 4, &seq, 4);               // uint32 LE
    memcpy(buf + 8, &snap.timestamp_us, 4); // uint32 LE
    memcpy(buf + 12, &snap.count, 2);       // uint16 LE
    memcpy(buf + 14, &payload_len, 2);      // uint16 LE
    memcpy(buf + 16, snap.values, payload_len);

    return WB_HDR_SIZE + payload_len;
}

// ═══════════════════════════════════════════════════════════════════════
//   Incoming control JSON parser  (Core 1 only)
//
//   Called when a complete '\n'-terminated line is in rx_buf.
//   Builds a new ControlSnap starting from the existing values as a
//   baseline, then overlays only the keys present in the JSON.
//   This means a partial update {"kp": 1.2} doesn't zero out ki or kd.
// ═══════════════════════════════════════════════════════════════════════

static void parse_ctrl_line()
{
    if (rx_len == 0)
        return;

    WB_LOGV("parse_ctrl_line: parsing %d bytes: \"%.*s\"", rx_len, rx_len, rx_buf);

    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, rx_buf, rx_len);

    // Special request key — not a control update
    if (doc.containsKey("_req"))
    {
        const char *req = doc["_req"] | "";
        WB_LOG("parse_ctrl_line: request \"%s\"", req);
        if (strcmp(req, "ctrl") == 0)
        {
            send_ctrl_response();
        }
        return;
    }

    if (err)
    {
        WB_LOG("parse_ctrl_line: JSON error [%s] — discarding: \"%.*s\"",
               err.c_str(), rx_len, rx_buf);
        return; // malformed JSON — silently discard
    }

    // Start from current ctrl_snap values so unmentioned keys are kept.
    // Core 1 is the sole writer of ctrl_snap, so reading it here without
    // the seqlock is safe (we're not racing with ourselves).
    ControlSnap nsnap;
    nsnap.count = ctrl_count;
    memcpy(nsnap.values, ctrl_snap.values, ctrl_count * sizeof(float));

    bool any = false;
    for (uint16_t i = 0; i < ctrl_count; i++)
    {
        if (doc.containsKey(ctrl_names[i]))
        {
            nsnap.values[i] = doc[ctrl_names[i]].as<float>();
            WB_LOGV("parse_ctrl_line: \"%s\" → %.6f", ctrl_names[i], nsnap.values[i]);
            any = true;
        }
    }

    if (!any)
    {
        WB_LOG("parse_ctrl_line: valid JSON but no registered ctrl keys matched — ignored");
        return;
    }

    // Seqlock write (Core 1)
    seqlock_write_begin(ctrl_seq);
    memcpy(&ctrl_snap, &nsnap, sizeof(ControlSnap));
    seqlock_write_end(ctrl_seq);

    WB_LOGV("parse_ctrl_line: ctrl_snap written (seq now 0x%08X, %u vars)",
            ctrl_seq.load(std::memory_order_relaxed), nsnap.count);
}

// ═══════════════════════════════════════════════════════════════════════
//   Ctrl-response sender  (Core 1 only)
//
//   Builds a JSON blob of all currently registered control vars with
//   their live values (read directly from user pointers) and sends it
//   as a type-0x02 binary-framed packet on the TCP socket.
//
//   Called from parse_ctrl_line() when a {"_req":"ctrl"} message arrives.
// ═══════════════════════════════════════════════════════════════════════
static void send_ctrl_response()
{
    if (!wb_client.connected())
    {
        WB_LOG("send_ctrl_response: socket not connected — skipping");
        return;
    }

    // Build JSON directly into the payload area of resp_buf
    StaticJsonDocument<1536> doc;
    for (uint16_t i = 0; i < ctrl_count; i++)
    {
        WB_LOG("  [%u] %s: addr=%p value=%f",
               (unsigned)i,
               ctrl_names[i],
               (void *)ctrl_ptrs[i],
               (double)(*ctrl_ptrs[i]));
        doc[ctrl_names[i]] = *ctrl_ptrs[i];
    }

    char *payload_area = (char *)(resp_buf + WB_HDR_SIZE);
    size_t payload_max = sizeof(resp_buf) - WB_HDR_SIZE;
    size_t json_len = serializeJson(doc, payload_area, payload_max);

    if (json_len == 0 || json_len >= payload_max)
    {
        WB_LOG("send_ctrl_response: JSON serialization failed (len=%u)", (unsigned)json_len);
        return;
    }

    uint32_t ts = micros();
    uint32_t seq = wb_pkt_seq++;
    uint16_t var_count = ctrl_count;
    uint16_t payload_len = (uint16_t)json_len;

    resp_buf[0] = WB_MAGIC_0;
    resp_buf[1] = WB_MAGIC_1;
    resp_buf[2] = WB_VERSION;
    resp_buf[3] = WB_TYPE_CTRL_RESP;
    memcpy(resp_buf + 4, &seq, 4);
    memcpy(resp_buf + 8, &ts, 4);
    memcpy(resp_buf + 12, &var_count, 2);
    memcpy(resp_buf + 14, &payload_len, 2);

    size_t total = WB_HDR_SIZE + json_len;
    size_t sent = wb_client.write(resp_buf, total);

    if (sent != total)
    {
        WB_LOG("send_ctrl_response: partial write %u/%u — reconnecting",
               (unsigned)sent, (unsigned)total);
        wb_client.stop();
    }
    else
    {
        WB_LOG("send_ctrl_response: sent %u vars, %u bytes",
               (unsigned)var_count, (unsigned)total);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//   WiFi task  (Core 1)
// ═══════════════════════════════════════════════════════════════════════

static void wb_task(void *)
{
    WB_LOG("wb_task started on core %d", xPortGetCoreID());

    // ── WiFi connect ─────────────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.begin(WB_WIFI_SSID, WB_WIFI_PASS);

    WB_LOG("WiFi: connecting to SSID \"%s\" ...", WB_WIFI_SSID);

    uint32_t wifi_attempt = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        WB_LOG("WiFi: waiting... attempt %lu (status=%d)", ++wifi_attempt, WiFi.status());
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    WB_LOG("WiFi: connected!  IP=%s  RSSI=%d dBm  channel=%d",
           WiFi.localIP().toString().c_str(),
           WiFi.RSSI(),
           WiFi.channel());

    for (;;)
    {
        // ── WiFi watchdog ─────────────────────────────────────────────────
        if (WiFi.status() != WL_CONNECTED)
        {
            WB_LOG("WiFi: connection lost (status=%d) — dropping TCP, reconnecting...",
                   WiFi.status());
            wb_connected.store(false, std::memory_order_relaxed);
            wb_client.stop();
            WiFi.reconnect();
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        // ── TCP reconnect ─────────────────────────────────────────────────
        if (!wb_client.connected())
        {
            wb_connected.store(false, std::memory_order_relaxed);
            wb_client.stop();

            WB_LOG("TCP: connecting to %s:%u ...", WB_SERVER_IP, (unsigned)WB_SERVER_PORT);

            if (!wb_client.connect(WB_SERVER_IP, WB_SERVER_PORT))
            {
                WB_LOG("TCP: connect FAILED — retrying in 2 s");
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            wb_client.setNoDelay(true); // disable Nagle — critical for low lag
            wb_connected.store(true, std::memory_order_relaxed);
            rx_len = 0; // flush stale receive data on new connection

            WB_LOG("TCP: connected to %s:%u  (Nagle disabled, rx_buf flushed)",
                   WB_SERVER_IP, (unsigned)WB_SERVER_PORT);
            WB_LOG("ptr check: gyroNoise addr=%p  val=%.6f", (void *)ctrl_ptrs[0], *ctrl_ptrs[0]);
            send_ctrl_response();
        }

        // ── Wait for telemetry notification from Core 0 ───────────────────
        // Timeout 25 ms doubles as a keep-alive / reconnect poll interval.
        // If no packet arrives in 25 ms we loop and recheck connection health.
        uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25));

        if (notif > 0)
        {
            WB_LOGV("wb_task: notification received (pending count was %lu)", notif);

            // Read telemetry seqlock
            TelemetrySnap snap;
            uint32_t snap_seq;
            if (seqlock_read(telem_seq, &snap, &telem_snap, sizeof(snap), snap_seq))
            {
                uint16_t len = pack_telem(snap, pkt_buf, wb_pkt_seq++);

                if (wb_client.connected())
                {
                    size_t sent = wb_client.write(pkt_buf, len);
                    if (sent != len)
                    {
                        WB_LOG("TCP: write partial/failed — sent %u of %u bytes (pkt #%lu) — reconnecting",
                               (unsigned)sent, (unsigned)len, (unsigned long)(wb_pkt_seq - 1));
                        // Force reconnect on next iteration
                        wb_client.stop();
                    }
                    else
                    {
                        WB_LOGV("TCP: pkt #%lu sent — %u bytes, %u vars, t=%lu µs",
                                (unsigned long)(wb_pkt_seq - 1),
                                (unsigned)len,
                                (unsigned)snap.count,
                                (unsigned long)snap.timestamp_us);
                    }
                }
            }
            // seqlock_read already logged the failure; silently skip this packet.
            // Next notification will bring a fresh snapshot.
        }
        else
        {
            // 25 ms timeout elapsed — no telemetry from Core 0.
            // Only log at verbose level to avoid flooding; this is normal
            // if sendTelemetry() rate is lower than the timeout.
            WB_LOGV("wb_task: 25 ms timeout — no telemetry notification (connected=%d)",
                    (int)wb_connected.load(std::memory_order_relaxed));
        }

        // ── Receive incoming control JSON (newline-delimited) ─────────────
        // Process all available bytes; may span multiple loop iterations
        // if the browser sends a long JSON string.
        while (wb_client.available())
        {
            int c = wb_client.read();
            if (c < 0)
                break;

            if ((char)c == '\n')
            {
                rx_buf[rx_len] = '\0';
                WB_LOGV("TCP rx: complete line (%d bytes) → parse_ctrl_line", rx_len);
                parse_ctrl_line();
                rx_len = 0;
            }
            else
            {
                if (rx_len < (int)(sizeof(rx_buf) - 1))
                {
                    rx_buf[rx_len++] = (char)c;
                }
                else
                {
                    // Buffer overflow — discard this line and start fresh
                    WB_LOG("TCP rx: rx_buf OVERFLOW at %d bytes — line discarded, buffer reset",
                           rx_len);
                    rx_len = 0;
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//   Public API
// ═══════════════════════════════════════════════════════════════════════

void wifiBridge_addTelemetry(const char *name, float *ptr)
{
    if (telem_count >= WB_MAX_TELEM_VARS)
    {
        WB_LOG("addTelemetry: IGNORED \"%s\" — table full (%u/%u)",
               name, (unsigned)telem_count, (unsigned)WB_MAX_TELEM_VARS);
        return;
    }
    telem_names[telem_count] = name;
    telem_ptrs[telem_count] = ptr;
    telem_count++;
    WB_LOG("addTelemetry: [%u] \"%s\" @ %p  (initial=%.4f)",
           (unsigned)(telem_count - 1), name, (void *)ptr, *ptr);
}

void wifiBridge_addControl(const char *name, float *ptr)
{
    if (ctrl_count >= WB_MAX_CTRL_VARS)
    {
        WB_LOG("addControl: IGNORED \"%s\" — table full (%u/%u)",
               name, (unsigned)ctrl_count, (unsigned)WB_MAX_CTRL_VARS);
        return;
    }
    ctrl_names[ctrl_count] = name;
    ctrl_ptrs[ctrl_count] = ptr;
    // Seed ctrl_snap with the variable's current value.
    // Without this, a partial JSON update from the laptop (e.g. only "kp")
    // would leave all other ctrl_snap slots at 0, and the first
    // receiveControl() call would silently zero out ki, kd, etc.
    ctrl_snap.values[ctrl_count] = *ptr;
    ctrl_count++;
    ctrl_snap.count = ctrl_count;
    WB_LOG("addControl:   [%u] \"%s\" @ %p  (seed=%.4f)",
           (unsigned)(ctrl_count - 1), name, (void *)ptr, *ptr);
}

void wifiBridge_init()
{
    WB_LOG("wifiBridge_init: spawning wb_task  core=%d  pri=%d  stack=%u",
           WB_TASK_CORE, WB_TASK_PRIORITY, (unsigned)WB_TASK_STACK_SIZE);
    WB_LOG("wifiBridge_init: %u telem vars, %u ctrl vars registered",
           (unsigned)telem_count, (unsigned)ctrl_count);

    xTaskCreatePinnedToCore(
        wb_task,
        "wb", // task name
        WB_TASK_STACK_SIZE,
        nullptr,
        WB_TASK_PRIORITY,
        &wb_task_handle,
        WB_TASK_CORE // always Core 1
    );

    WB_LOG("wifiBridge_init: task handle = %p", (void *)wb_task_handle);
}

// Called from Core 0 at telemetry rate (e.g. 100 Hz).
// Non-blocking — seqlock write takes < 1 µs at 20 floats.
void wifiBridge_sendTelemetry()
{
    seqlock_write_begin(telem_seq);

    telem_snap.count = telem_count;
    telem_snap.timestamp_us = micros();
    for (uint16_t i = 0; i < telem_count; i++)
    {
        telem_snap.values[i] = *telem_ptrs[i];
    }

    seqlock_write_end(telem_seq);

    WB_LOGV("sendTelemetry: wrote %u vars, t=%lu µs",
            (unsigned)telem_count, (unsigned long)telem_snap.timestamp_us);

    // Wake the WiFi task; it will read the snapshot and send a packet.
    // If the task is busy or not connected the notification accumulates;
    // ulTaskNotifyTake clears it and we always send the latest snapshot.
    if (wb_task_handle)
    {
        xTaskNotifyGive(wb_task_handle);
    }
    else
    {
        WB_LOG("sendTelemetry: WARNING — wb_task_handle is null (init not called yet?)");
    }
}

// Called from Core 0. Near-zero cost when nothing is new.
bool wifiBridge_receiveControl()
{
    ControlSnap snap;
    uint32_t s;

    if (!seqlock_read(ctrl_seq, &snap, &ctrl_snap, sizeof(snap), s))
    {
        WB_LOG("receiveControl: seqlock retries exhausted — skipping");
        return false; // retries exhausted (shouldn't happen — control writes are rare)
    }

    if (s == last_ctrl_seq)
        return false; // no new data since last call

    WB_LOG("receiveControl: new ctrl data (seq 0x%08X → 0x%08X, %u vars)",
           last_ctrl_seq, s, (unsigned)snap.count);

    last_ctrl_seq = s;

    // Write new values to user's registered pointers
    for (uint16_t i = 0; i < snap.count && i < ctrl_count; i++)
    {
        WB_LOGV("receiveControl:   \"%s\" → %.6f", ctrl_names[i], snap.values[i]);
        *ctrl_ptrs[i] = snap.values[i];
    }
    return true;
}

bool wifiBridge_isConnected()
{
    bool c = wb_connected.load(std::memory_order_relaxed);
    WB_LOGV("wifiBridge_isConnected: %s", c ? "YES" : "NO");
    return c;
}