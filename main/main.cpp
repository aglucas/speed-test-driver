#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "driver/dac_oneshot.h"
#include "driver/ledc.h"
#include "driver/i2c_slave.h"
#include "esp_rom_sys.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "speed_test";

// =============================================================================
// CAN bus — ESP32 DOIT DEVKIT V1
//   TJA1050 TXD → GPIO 17 (TX2)
//   TJA1050 RXD → GPIO 16 (RX2)
// =============================================================================
#define CAN_TX_PIN  GPIO_NUM_19
#define CAN_RX_PIN  GPIO_NUM_18

// =============================================================================
// CANopen SDO — mirrors ZombieVerterDisplay/CanSDO.cpp
// =============================================================================
#define ZOMBIE_NODE_ID   3
#define SDO_REQ_ID      (0x600U + ZOMBIE_NODE_ID)   // 0x603 master→slave
#define SDO_REP_ID      (0x580U + ZOMBIE_NODE_ID)   // 0x583 slave→master
#define SDO_INDEX_BASE   0x2100U

#define SDO_READ         0x40
#define SDO_ABORT        0x80

// ZombieVerter parameter IDs (from ZombieVerterDisplay/Globals.h)
#define SOC_VALUE_ID           2015
#define BMS_T_MAX_VALUE_ID     2087
#define MOTOR_RPM_PARAM_ID     2016
#define INVERTER_TEMP_ID       2028
#define MOTOR_TEMP_ID          2029
#define ISA_TEMP_ID            2030
#define UDC_IDX                2006
#define POWER_ID               2011    // DC bus power, kW (VCU already computes udc*idc)
#define OPMODE_ID              2002    // opmode spot value (see OPMODES below)
#define KWH_ID                 2013    // cumulative energy counter, kWh (VALUE_ENTRY(KWh,"kwh",2013))

// ZombieVerter opmode states (param_prj.h "modes" enum / OPMODES string:
// "0=Off, 1=Run, 2=Precharge, 3=PchFail, 4=Charge, 5=Preheat"). The DC bus
// voltage (UDC) is only meaningful while the pack contactors are closed, i.e.
// in Run or Charge; in any other state we report a default nominal voltage.
#define MOD_RUN                1
#define MOD_CHARGE             4
#define DEFAULT_UDC_V          400.0f
#define DEFAULT_SOC_PCT        15.0f

// Cumulative energy comes from the ZombieVerter's KWh spot value, NOT from the
// IVT/Isabellenhütte shunt directly: the shunt's 0x528 broadcast lives on a CAN
// bus we are not wired to, so it can never be snooped here. The VCU *is* on that
// bus — it decodes 0x528 (ISA::handle528, isa_shunt.cpp) and republishes the
// counter as Param::KWh in kWh (utils.cpp: KWh/1000) — so we read the same number
// back out over SDO instead.
//
// Resolution: openinverter SDO values are fixed-point with 5 fractional bits, so
// a kWh-valued param quantises to 1/32 kWh = 31.25 Wh. Fine for the trip/lifetime
// counters and the efficiency average (~1/8 mile of driving per step), but it does
// mean energy_wh advances in steps rather than smoothly.

// =============================================================================
// ESP-NOW — Qualia receiver MAC
// =============================================================================
static uint8_t receiverMAC[] = {0x20, 0x6E, 0xF1, 0xB8, 0xE3, 0x2C};

// I2C telemetry packet — the full dashboard, sent to the waveform display only.
// Two-way contract, change both together:
//   - this struct
//   - telemetry_t in waveform_tr_display/main/i2c_rx.h
// (ESP-NOW no longer uses this struct — see EspNowPowerPacket below.)
typedef struct __attribute__((packed)) {
    float speed_mph;
    float temp_bat_c;
    float temp_mot_c;
    float temp_inv_c;
    float temp_isa_c;
    float soc_pct;
    float udc_v;
    float energy_wh; // cumulative energy counter, Wh (signed; ZombieVerter KWh spot value x1000)
    uint8_t opmode;  // ZombieVerter opmode: 0=Off 1=Run 2=Precharge 3=PchFail 4=Charge 5=Preheat
    uint8_t ready;   // set to 0xA5 when valid; must stay last
} TelemetryPacket;

// ESP-NOW packet — the Qualia is now a power-only gauge, so it gets just power.
// Deliberately separate from TelemetryPacket so the power link and the I2C dash
// evolve independently: changing one never touches the other's wire format.
// Contract: struct.unpack('<fB', msg) in qualia_tr_display/code.py.
typedef struct __attribute__((packed)) {
    float   power_kw;
    uint8_t ready;   // 0xA5 when valid
} EspNowPowerPacket;

static TelemetryPacket     packet;
static esp_now_peer_info_t peerInfo;

// --- ESP-NOW error recovery ---
#define ESP_NOW_ERROR_LIMIT 20
static int   espNowErrorCount  = 0;
static bool  espNowNeedsReinit = false;

// =============================================================================
// DS18B20 battery temperature sensors (OneWire, single GPIO)
// NOTE: read_battery_temps.ino used ONE_WIRE_BUS=4, which is the same pin as
//       CAN_RX (GPIO 4). Move the OneWire bus to DS18B20_PIN (default GPIO 15).
// =============================================================================
#define DS18B20_PIN         GPIO_NUM_15   // ← rewire OneWire bus here
#define DS18B20_COUNT       5

// Cycle *period* (not a trailing sleep): conversion + reads + idle. The pack
// heats fastest while charging, so the ZombieVerter gets fresh temps every 2 s
// then, and a relaxed 5 s otherwise to keep the OneWire bus quiet.
//
// Both periods must stay comfortably under the ZombieVerter's BMS_Timeout
// (custombms.cpp, default 10 s) or CustomBMS::BMSDataValid() goes false between
// our broadcasts and it reports BMS_Tmin/Tmax/Tavg as -100. 5 s leaves 2x margin.
//
// Floor is ~860 ms: DS18B20_CONVERT_MS + ~60 ms to read DS18B20_COUNT sensors.
// A 2000 ms charging period leaves ~1.1 s of slack; if a cycle overruns (bus
// retries) it simply runs late rather than drifting the schedule.
#define DS18B20_PERIOD_MS         5000   // Off / Run / anything but charging
#define DS18B20_PERIOD_CHARGE_MS  2000   // while charging
#define DS18B20_CONVERT_MS         800   // 12-bit conversion is spec'd <= 750 ms

// Per-cycle readings are far too chatty at the charging rate, but going quiet
// entirely would hide a dying sensor. Log a full summary on this interval, or
// immediately on any cycle where a sensor failed to read. Warnings from the
// individual read paths are never suppressed.
#define DS18B20_LOG_INTERVAL_MS  30000

#define OW_SKIP_ROM   0xCC
#define OW_MATCH_ROM  0x55
#define OW_CONVERT_T  0x44
#define OW_READ_PAD   0xBE

// Sensor ROM addresses (copied from read_battery_temps.ino)
static const uint8_t ds18b20_addr[DS18B20_COUNT][8] = {
    { 0x28, 0x49, 0xDB, 0xB3, 0x00, 0x00, 0x00, 0xCA },
    { 0x28, 0x94, 0x2F, 0xB3, 0x00, 0x00, 0x00, 0xFD },
    { 0x28, 0xCA, 0x30, 0xBA, 0x00, 0x00, 0x00, 0x40 },
    { 0x28, 0x28, 0x19, 0xBA, 0x00, 0x00, 0x00, 0x0C },
    { 0x28, 0xBE, 0x43, 0xB2, 0x00, 0x00, 0x00, 0x3B },
};

//static const uint8_t ds18b20_addr[DS18B20_COUNT][8] = {
//    { 0x28, 0x6D, 0xB5, 0xB2, 0x00, 0x00, 0x00, 0x9C }
//};

// Shared with main task; 32-bit float reads/writes are atomic on ESP32
static volatile float bat_ave_c = NAN;
static volatile float bat_min_c = NAN;
static volatile float bat_max_c = NAN;

static dac_oneshot_handle_t dac_handle  = NULL;
static portMUX_TYPE         ow_mux      = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// PWM outputs (LEDC) — analogue-style gauge drives
//   GPIO 32 → state of charge   (0 % SOC = 0 % duty, 100 % SOC = 100 % duty)
//   GPIO 33 → battery (BMS) temp (2-segment: -15 °C = 0 %, 30 °C = 50 %, 50 °C = 100 %)
// Both pins are free on the DOIT DEVKIT V1: not strapping pins, not input-only
// (34-39), clear of the SPI flash (6-11), the DAC (25/26), CAN (18/19),
// OneWire (15) and I2C (21/22).
//
// 5 kHz / 12-bit: fast enough to smooth into a clean DC level with a simple RC
// low-pass, while 4096 steps keep the quantisation well under a gauge's needle.
// =============================================================================
#define PWM_SOC_PIN        GPIO_NUM_32
#define PWM_BAT_TEMP_PIN   GPIO_NUM_33

#define PWM_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define PWM_TIMER          LEDC_TIMER_0
#define PWM_SOC_CHANNEL    LEDC_CHANNEL_0
#define PWM_BAT_CHANNEL    LEDC_CHANNEL_1
#define PWM_FREQ_HZ        5000
#define PWM_RESOLUTION     LEDC_TIMER_12_BIT
#define PWM_MAX_DUTY       ((1 << 12) - 1)

// Battery-temperature → duty mapping. A single straight line put normal ambient
// down near a quarter scale and bunched the useful range together. These three
// calibration points are instead joined by two linear segments (bat_temp_to_pct):
// mid-scale lands on typical ambient and the needle spreads out as it gets hot.
#define BAT_TEMP_PWM_MIN_C (-15.0f)   // → 0 %
#define BAT_TEMP_PWM_MID_C ( 30.0f)   // → 50 %
#define BAT_TEMP_PWM_MAX_C ( 50.0f)   // → 100 %

// --- Current telemetry values ---
static float speed_mph  = 0.0f;
static float temp_bat_c = 0.0f;
static float temp_mot_c = 0.0f;
static float temp_inv_c = 0.0f;
static float temp_isa_c = 0.0f;
static float soc_pct    = 100.0f;
static float udc_v      = 0.0f;
static float power_kw   = 0.0f;   // DC bus power from ZombieVerter (POWER_ID)
static float energy_wh  = 0.0f;   // cumulative energy counter, Wh (ZombieVerter KWh spot value x1000)
// volatile: written by the main loop (CPU 0), read by ds18b20_task (CPU 1) to
// pick its sample period. 32-bit aligned reads/writes are atomic on ESP32.
static volatile int opmode = 0;   // ZombieVerter opmode (0=Off … see OPMODE_ID)

// --- Timing ---
// I2C has no interval: frames are built on demand, when the master's read fires
// the on_request callback (see the I2C slave section).
static uint64_t lastSendTime   = 0;
static uint64_t lastSlowTime   = 0;
#define SEND_INTERVAL_MS   200   // ESP-NOW send rate + RPM poll
#define SLOW_INTERVAL_MS   500   // rotating SDO reads (SOC, temps, UDC)

// =============================================================================
// Helpers
// =============================================================================

static inline uint64_t millis(void) {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static inline float constrain_f(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Battery temperature → PWM duty percent: two straight segments joined at the
// midpoint, MIN→MID spanning 0..50 % and MID→MAX spanning 50..100 %.
//
// This replaced a quadratic through the same three points. The quadratic was
// smooth, but its vertex crept upward as the midpoint rose, and by MID = 28 C
// the vertex passed the low endpoint and the gauge ran *backwards* between
// -15 C and -13 C. The cold end also went nearly flat (0.08 %/C at MID = 27).
//
// The piecewise fit is monotonic by construction for any MIN < MID < MAX, and
// each half keeps a constant, legible slope. The cost is a slope discontinuity
// at MID (1.11 %/C below, 2.50 %/C above) — a slight kink as the needle crosses
// centre, which is a fair trade for a gauge that can never read backwards.
//
// The input is clamped, not the output, so an absurd reading pins to an end
// stop rather than landing somewhere plausible mid-scale.
static inline float bat_temp_to_pct(float tc) {
    const float x0 = BAT_TEMP_PWM_MIN_C;   // → 0 %
    const float x1 = BAT_TEMP_PWM_MID_C;   // → 50 %
    const float x2 = BAT_TEMP_PWM_MAX_C;   // → 100 %
    const float t  = constrain_f(tc, x0, x2);

    if (t <= x1) return         50.0f * (t - x0) / (x1 - x0);   // cold half
    else         return 50.0f + 50.0f * (t - x1) / (x2 - x1);   // hot half
}

// =============================================================================
// PWM (LEDC) helpers
// =============================================================================

static void pwm_init(void) {
    ledc_timer_config_t timer = {};
    timer.speed_mode      = PWM_SPEED_MODE;
    timer.timer_num       = PWM_TIMER;
    timer.duty_resolution = PWM_RESOLUTION;
    timer.freq_hz         = PWM_FREQ_HZ;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
        return;
    }

    ledc_channel_config_t ch = {};
    ch.speed_mode = PWM_SPEED_MODE;
    ch.timer_sel  = PWM_TIMER;
    ch.intr_type  = LEDC_INTR_DISABLE;
    ch.duty       = 0;
    ch.hpoint     = 0;

    ch.channel  = PWM_SOC_CHANNEL;
    ch.gpio_num = PWM_SOC_PIN;
    if (ledc_channel_config(&ch) != ESP_OK)
        ESP_LOGE(TAG, "LEDC SOC channel config failed");

    ch.channel  = PWM_BAT_CHANNEL;
    ch.gpio_num = PWM_BAT_TEMP_PIN;
    if (ledc_channel_config(&ch) != ESP_OK)
        ESP_LOGE(TAG, "LEDC battery-temp channel config failed");

    ESP_LOGI(TAG, "PWM ready (SOC GPIO %d, BMS temp GPIO %d, %d Hz, 12-bit)",
             PWM_SOC_PIN, PWM_BAT_TEMP_PIN, PWM_FREQ_HZ);
}

// pct is clamped to 0..100 before being scaled to the 12-bit duty range.
static void pwm_set_pct(ledc_channel_t channel, float pct) {
    uint32_t duty = (uint32_t)(constrain_f(pct, 0.0f, 100.0f) / 100.0f * PWM_MAX_DUTY + 0.5f);
    ledc_set_duty(PWM_SPEED_MODE, channel, duty);
    ledc_update_duty(PWM_SPEED_MODE, channel);
}

// Drive both gauge outputs from the latest telemetry values.
static void updatePwmOutputs(void) {
    pwm_set_pct(PWM_SOC_CHANNEL, soc_pct);

    // Curve-map the battery temperature onto 0..100 % duty (see bat_temp_to_pct).
    // Skip a NaN reading rather than slamming the output to 0 %.
    if (!isnan(temp_bat_c))
        pwm_set_pct(PWM_BAT_CHANNEL, bat_temp_to_pct(temp_bat_c));
}

// =============================================================================
// CANopen SDO helpers (mirrors CanSDO::requestSdoElement / CanSDO::GetValue)
// =============================================================================

static esp_err_t requestSdo(int id) {
    uint16_t index  = SDO_INDEX_BASE | (uint16_t)(id >> 8);
    uint8_t  subIdx = (uint8_t)(id & 0xFF);

    twai_message_t msg = {};
    msg.extd             = 0;
    msg.identifier       = SDO_REQ_ID;
    msg.data_length_code = 8;
    msg.data[0] = SDO_READ;
    msg.data[1] = (uint8_t)(index & 0xFF);
    msg.data[2] = (uint8_t)(index  >> 8);
    msg.data[3] = subIdx;
    // data[4-7] zero-initialised above
    return twai_transmit(&msg, pdMS_TO_TICKS(10));
}

// Discard anything already sitting in the RX queue. Called immediately before a
// request so the receive loop can only ever match a reply to *this* request.
//
// Without it, a reply that arrived too late to beat its own deadline stays queued
// and is picked up by the next call. A stale reply for a *different* param is
// harmless (the index/subindex check rejects it), but a stale reply for the *same*
// param passes every check and returns a value one poll-cycle old — a lagging
// speedo that looks like a working one. Timing out and returning NAN is the honest
// outcome there, and the callers already handle it.
static void flushRxQueue(void) {
    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) { /* drop */ }
}

// The VCU answers synchronously inside its own CAN receive handler (libopeninv
// cansdo.cpp: HandleRx -> ProcessSDO -> SendSdoReply), so a healthy reply lands
// within a millisecond or two. There is no server-side queue or rate limit to
// wait on, which means a deadline of tens of ms is already enormous headroom —
// so rather than spend the whole budget on one attempt, make two shorter ones.
// An occasional dropped frame then self-heals inside a single poll cycle instead
// of surfacing as a NAN, and the worst-case time per parameter is unchanged.
#define SDO_ATTEMPT_MS  25
#define SDO_ATTEMPTS     2

// One request/reply exchange. Returns the value, or NAN on timeout.
//
// `sawReply` reports whether any frame arrived during the wait. The acceptance
// filter admits only 0x583, so anything we receive IS an SDO reply — meaning
// sawReply=true on a timeout says a reply landed but for a *different* parameter
// (a late answer to an earlier request), while sawReply=false says the VCU sent
// nothing at all. `polls` counts how many times we were scheduled to look, which
// separates "the reply never came" from "we were never running to receive it".
static float sdoAttempt(int id, bool *sawReply, int *polls) {
    uint16_t index  = SDO_INDEX_BASE | (uint16_t)(id >> 8);
    uint8_t  subIdx = (uint8_t)(id & 0xFF);

    *sawReply = false;
    *polls    = 0;

    flushRxQueue();

    // A failed transmit means the request never left the board (TX queue full, or
    // the controller is bus-off/recovering). Waiting out the deadline for a reply
    // to a question we never asked would just log it as a "timeout" and hide the
    // real fault, so bail immediately and name it.
    esp_err_t txErr = requestSdo(id);
    if (txErr != ESP_OK) {
        ESP_LOGW(TAG, "SDO request for param %d not sent: %s", id, esp_err_to_name(txErr));
        return NAN;
    }

    // twai_receive blocks (1 tick = 10 ms at CONFIG_FREERTOS_HZ=100) so the loop
    // yields rather than busy-waits, and taskYIELD() lets IDLE0 pet the watchdog
    // even when frames arrive back-to-back.
    uint64_t deadline = millis() + SDO_ATTEMPT_MS;
    while (millis() < deadline) {
        twai_message_t rx = {};
        if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK) {
            *sawReply = true;
            if (rx.identifier       == SDO_REP_ID
                    && rx.data_length_code == 8
                    && rx.data[0]          != SDO_ABORT
                    && rx.data[1]          == (uint8_t)(index & 0xFF)
                    && rx.data[2]          == (uint8_t)(index  >> 8)
                    && rx.data[3]          == subIdx) {
                int32_t raw;
                memcpy(&raw, &rx.data[4], 4);
                return (float)(raw / 32.0);
            }
        }
        (*polls)++;
        taskYIELD();
    }
    return NAN;
}

static float getSdoValue(int id) {
    bool sawReply = false;
    int  polls    = 0;

    for (int attempt = 1; attempt <= SDO_ATTEMPTS; attempt++) {
        uint64_t started = millis();
        float    v       = sdoAttempt(id, &sawReply, &polls);
        if (!isnan(v)) {
            if (attempt > 1)
                ESP_LOGI(TAG, "SDO param %d recovered on attempt %d", id, attempt);
            return v;
        }
        uint64_t elapsed = millis() - started;

        // Overshooting the deadline by a wide margin, having barely been scheduled
        // to look, means the reply was probably never missed — we simply weren't
        // running to receive it. On this board the usual culprit is the WiFi stack
        // preempting the main task on CPU 0 (ESP-NOW sends from this same loop), not
        // anything on the CAN bus. Called out separately because no amount of CAN-side
        // tuning fixes a scheduling problem.
        if (elapsed > SDO_ATTEMPT_MS + 15 && polls <= 2) {
            ESP_LOGW(TAG, "SDO param %d: task starved (waited %llu ms, polled %d x) — "
                          "preempted, not a bus fault",
                     id, (unsigned long long)elapsed, polls);
        }
    }

    // "stray reply" = the VCU is answering, but we got an answer to an earlier
    // request instead of this one (we are running behind it). "silent" = no reply
    // reached us at all: the VCU never answered, or the reply was dropped/corrupted.
    ESP_LOGW(TAG, "SDO timeout for param %d after %d attempts (%s)",
             id, SDO_ATTEMPTS, sawReply ? "stray reply seen — running behind" : "VCU silent");
    return NAN;
}

// =============================================================================
// TWAI (CAN) health monitor — the controller can accumulate bus errors (e.g.
// from noise or a momentarily disconnected transceiver) until it trips into
// TWAI_STATE_BUS_OFF, after which it stops transmitting/receiving entirely
// and never recovers on its own. Poll status each loop tick and drive the
// bus-off -> recovering -> stopped -> running restart sequence automatically
// instead of requiring a manual ESP32 reset.
// =============================================================================
static void checkTwaiHealth(void) {
    static uint64_t lastWarnLog = 0;

    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return;

    if (status.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGE(TAG, "TWAI bus-off (tx_err=%lu rx_err=%lu bus_err=%lu) - initiating recovery",
                 (unsigned long)status.tx_error_counter,
                 (unsigned long)status.rx_error_counter,
                 (unsigned long)status.bus_error_count);
        twai_initiate_recovery();
        return;
    }

    if (status.state == TWAI_STATE_STOPPED) {
        ESP_LOGW(TAG, "TWAI stopped (recovery complete) - restarting driver");
        if (twai_start() == ESP_OK) ESP_LOGI(TAG, "TWAI restarted OK");
        return;
    }

    if (status.state == TWAI_STATE_RUNNING &&
            (status.tx_error_counter > 32 || status.rx_error_counter > 32)) {
        uint64_t now = millis();
        if (now - lastWarnLog >= 1000) {
            ESP_LOGW(TAG, "TWAI error counters elevated: tx=%lu rx=%lu bus_err=%lu",
                     (unsigned long)status.tx_error_counter,
                     (unsigned long)status.rx_error_counter,
                     (unsigned long)status.bus_error_count);
            lastWarnLog = now;
        }
    }

    // Frames the driver had to throw away because the RX queue was full
    // (rx_missed) or the hardware FIFO overran (rx_overrun). With the acceptance
    // filter admitting only 0x583, these should sit at zero — anything else means
    // SDO replies are still being dropped before getSdoValue can read them, which
    // is exactly what the timeouts look like from the outside. Report only when
    // the count actually moves, so a healthy bus stays silent.
    static uint32_t lastMissed = 0, lastOverrun = 0;
    if (status.rx_missed_count != lastMissed || status.rx_overrun_count != lastOverrun) {
        ESP_LOGW(TAG, "TWAI dropped RX frames: missed=%lu overrun=%lu (SDO replies may be lost)",
                 (unsigned long)status.rx_missed_count,
                 (unsigned long)status.rx_overrun_count);
        lastMissed  = status.rx_missed_count;
        lastOverrun = status.rx_overrun_count;
    }
}

// =============================================================================
// I2C slave — serves telemetry to the waveform display master (waveform_tr_display)
// SCL = GPIO 22, SDA = GPIO 21, slave address 0x42
//
// Each snapshot is wrapped in a self-describing frame the master can resync to:
//     [SYNC0][SYNC1][ TelemetryPacket payload ][CRC8 over payload]
// The frame is larger than the 32-byte HW TX FIFO and the master polls on its own
// schedule, so a raw read can land mid-packet. The master therefore reads a
// two-frame window, scans it for the sync marker (newest first) and accepts the
// payload only if the CRC checks out — torn or underrun reads are rejected rather
// than displayed. (The original ESP32 has no I2C RAM/snapshot mode, so framing is
// the fix here.) This wire format is unchanged; the display needs no edits.
//
// Data is served ON DEMAND, from the master's read, via the v2 slave driver's
// on_request callback — NOT pre-queued into the TX ring buffer. That is deliberate,
// and it is what stops the board crashing:
//
//   i2c_slave_transmit() (v1) pushes bytes into the ring buffer BEFORE updating
//   trans_data_length under the spinlock. If the TX interrupt is already enabled
//   (a previous frame still draining) the ISR can fire in that gap, pull the bytes
//   just added, and find its local size exceeds the stale trans_data_length. That
//   path logs "I2C TX BUFFER SIZE ERROR" and then neither decrements the counter
//   nor disables the TX interrupt — so the level-triggered TX-FIFO-watermark
//   interrupt re-asserts forever, storming CPU0 until the interrupt watchdog
//   panics the chip. Keeping a backlog topped up (as the old design did, writing
//   every 40 ms into a 4-frame buffer) sits on that window continuously.
//
// Serving on request means there is never a pending frame when we write, so the
// racing path is never executed. It also means the master always gets the NEWEST
// snapshot instead of one up to four frames stale, and the whole stall-detect /
// backoff / recreate-the-device apparatus is gone: if the master stops reading,
// on_request simply stops firing and we go quiet on our own.
//
// Requires CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2=y (see sdkconfig.defaults).
// =============================================================================
#define I2C_SCL_PIN    GPIO_NUM_22
#define I2C_SDA_PIN    GPIO_NUM_21

//#define I2C_SCL_PIN    GPIO_NUM_12
//#define I2C_SDA_PIN    GPIO_NUM_14

#define I2C_SLAVE_ADDR 0x42

#define I2C_SYNC0      0xAAu
#define I2C_SYNC1      0x55u
#define I2C_FRAME_LEN  (2 + (int)sizeof(TelemetryPacket) + 1)   // sync(2)+payload+crc(1)

// The master reads a two-frame window and scans it newest-first (I2C_READ_LEN in
// waveform_tr_display/main/i2c_rx.c). Serve exactly that: two copies of the current
// frame, so whatever byte its read lands on, a whole valid frame follows.
#define I2C_WINDOW_LEN (I2C_FRAME_LEN * 2)

// The write happens in task context, not in the callback: i2c_slave_write() takes a
// mutex and can block, so it has no business running in an ISR. The callback only
// posts the request (this is the pattern in IDF's own i2c_slave_network_sensor
// example). The master clock-stretches while it waits, which is what buys the task
// time to respond.
#define I2C_WRITE_TIMEOUT_MS 50

static i2c_slave_dev_handle_t i2c_slave     = NULL;
static QueueHandle_t          i2c_req_queue = NULL;

static uint8_t ow_crc8(const uint8_t *data, int len);   // defined with the OneWire helpers
static void    fillPacket(void);                        // defined with the ESP-NOW helpers

// ISR context — keep it to posting the event and nothing else.
static IRAM_ATTR bool onI2cRequest(i2c_slave_dev_handle_t /*dev*/,
                                   const i2c_slave_request_event_data_t * /*evt*/,
                                   void * /*arg*/) {
    BaseType_t taskWoken = pdFALSE;
    uint8_t    evt       = 1;
    xQueueSendFromISR(i2c_req_queue, &evt, &taskWoken);
    return taskWoken == pdTRUE;
}

// Builds a fresh frame per request and hands it over. Blocks on the queue when the
// master isn't reading, so an absent master costs nothing and logs nothing.
static void i2c_slave_task(void *) {
    uint8_t window[I2C_WINDOW_LEN];

    while (true) {
        uint8_t evt;
        if (xQueueReceive(i2c_req_queue, &evt, portMAX_DELAY) != pdTRUE) continue;

        fillPacket();   // latest values (opmode-gated UDC, etc.) at the moment of the read

        window[0] = I2C_SYNC0;
        window[1] = I2C_SYNC1;
        memcpy(&window[2], &packet, sizeof(packet));
        window[2 + sizeof(packet)] = ow_crc8((const uint8_t *)&packet, sizeof(packet));
        memcpy(&window[I2C_FRAME_LEN], window, I2C_FRAME_LEN);   // second copy

        uint32_t total = 0;
        while (total < sizeof(window)) {
            uint32_t  written = 0;
            esp_err_t err     = i2c_slave_write(i2c_slave, window + total,
                                                sizeof(window) - total, &written,
                                                I2C_WRITE_TIMEOUT_MS);
            if (err != ESP_OK || written == 0) {
                // A timeout here means the master started a read and then stopped
                // clocking (unplugged mid-transaction, or a noisy bus). Nothing to
                // recover — the next read starts a fresh frame. Rate-limit so a
                // disconnected master can't flood the log.
                static uint64_t lastWarn = 0;
                if (millis() - lastWarn >= 1000) {
                    ESP_LOGW(TAG, "I2C write stalled after %lu/%d bytes: %s — master stopped clocking?",
                             (unsigned long)total, (int)sizeof(window), esp_err_to_name(err));
                    lastWarn = millis();
                }
                break;
            }
            total += written;
        }
    }
}

static bool i2cSlaveStart(void) {
    i2c_req_queue = xQueueCreate(4, sizeof(uint8_t));
    if (i2c_req_queue == NULL) {
        ESP_LOGE(TAG, "I2C request queue create failed");
        return false;
    }

    i2c_slave_config_t cfg = {};
    cfg.i2c_port          = I2C_NUM_0;
    cfg.scl_io_num        = I2C_SCL_PIN;
    cfg.sda_io_num        = I2C_SDA_PIN;
    cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    cfg.slave_addr        = I2C_SLAVE_ADDR;
    cfg.addr_bit_len      = I2C_ADDR_BIT_LEN_7;
    // One window in flight at a time — we write only in response to a read, so there
    // is no backlog to size for.
    cfg.send_buf_depth    = I2C_WINDOW_LEN;
    cfg.receive_buf_depth = I2C_WINDOW_LEN;   // unused (master never writes) but required

    esp_err_t err = i2c_new_slave_device(&cfg, &i2c_slave);
    if (err != ESP_OK) {
        i2c_slave = NULL;
        ESP_LOGE(TAG, "I2C slave init failed: %s", esp_err_to_name(err));
        return false;
    }

    i2c_slave_event_callbacks_t cbs = {};
    cbs.on_request = onI2cRequest;
    err = i2c_slave_register_event_callbacks(i2c_slave, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C slave callback register failed: %s", esp_err_to_name(err));
        return false;
    }

    // Priority above the main loop so a master read is answered promptly (it is
    // stretching the clock until we do). CPU 1 keeps it clear of the WiFi stack.
    xTaskCreatePinnedToCore(i2c_slave_task, "i2c_tx", 4096, NULL, 10, NULL, 1);
    return true;
}

// =============================================================================
// ESP-NOW
// =============================================================================

// Populate the I2C telemetry packet from the current values. Called from
// i2c_slave_task at the moment the master reads, so the frame is always current.
static void fillPacket(void) {
    packet.speed_mph  = speed_mph;
    packet.temp_bat_c = temp_bat_c;
    packet.temp_mot_c = temp_mot_c;
    packet.temp_inv_c = temp_inv_c;
    packet.temp_isa_c = temp_isa_c;
    packet.soc_pct    = (opmode == MOD_RUN || opmode == MOD_CHARGE) ? soc_pct : DEFAULT_SOC_PCT;
    // UDC is only valid with the pack contactors closed (Run or Charge);
    // in every other opmode report a nominal default instead of a stale/0 V read.
    packet.udc_v      = (opmode == MOD_RUN || opmode == MOD_CHARGE) ? udc_v : DEFAULT_UDC_V;
    // Raw opmode so the displays can show Run / Charging / Off themselves.
    packet.opmode     = (uint8_t)constrain_f((float)opmode, 0.0f, 255.0f);
    // Cumulative energy counter, forwarded in Wh. The display owns the
    // trip/lifetime/charge bookkeeping by integrating deltas of this value.
    packet.energy_wh  = energy_wh;
    packet.ready      = 0xA5;
}

static __attribute__((unused)) void sendPacket(void) {
    // ESP-NOW feeds the power-only Qualia. Independent of the I2C TelemetryPacket
    // (the full dash to the waveform display), so this never affects that link.
    EspNowPowerPacket p;
    p.power_kw = power_kw;
    p.ready    = 0xA5;
    esp_now_send(receiverMAC, (uint8_t *)&p, sizeof(p));
    // I2C is sent on its own faster cadence (see main loop), not here — the master
    // polls ~50 Hz and needs frames more often than the 200 ms ESP-NOW tick.
}

// IDF 5.5 changed the first arg from (const uint8_t *mac) to (const wifi_tx_info_t *)
static void onSent(const wifi_tx_info_t * /*info*/, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        espNowErrorCount = 0;
    } else {
        espNowErrorCount = espNowErrorCount + 1;
        ESP_LOGW(TAG, "ESP-NOW send failed (%d/%d)", espNowErrorCount, ESP_NOW_ERROR_LIMIT);
        if (espNowErrorCount >= ESP_NOW_ERROR_LIMIT)
            espNowNeedsReinit = true;
    }
}

static void reinitEspNow(void) {
    ESP_LOGI(TAG, "Reinitializing ESP-NOW...");
    esp_now_del_peer(receiverMAC);
    esp_now_deinit();
    vTaskDelay(pdMS_TO_TICKS(200));

    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW reinit failed - will retry");
        return;
    }
    esp_now_register_send_cb(onSent);

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-add peer - will retry");
        return;
    }

    espNowErrorCount  = 0;
    espNowNeedsReinit = false;
    ESP_LOGI(TAG, "ESP-NOW reinitialized OK");
}


// =============================================================================
// OneWire bit-bang (open-drain on DS18B20_PIN)
// Each bit slot is ~70 µs; portENTER_CRITICAL keeps the timing tight.
// =============================================================================
static inline void ow_lo(void) { gpio_set_level(DS18B20_PIN, 0); }
static inline void ow_hi(void) { gpio_set_level(DS18B20_PIN, 1); }
static inline int  ow_rd(void) { return gpio_get_level(DS18B20_PIN); }

static bool ow_reset(void) {
    // Hold low ≥480 µs — no critical section needed, just needs to be long enough
    ow_lo();
    esp_rom_delay_us(480);
    // Release and sample presence pulse within a critical section for accurate timing.
    // Matches the standard OneWire library reset: sample once ~70 us after release,
    // then just spend the remaining ~410 us as recovery time. A DS18B20's presence
    // pulse can legitimately still be low well past that if it's a clone chip or
    // several devices are settling out of sync right after a simultaneous
    // conversion — re-sampling here and overriding `present` on a still-low line
    // turned those legitimate (if slow) presence pulses into false negatives.
    portENTER_CRITICAL(&ow_mux);
    ow_hi();
    esp_rom_delay_us(70);
    bool present = (ow_rd() == 0);
    portEXIT_CRITICAL(&ow_mux);
    esp_rom_delay_us(410);
    return present;
}

static void ow_write_bit(bool b) {
    portENTER_CRITICAL(&ow_mux);
    ow_lo();
    if (b) {
        esp_rom_delay_us(6);   // write-1: release early
        ow_hi();
        esp_rom_delay_us(64);
    } else {
        esp_rom_delay_us(60);  // write-0: hold low most of slot
        ow_hi();
        esp_rom_delay_us(10);
    }
    portEXIT_CRITICAL(&ow_mux);
}

static bool ow_read_bit(void) {
    bool b;
    portENTER_CRITICAL(&ow_mux);
    ow_lo();
    esp_rom_delay_us(6);
    ow_hi();
    esp_rom_delay_us(9);
    b = (ow_rd() != 0);
    esp_rom_delay_us(55);
    portEXIT_CRITICAL(&ow_mux);
    return b;
}

static void ow_write_byte(uint8_t v) {
    for (int i = 0; i < 8; i++) ow_write_bit((v >> i) & 1);
}

static uint8_t ow_read_byte(void) {
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) if (ow_read_bit()) v |= (1 << i);
    return v;
}

// Dallas/Maxim CRC8 (poly x^8+x^5+x^4+1, LSB-first) over ROM codes and scratchpads.
static uint8_t ow_crc8(const uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int b = 0; b < 8; b++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

// =============================================================================
// SEARCH ROM (0xF0) — enumerates every device on the bus.
// Standard Dallas/Maxim bit-walk algorithm; works for any number of sensors.
// =============================================================================
#define OW_SEARCH_ROM 0xF0

typedef struct {
    uint8_t rom[8];
    int     last_discrepancy;
    bool    last_device_flag;
} ow_search_t;

static void ow_search_init(ow_search_t *s) {
    memset(s->rom, 0, 8);
    s->last_discrepancy  = 0;
    s->last_device_flag  = false;
}

// Returns true and fills s->rom with the next device address, or false when done.
static bool ow_search_next(ow_search_t *s) {
    if (s->last_device_flag) return false;
    if (!ow_reset()) {
        ow_search_init(s);
        return false;
    }
    ow_write_byte(OW_SEARCH_ROM);

    uint8_t  new_rom[8] = {};
    int      last_zero  = 0;
    for (int bit = 1; bit <= 64; bit++) {
        bool id_bit  = ow_read_bit();
        bool cmp_bit = ow_read_bit();
        if (id_bit && cmp_bit) {            // no devices on bus
            ow_search_init(s);
            return false;
        }
        bool dir;
        if (id_bit != cmp_bit) {
            dir = id_bit;                   // only one value present
        } else {
            // conflict: choose direction based on previous pass
            if (bit < s->last_discrepancy)
                dir = (s->rom[(bit - 1) / 8] >> ((bit - 1) % 8)) & 1;
            else
                dir = (bit == s->last_discrepancy);
            if (!dir) last_zero = bit;
        }
        if (dir) new_rom[(bit - 1) / 8] |= (1 << ((bit - 1) % 8));
        ow_write_bit(dir);
    }

    memcpy(s->rom, new_rom, 8);
    s->last_discrepancy = last_zero;
    s->last_device_flag = (last_zero == 0);
    return true;
}

#define DS18B20_FAMILY_CODE 0x28

// Print every DS18B20 address found on the bus in ds18b20_addr[][] format.
// A ROM code is only reported as real if its family code is 0x28 (DS18B20)
// and its CRC8 checks out — a bus with no device attached (or a pull-up too
// weak to ever read back a "1") makes ow_search_next() manufacture bogus
// 0x00-family addresses instead of finding nothing, so those must be filtered
// out explicitly rather than trusted at face value.
static void ds18b20_enumerate(void) {
    ESP_LOGI(TAG, "--- DS18B20 SEARCH on GPIO %d ---", DS18B20_PIN);
    ow_search_t s;
    ow_search_init(&s);
    int n = 0, valid = 0;
    while (ow_search_next(&s)) {
        if (++n > 16) { ESP_LOGE(TAG, "search loop limit — bus stuck or shorted"); break; }
        bool ok = (s.rom[0] == DS18B20_FAMILY_CODE) && (ow_crc8(s.rom, 7) == s.rom[7]);
        ESP_LOGI(TAG, " ds18b20_addr [%d] { 0x%02X, 0x%02X, 0x%02X, 0x%02X,"
                      " 0x%02X, 0x%02X, 0x%02X, 0x%02X }%s",
                 n,
                 s.rom[0], s.rom[1], s.rom[2], s.rom[3],
                 s.rom[4], s.rom[5], s.rom[6], s.rom[7],
                 ok ? "" : "  <- bogus (bad family code/CRC), ignore");
        if (ok) valid++;
    }
    if (valid == 0)
        ESP_LOGW(TAG, "  no real DS18B20 found — check wiring/pull-up on GPIO %d", DS18B20_PIN);
    else
        ESP_LOGI(TAG, "  %d real device(s) found (%d total addresses seen)", valid, n);
    ESP_LOGI(TAG, "---------------------------------");
}

// Sleep whatever is left of this cycle's period, so the CAN broadcast rate is a
// true period rather than "work time + fixed sleep". An over-budget cycle just
// starts the next one immediately instead of accumulating drift.
static void ds18b20_pace(uint64_t cycle_start, uint32_t period_ms) {
    uint64_t elapsed = millis() - cycle_start;
    if (elapsed < period_ms) vTaskDelay(pdMS_TO_TICKS(period_ms - elapsed));
    else                     vTaskDelay(1);   // yield so IDLE1 can pet the watchdog
}

// =============================================================================
// DS18B20 task — samples all sensors once per cycle and broadcasts the pack
// temps to the ZombieVerter on CAN 0x4F1. Pinned to CPU 1 so the 800 ms
// conversion wait never blocks the main CAN/ESP-NOW loop on CPU 0.
// Cycle period is DS18B20_PERIOD_CHARGE_MS while charging, else DS18B20_PERIOD_MS.
// =============================================================================
static void ds18b20_task(void *) {
    // Configure pin as open-drain with internal pull-up (no external resistor needed)
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << DS18B20_PIN;
    io.mode         = GPIO_MODE_INPUT_OUTPUT_OD;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);
    ow_hi();  // idle high

    ds18b20_enumerate();  // scan bus and print all addresses once on boot

    while (true) {
        // Sample the mode once per cycle so a mode change mid-cycle can't make
        // the pacing at the bottom disagree with the deadline set at the top,
        // nor the broadcast decision disagree with the period.
        uint64_t cycle_start = millis();
        bool     charging    = (opmode == MOD_CHARGE);
        uint32_t period_ms   = charging ? DS18B20_PERIOD_CHARGE_MS
                                        : DS18B20_PERIOD_MS;

        // Broadcast conversion (SKIP ROM) to all sensors at once, then read
        // each individually (MATCH ROM) — reverted from per-sensor conversion,
        // which doubled OneWire bus traffic per cycle (two full reset+8-byte-
        // address round trips per sensor instead of one shared broadcast + one
        // round trip) and made every sensor twice as exposed to intermittent
        // bus noise. This is the simpler, previously-working design.
        if (!ow_reset()) {
            ESP_LOGW(TAG, "DS18B20: no presence pulse on GPIO %d — check wiring", DS18B20_PIN);
            ds18b20_pace(cycle_start, period_ms);
            continue;
        }
        ow_write_byte(OW_SKIP_ROM);
        ow_write_byte(OW_CONVERT_T);
        vTaskDelay(pdMS_TO_TICKS(DS18B20_CONVERT_MS));

        float sum = 0.0f, mn = 200.0f, mx = -200.0f;
        int valid = 0;
        float readings[DS18B20_COUNT];
        for (int i = 0; i < DS18B20_COUNT; i++) readings[i] = NAN;
        for (int i = 0; i < DS18B20_COUNT; i++) {
            if (!ow_reset()) {
                ESP_LOGW(TAG, "DS18B20[%d]: no presence pulse before read — skipping", i);
                continue;
            }
            ow_write_byte(OW_MATCH_ROM);
            for (int b = 0; b < 8; b++) ow_write_byte(ds18b20_addr[i][b]);
            ow_write_byte(OW_READ_PAD);
            uint8_t sp[9];
            for (int b = 0; b < 9; b++) sp[b] = ow_read_byte();
            if (sp[0] == 0xFF && sp[1] == 0xFF) {
                ESP_LOGW(TAG, "DS18B20[%d]: no response to MATCH ROM (absent) — skipping", i);
                continue;
            }
            // A dead/miswired bus (line never rising to a valid "1") reads back
            // as all-0x00 or all-0xFF, and CRC8 of an all-0x00 scratchpad is 0x00
            // — a fixed point that would otherwise slip past the CRC check below.
            bool all_zero = true, all_ff = true;
            for (int b = 0; b < 9; b++) {
                if (sp[b] != 0x00) all_zero = false;
                if (sp[b] != 0xFF) all_ff = false;
            }
            if (all_zero || all_ff) {
                ESP_LOGW(TAG, "DS18B20[%d]: degenerate read (bus stuck?) — skipping", i);
                continue;
            }
            if (ow_crc8(sp, 8) != sp[8]) {
                ESP_LOGW(TAG, "DS18B20[%d]: CRC mismatch — skipping bad read", i);
                continue;
            }
            int16_t raw = (int16_t)((sp[1] << 8) | sp[0]);
            // CRC8 only proves the scratchpad bytes are self-consistent — it says
            // nothing about whether the value is physically plausible or fresh.
            // 0x0550 (85.00 C) is the DS18B20's power-on-reset default, which has
            // a perfectly valid CRC of its own; seeing it means CONVERT T most
            // likely never landed (bus glitch) and this is stale, not a real
            // reading. Anything outside the sensor's -55..125 C spec range is
            // similarly a corrupted read that happened to pass the 8-bit CRC by
            // chance (roughly 1-in-256 odds), not a real temperature.
            if (raw == 0x0550) {
                ESP_LOGW(TAG, "DS18B20[%d]: reads power-on default (85.00 C) — CONVERT T likely missed, skipping", i);
                continue;
            }
            float tc = raw / 16.0f;
            if (tc < -55.0f || tc > 125.0f) {
                ESP_LOGW(TAG, "DS18B20[%d]: out-of-spec reading %.2f C — skipping", i, tc);
                continue;
            }
            ESP_LOGD(TAG, "DS18B20[%d]: %.2f C", i, tc);
            readings[i] = tc;
            sum += tc;
            if (tc < mn) mn = tc;
            if (tc > mx) mx = tc;
            valid++;
        }

        // Summarise on the log interval, or straight away if a sensor dropped out.
        // Always log the first cycle so the boot-time readings are visible.
        static uint64_t lastLogTime = 0;
        static bool     loggedOnce  = false;
        bool any_failed = (valid < DS18B20_COUNT);
        bool log_now    = any_failed || !loggedOnce ||
                          (millis() - lastLogTime >= DS18B20_LOG_INTERVAL_MS);
        if (log_now) { lastLogTime = millis(); loggedOnce = true; }

        if (log_now) {
            char line[128];
            int  len = 0;
            len += snprintf(line + len, sizeof(line) - len, "DS18B20 readings:");
            for (int i = 0; i < DS18B20_COUNT; i++) {
                if (isnan(readings[i]))
                    len += snprintf(line + len, sizeof(line) - len, " [%d]=FAIL", i);
                else
                    len += snprintf(line + len, sizeof(line) - len, " [%d]=%.2fC", i, readings[i]);
            }
            ESP_LOGI(TAG, "%s", line);
        }

        if (valid > 0) {
            float ave = sum / valid;
            bat_ave_c = ave;
            bat_min_c = mn;
            bat_max_c = mx;

            // Map average temp (−20…100 °C) to DAC byte (0…255) on GPIO 25
            uint8_t dac_val = (uint8_t)(constrain_f((ave + 20.0f) / 120.0f * 255.0f, 0.0f, 255.0f));
            dac_oneshot_output_voltage(dac_handle, dac_val);
            if (log_now)
                ESP_LOGI(TAG, "DS18B20 ave:%.1f min:%.1f max:%.1f DAC:%u", ave, mn, mx, dac_val);

            // Charging safety interlock: if any sensor failed to report, the min/max
            // we computed silently omit that cell — the dead sensor could be the hot
            // one. Rather than hand the BMS a confident-looking but incomplete
            // reading, withhold the broadcast entirely. ZombieVerter's CustomBMS
            // (custombms.cpp) then times out after BMS_Timeout (default 10 s):
            // BMSDataValid() -> false, ChargeAllowed() -> false, MaxChargeCurrent()
            // -> 0, which commands an AC charge shutdown, and BMS_T* read -100.
            //
            // Silence is the fail-safe signal here. Sending deliberately corrupt
            // bytes would NOT be: CustomBMS::DecodeCAN does no CRC/plausibility
            // check, so garbage decodes straight into avgTempC/minTempC/maxTempC
            // and could read as a perfectly acceptable cool pack — letting a full
            // charge continue with no working temperature sensing.
            if (charging && any_failed) {
                ESP_LOGW(TAG, "DS18B20: %d/%d sensors reporting while charging — "
                              "withholding CAN 0x4F1 so the BMS times out and stops charging",
                         valid, DS18B20_COUNT);
            } else {
                // Broadcast battery temps on CAN bus (0x4F1, 3 signed bytes: ave, min, max)
                twai_message_t msg = {};
                msg.identifier       = 0x4F1;
                msg.data_length_code = 3;
                msg.data[0] = (uint8_t)(int8_t)constrain_f(ave, -128.0f, 127.0f);
                msg.data[1] = (uint8_t)(int8_t)constrain_f(mn,  -128.0f, 127.0f);
                msg.data[2] = (uint8_t)(int8_t)constrain_f(mx,  -128.0f, 127.0f);
                esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
                if (err != ESP_OK)
                    ESP_LOGW(TAG, "DS18B20 CAN TX failed: %s", esp_err_to_name(err));   // never suppressed
                else if (log_now)
                    ESP_LOGI(TAG, "DS18B20 CAN TX id=0x%03X ave=%d min=%d max=%d",
                             msg.identifier, (int8_t)msg.data[0], (int8_t)msg.data[1], (int8_t)msg.data[2]);
            }
        }

        // Sleep whatever is left of this cycle's period
        ds18b20_pace(cycle_start, period_ms);
    }
}

// =============================================================================
// app_main
// =============================================================================

extern "C" void app_main(void) {
    // NVS is required by the WiFi driver
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // --- CAN / TWAI init ---
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 32;
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();

    // Accept ONLY the SDO reply (0x583) in hardware — everything else on the bus is
    // rejected by the controller and never reaches the RX queue.
    //
    // This is what fixes the frequent SDO timeouts. The queue is drained in exactly
    // one place (getSdoValue's receive loop), which returns the moment it finds its
    // reply — so for the ~190 ms between polls nothing drains it while the bus keeps
    // filling it. Under the previous ACCEPT_ALL filter, foreign traffic (inverter,
    // BMS, VCU broadcasts) overran all 32 slots within that gap, and once the queue
    // is full the controller drops *new* arrivals — including the 0x583 reply we
    // were waiting on. The VCU answered promptly; the answer was binned before we
    // ever looked at it, and we sat out the full 50 ms deadline and called it a
    // timeout. Filtering in hardware means a foreign frame can never take the slot.
    //
    // Nothing else needs receiving: the IVT 0x528 snoop was the only other consumer
    // of the RX path, and it's gone (energy now comes from the KWh spot value). The
    // filter does not affect transmission, so the 0x4F1 battery-temp broadcast and
    // the 0x603 SDO requests are unaffected.
    //
    // Single-filter format for 11-bit IDs: the ID sits in the top 11 bits of the
    // 32-bit acceptance code; a set mask bit means "don't care", so masking off
    // everything below the ID ignores RTR and the first two data bytes.
    twai_filter_config_t f_config = {};
    f_config.acceptance_code = ((uint32_t)SDO_REP_ID) << 21;
    f_config.acceptance_mask = ~(((uint32_t)0x7FF) << 21);
    f_config.single_filter   = true;

    while (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "TWAI install failed - retrying in 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    while (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed - retrying in 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "CAN ready (500 kbps, TWAI)");

    // --- WiFi init (required by ESP-NOW) ---
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    // Lock to channel 1 — CircuitPython defaults to ch1 when not joined to an AP.
    // peerInfo.channel=0 would follow whatever channel IDF picks, which is
    // undefined when not associated and causes silent delivery failures.
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    // Strip LR from the protocol set. IDF 5.x includes WIFI_PROTOCOL_LR by
    // default; CircuitPython devices don't speak ESP-NOW LR and will drop it.
    esp_wifi_set_protocol(WIFI_IF_STA,
                          WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    // Disable modem-sleep so ESP-NOW TX is not delayed by power-save windows.
    esp_wifi_set_ps(WIFI_PS_NONE);

    // --- ESP-NOW init ---
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    esp_now_register_send_cb(onSent);

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // --- DAC init (GPIO 25 = DAC_CHAN_0 → analogue average battery temperature) ---
    dac_oneshot_config_t dac_cfg = { .chan_id = DAC_CHAN_0 };
    dac_oneshot_new_channel(&dac_cfg, &dac_handle);

    // --- PWM init (GPIO 32 = SOC, GPIO 33 = BMS temperature) ---
    pwm_init();

    // --- I2C slave init (SCL GPIO 22, SDA GPIO 21, addr 0x42) ---
    // Also starts i2c_slave_task, which serves a frame whenever the master reads.
    if (i2cSlaveStart())
        ESP_LOGI(TAG, "I2C slave ready (SCL GPIO %d, SDA GPIO %d, addr 0x%02X)",
                 I2C_SCL_PIN, I2C_SDA_PIN, I2C_SLAVE_ADDR);

    // --- DS18B20 task (CPU 1, 4 kB stack, reads sensors every 10 s) ---
    xTaskCreatePinnedToCore(ds18b20_task, "ds18b20", 4096, NULL, 2, NULL, 1);

    lastSendTime = millis();
    ESP_LOGI(TAG, "Ready - reading CAN, forwarding via ESP-NOW");

    // =============================================================================
    // Main loop — RPM and power polled at SEND_INTERVAL_MS (200 ms, 5 Hz).
    // Slow params (SOC, temps, UDC+opmode, energy) rotate one slot per
    // SLOW_INTERVAL_MS across 7 slots; the UDC slot reads opmode too, so the
    // ZombieVerter SDO server sees at most 12 requests/second total.
    // =============================================================================
    uint8_t fetchIndex = 0;
    while (true) {
        checkTwaiHealth();

        if (espNowNeedsReinit)
            reinitEspNow();

        uint64_t now = millis();

        // Fast tick — 5 Hz. RPM (speed) and power both need to track the throttle,
        // so both are polled every tick (two SDO requests) rather than rotated in
        // with the slow params. On a healthy bus each reply lands within a few ms.
        if (now - lastSendTime >= SEND_INTERVAL_MS) {
            // On an SDO timeout the old code simply kept the previous speed_mph.
            // That's the wrong failure mode for an odometer: the display integrates
            // this value continuously, so a held 20 mph while the RPM reads are
            // failing invents distance the car never travelled. After a short
            // streak of failures, report 0 — "unknown" must not read as "moving".
            static int rpmFailStreak = 0;
            float v = getSdoValue(MOTOR_RPM_PARAM_ID);
            if (!isnan(v)) {
                speed_mph = constrain_f(v * 0.00981f, 0.0f, 100.0f);
                rpmFailStreak = 0;
            } else if (++rpmFailStreak >= 3 && speed_mph != 0.0f) {
                ESP_LOGW(TAG, "RPM SDO failed %dx — zeroing speed so the odometer stops accumulating",
                         rpmFailStreak);
                speed_mph = 0.0f;
            }

            v = getSdoValue(POWER_ID);
            if (!isnan(v)) power_kw = v;

            // DS18B20 gives direct cell readings every 10 s; prefer those over
            // the ZombieVerter's SDO BMS_T_MAX when fresh readings are available.
            if (!isnan(bat_max_c)) temp_bat_c = bat_max_c;

            lastSendTime = millis();
            sendPacket();
            updatePwmOutputs();
            ESP_LOGI(TAG, "spd:%.1f soc:%.1f bat:%.1f mot:%.1f inv:%.1f isa:%.1f udc:%.1f pwr:%.1f wh:%.0f op:%d",
                     speed_mph, soc_pct, temp_bat_c, temp_mot_c, temp_inv_c, temp_isa_c, udc_v, power_kw, energy_wh, opmode);
        }

        // (No I2C send here — i2c_slave_task builds a frame when the master reads.)

        // Slow params — rotating across 7 slots (the UDC slot also reads opmode)
        if (now - lastSlowTime >= SLOW_INTERVAL_MS) {
            float v;
            switch (fetchIndex % 7) {
                case 0:
                    v = getSdoValue(SOC_VALUE_ID);
                    if (!isnan(v)) soc_pct    = constrain_f(v, 0.0f, 100.0f);
                    break;
                case 1:
                    v = getSdoValue(BMS_T_MAX_VALUE_ID);
                    if (!isnan(v)) temp_bat_c = v;
                    break;
                case 2:
                    v = getSdoValue(INVERTER_TEMP_ID);
                    if (!isnan(v)) temp_inv_c = v;
                    break;
                case 3:
                    v = getSdoValue(MOTOR_TEMP_ID);
                    if (!isnan(v)) temp_mot_c = v;
                    break;
                case 4:
                    v = getSdoValue(ISA_TEMP_ID);
                    if (!isnan(v)) temp_isa_c = v;
                    break;
                case 5:
                    // UDC and opmode are read together: opmode gates whether the
                    // UDC reading is forwarded (Run/Charge) or replaced by the
                    // default, so both must be refreshed on the same tick.
                    v = getSdoValue(UDC_IDX);
                    if (!isnan(v)) udc_v      = v;
                    v = getSdoValue(OPMODE_ID);
                    if (!isnan(v)) opmode     = (int)lroundf(v);
                    break;
                case 6:
                    // Cumulative energy, kWh from the VCU -> Wh on the wire. Being a
                    // running total, a slow poll loses no energy: the display
                    // integrates deltas, so whatever accrued between reads still
                    // lands in full on the next one. A dropped read is likewise
                    // harmless — the next successful one carries the whole gap.
                    v = getSdoValue(KWH_ID);
                    if (!isnan(v)) energy_wh  = v * 1000.0f;
                    break;
            }
            fetchIndex++;
            lastSlowTime = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
