#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "driver/dac_oneshot.h"
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


// =============================================================================
// ESP-NOW — Qualia receiver MAC
// =============================================================================
static uint8_t receiverMAC[] = {0x20, 0x6E, 0xF1, 0xB8, 0xE3, 0x2C};

// Packet layout must match CircuitPython: struct.unpack('<fffffffB', msg)
typedef struct __attribute__((packed)) {
    float speed_mph;
    float temp_bat_c;
    float temp_mot_c;
    float temp_inv_c;
    float temp_isa_c;
    float soc_pct;
    float udc_v;
    uint8_t ready;   // set to 0xA5 when valid; must stay last
} TelemetryPacket;

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
#define DS18B20_INTERVAL_MS 10000

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

// --- Current telemetry values ---
static float speed_mph  = 0.0f;
static float temp_bat_c = 0.0f;
static float temp_mot_c = 0.0f;
static float temp_inv_c = 0.0f;
static float temp_isa_c = 0.0f;
static float soc_pct    = 100.0f;
static float udc_v      = 0.0f;

// --- Timing ---
static uint64_t lastSendTime = 0;
#define SEND_INTERVAL_MS  200 //old value 200

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

// =============================================================================
// CANopen SDO helpers (mirrors CanSDO::requestSdoElement / CanSDO::GetValue)
// =============================================================================

static void requestSdo(int id) {
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
    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

static float getSdoValue(int id) {
    uint16_t index  = SDO_INDEX_BASE | (uint16_t)(id >> 8);
    uint8_t  subIdx = (uint8_t)(id & 0xFF);

    requestSdo(id);

    // pdMS_TO_TICKS(5) = 0 ticks at CONFIG_FREERTOS_HZ=100 (10 ms/tick), so the
    // original twai_receive was non-blocking and the loop busy-waited for 50 ms.
    // Use 10 ms (= 1 tick) so each call blocks and yields when no message is
    // waiting, and taskYIELD() ensures IDLE0 can reset the watchdog even when
    // messages arrive continuously.
    uint64_t deadline = millis() + 50;
    while (millis() < deadline) {
        twai_message_t rx = {};
        if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK) {
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
        taskYIELD();
    }

    ESP_LOGW(TAG, "SDO timeout for param %d", id);
    return NAN;
}

// =============================================================================
// I2C slave — serves TelemetryPacket to the speedometer master (09_speedometer)
// SCL = GPIO 22, SDA = GPIO 21, slave address 0x42
//
// The speedometer calls i2c_master_receive() every 100 ms.  We keep the slave
// TX FIFO populated by writing a fresh packet on every 200 ms telemetry tick.
// The FIFO holds 2 full packets so the master always has data to read even if
// our write and its poll are briefly out of phase.
// =============================================================================
#define I2C_SCL_PIN    GPIO_NUM_22
#define I2C_SDA_PIN    GPIO_NUM_21

//#define I2C_SCL_PIN    GPIO_NUM_12
//#define I2C_SDA_PIN    GPIO_NUM_14

#define I2C_SLAVE_ADDR 0x42

static i2c_slave_dev_handle_t i2c_slave = NULL;

static void sendI2c(void) {
    if (i2c_slave == NULL) return;
    esp_err_t err = i2c_slave_transmit(i2c_slave,
                                       (const uint8_t *)&packet, sizeof(packet),
                                       0);
    // ESP_ERR_INVALID_STATE = buffer full, previous packet not yet read — skip silently
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "I2C slave write failed: %s", esp_err_to_name(err));
}

// =============================================================================
// ESP-NOW
// =============================================================================

static __attribute__((unused)) void sendPacket(void) {
    packet.speed_mph  = speed_mph;
    packet.temp_bat_c = temp_bat_c;
    packet.temp_mot_c = temp_mot_c;
    packet.temp_inv_c = temp_inv_c;
    packet.temp_isa_c = temp_isa_c;
    packet.soc_pct    = soc_pct;
    packet.udc_v      = udc_v;
    packet.ready      = 0xA5;
    esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
    sendI2c();
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
    // Release and sample presence pulse within a critical section for accurate timing
    portENTER_CRITICAL(&ow_mux);
    ow_hi();
    esp_rom_delay_us(70);
    bool present = (ow_rd() == 0);
    portEXIT_CRITICAL(&ow_mux);
    esp_rom_delay_us(410);
    if (ow_rd() == 0) return false;  // bus stuck low — shorted or no pull-up
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

// Print every DS18B20 address found on the bus in ds18b20_addr[][] format.
static void ds18b20_enumerate(void) {
    ESP_LOGI(TAG, "--- DS18B20 SEARCH on GPIO %d ---", DS18B20_PIN);
    ow_search_t s;
    ow_search_init(&s);
    int n = 0;
    while (ow_search_next(&s)) {
        if (++n > 16) { ESP_LOGE(TAG, "search loop limit — bus stuck or shorted"); break; }
        ESP_LOGI(TAG, " ds18b20_addr [%d] { 0x%02X, 0x%02X, 0x%02X, 0x%02X,"
                      " 0x%02X, 0x%02X, 0x%02X, 0x%02X }",
                 n,
                 s.rom[0], s.rom[1], s.rom[2], s.rom[3],
                 s.rom[4], s.rom[5], s.rom[6], s.rom[7]);
    }
    if (n == 0)
        ESP_LOGW(TAG, "  no devices found — check wiring on GPIO %d", DS18B20_PIN);
    else
        ESP_LOGI(TAG, "  %d device(s) found", n);
    ESP_LOGI(TAG, "---------------------------------");
}

// =============================================================================
// DS18B20 task — reads all sensors every DS18B20_INTERVAL_MS, pinned to CPU 1
// so its 800 ms conversion wait never blocks the main CAN/ESP-NOW loop on CPU 0.
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
        // Trigger conversion on ALL sensors simultaneously (SKIP ROM + CONVERT T)
        if (!ow_reset()) {
            ESP_LOGW(TAG, "DS18B20: no presence pulse on GPIO %d — check wiring", DS18B20_PIN);
            vTaskDelay(pdMS_TO_TICKS(DS18B20_INTERVAL_MS));
            continue;
        }
        ow_write_byte(OW_SKIP_ROM);
        ow_write_byte(OW_CONVERT_T);
        vTaskDelay(pdMS_TO_TICKS(800));  // 12-bit conversion ≤750 ms

        // Read each sensor individually (MATCH ROM + READ SCRATCHPAD)
        float sum = 0.0f, mn = 200.0f, mx = -200.0f;
        int valid = 0;
        for (int i = 0; i < DS18B20_COUNT; i++) {
            if (!ow_reset()) continue;
            ow_write_byte(OW_MATCH_ROM);
            for (int b = 0; b < 8; b++) ow_write_byte(ds18b20_addr[i][b]);
            ow_write_byte(OW_READ_PAD);
            uint8_t sp[9];
            for (int b = 0; b < 9; b++) sp[b] = ow_read_byte();
            if (sp[0] == 0xFF && sp[1] == 0xFF) continue;  // sensor absent
            int16_t raw = (int16_t)((sp[1] << 8) | sp[0]);
            float tc = raw / 16.0f;
            ESP_LOGI(TAG, "DS18B20[%d]: %.2f C", i, tc);
            sum += tc;
            if (tc < mn) mn = tc;
            if (tc > mx) mx = tc;
            valid++;
        }

        if (valid > 0) {
            float ave = sum / valid;
            bat_ave_c = ave;
            bat_min_c = mn;
            bat_max_c = mx;

            // Map average temp (−20…100 °C) to DAC byte (0…255) on GPIO 25
            uint8_t dac_val = (uint8_t)(constrain_f((ave + 20.0f) / 120.0f * 255.0f, 0.0f, 255.0f));
            dac_oneshot_output_voltage(dac_handle, dac_val);
            ESP_LOGI(TAG, "DS18B20 ave:%.1f min:%.1f max:%.1f DAC:%u", ave, mn, mx, dac_val);

            // Broadcast battery temps on CAN bus (0x4F1, 3 signed bytes: ave, min, max)
            twai_message_t msg = {};
            msg.identifier       = 0x4F1;
            msg.data_length_code = 3;
            msg.data[0] = (uint8_t)(int8_t)constrain_f(ave, -128.0f, 127.0f);
            msg.data[1] = (uint8_t)(int8_t)constrain_f(mn,  -128.0f, 127.0f);
            msg.data[2] = (uint8_t)(int8_t)constrain_f(mx,  -128.0f, 127.0f);
            esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
            if (err != ESP_OK)
                ESP_LOGW(TAG, "DS18B20 CAN TX failed: %s", esp_err_to_name(err));
        }

        // Sleep the remainder of the interval
        vTaskDelay(pdMS_TO_TICKS(DS18B20_INTERVAL_MS));
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
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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

    // --- I2C slave init (SCL GPIO 22, SDA GPIO 21, addr 0x42) ---
    i2c_slave_config_t i2c_slave_cfg = {};
    i2c_slave_cfg.i2c_port       = I2C_NUM_0;
    i2c_slave_cfg.scl_io_num     = I2C_SCL_PIN;
    i2c_slave_cfg.sda_io_num     = I2C_SDA_PIN;
    i2c_slave_cfg.clk_source     = I2C_CLK_SRC_DEFAULT;
    i2c_slave_cfg.slave_addr     = I2C_SLAVE_ADDR;
    i2c_slave_cfg.addr_bit_len   = I2C_ADDR_BIT_LEN_7;
    i2c_slave_cfg.send_buf_depth = sizeof(packet);  // 1 packet: full = not yet read
    if (i2c_new_slave_device(&i2c_slave_cfg, &i2c_slave) == ESP_OK) {
        ESP_LOGI(TAG, "I2C slave ready (SCL GPIO %d, SDA GPIO %d, addr 0x%02X)",
                 I2C_SCL_PIN, I2C_SDA_PIN, I2C_SLAVE_ADDR);
    } else {
        ESP_LOGE(TAG, "I2C slave init failed");
    }

    // --- DS18B20 task (CPU 1, 4 kB stack, reads sensors every 10 s) ---
    xTaskCreatePinnedToCore(ds18b20_task, "ds18b20", 4096, NULL, 2, NULL, 1);

    lastSendTime = millis();
    ESP_LOGI(TAG, "Ready - reading CAN, forwarding via ESP-NOW");

    // =============================================================================
    // Main loop — fetch one CAN parameter per iteration (round-robin),
    // send every SEND_INTERVAL_MS ms.
    // RPM is fetched every tick for responsiveness; the remaining 6 parameters
    // rotate one-per-tick 
    // Each getSdoValue() blocks up to 50 ms → two reads = up to 100 ms per tick.
    // =============================================================================
    uint8_t fetchIndex = 0;
    while (true) {
        if (espNowNeedsReinit)
            reinitEspNow();

        if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
            float v;

            v = getSdoValue(MOTOR_RPM_PARAM_ID);
            if (!isnan(v)) speed_mph = constrain_f(v * 0.00981f, 0.0f, 100.0f);

            switch (fetchIndex % 6) {
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
                    v = getSdoValue(UDC_IDX);
                    if (!isnan(v)) udc_v      = v;
                    break;
            }
            fetchIndex++;

            // DS18B20 gives direct cell readings every 10 s; prefer those over
            // the ZombieVerter's SDO BMS_T_MAX when fresh readings are available.
            if (!isnan(bat_max_c)) temp_bat_c = bat_max_c;

            lastSendTime = millis();
            sendPacket();
            ESP_LOGI(TAG, "spd:%.1f soc:%.1f bat:%.1f mot:%.1f inv:%.1f isa:%.1f udc:%.1f",
                     speed_mph, soc_pct, temp_bat_c, temp_mot_c, temp_inv_c, temp_isa_c, udc_v);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
