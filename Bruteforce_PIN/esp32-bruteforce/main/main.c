#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"

static const char *TAG = "cyberbrute";


#define PIN_ENC_A     16
#define PIN_ENC_B     17
#define PIN_BTN       21
#define PIN_WS2812    10
#define PIN_RUN       11
#define PIN_TRIG       4 


#define PIN_LEN        4
#define RADIX         10
#define BELIEVED_PIN  {1,2,6,4} /*Для финальной проверки*/
#define CAL_WRONG_PIN {0,0,0,0}


#define DIGIT_RESETS_EACH   1

#define BIDIR_SHORTEST      0
#define STEP_DIR           +1
#define EDGES_PER_DIGIT         4
#define MICROSTEP_US        20000
#define INTER_DIGIT_US       3000
#define SETTLE_US            5000
#define BTN_PRESS_US        40000
#define BTN_RELEASE_US      60000
#define POST_CONFIRM_US     50000

#define RESULT_WINDOW_MS     1500
#define WS_ONE_THRESH_TICKS     6

#define RESET_NONE            0
#define RESET_RUN_PIN         1
#define RESET_CANCEL_GESTURE  2
#define RESET_MODE  RESET_RUN_PIN
#define RUN_LOW_US         5000
#define BOOT_WAIT_MS        800 
#define RESET_WAIT_MS       400

#define PROGRESS_EVERY      100

#define MODE_SWEEP           0
#define MODE_SMOKE           1
#define MODE_CALIB           2
#define MODE_RANGE           3
#define RANGE_START       1260
#define RANGE_END         1264
#define MODE                 MODE_RANGE

static const uint8_t PHASE_A[4] = {1, 0, 0, 1};
static const uint8_t PHASE_B[4] = {1, 1, 0, 0};

static int g_phase = 0;
static int g_value = 0;

static rmt_channel_handle_t s_ws_chan  = NULL;
static QueueHandle_t        s_ws_queue = NULL;

static void gpio_od(int pin)
{
    gpio_set_level(pin, 0);
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
}

static void gpio_pp(int pin)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
    gpio_set_level(pin, 0);
}

static void gpio_init_all(void)
{
    gpio_od(PIN_ENC_A); gpio_od(PIN_ENC_B);
    gpio_od(PIN_BTN);
    gpio_od(PIN_RUN);
    gpio_pp(PIN_TRIG);

}

static void set_line(int pin, int level)
{
    if (level) {
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

static void emit_edges(int n, int dir)
{
    for (int i = 0; i < n; i++) {
        g_phase = (g_phase + dir + 4) & 3;
        enc_write(g_phase);
        esp_rom_delay_us(MICROSTEP_US);
    }
}

static void dial_to(int target)
{
    int fwd = ((target - g_value) % RADIX + RADIX) % RADIX;
    int numbers, dir;
    if (BIDIR_SHORTEST && (RADIX - fwd) < fwd) {
        numbers = RADIX - fwd;  dir = -STEP_DIR;
    } else {
        numbers = fwd;          dir = +STEP_DIR;
    }
    for (int k = 0; k < numbers; k++) {
        emit_edges(EDGES_PER_DIGIT, dir);
        esp_rom_delay_us(INTER_DIGIT_US);
    }
    g_value = target;
}

static void press_confirm(void)
{
    set_line(PIN_BTN, 0);
    esp_rom_delay_us(BTN_PRESS_US);
    set_line(PIN_BTN, 1);
    esp_rom_delay_us(BTN_RELEASE_US);
}

static void enter_pin(const uint8_t pin[PIN_LEN])
{
    for (int i = 0; i < PIN_LEN; i++) {
        if (DIGIT_RESETS_EACH) g_value = 0;
        dial_to(pin[i]);
        esp_rom_delay_us(SETTLE_US);
        press_confirm();
        esp_rom_delay_us(POST_CONFIRM_US);
    }
}

static void reset_state(void)
{
#if RESET_MODE == RESET_RUN_PIN
    set_line(PIN_RUN, 0);
    esp_rom_delay_us(RUN_LOW_US);
    set_line(PIN_RUN, 1);
    vTaskDelay(pdMS_TO_TICKS(BOOT_WAIT_MS));
#elif RESET_MODE == RESET_CANCEL_GESTURE
    vTaskDelay(pdMS_TO_TICKS(RESET_WAIT_MS));
#else
    vTaskDelay(pdMS_TO_TICKS(RESET_WAIT_MS));
#endif
    g_phase = 0; g_value = 0;
    enc_write(0);
}

static rmt_symbol_word_t s_ws_syms[48];

static bool IRAM_ATTR ws_rx_done(rmt_channel_handle_t ch,
                                 const rmt_rx_done_event_data_t *ed, void *ctx)
{
    (void)ch;
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)ctx, ed, &hp);
    return hp == pdTRUE;
}

static void ws2812_init(void)
{
    rmt_rx_channel_config_t cfg = {
        .gpio_num = PIN_WS2812,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,   /* 0.1 us / tick */
        .mem_block_symbols = 48,
    };
    if (rmt_new_rx_channel(&cfg, &s_ws_chan) != ESP_OK) {
        ESP_LOGE(TAG, "WS2812 RMT init failed -- success detection disabled");
        s_ws_chan = NULL;
        return;
    }
    s_ws_queue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = { .on_recv_done = ws_rx_done };
    rmt_rx_register_event_callbacks(s_ws_chan, &cbs, s_ws_queue);
    rmt_enable(s_ws_chan);
    ESP_LOGI(TAG, "WS2812 RMT decoder ready on GPIO%d", PIN_WS2812);
}

static void ws_decode(const rmt_symbol_word_t *sy, size_t n, uint8_t rgb[3])
{
    rgb[0] = rgb[1] = rgb[2] = 0;
    int nbits = (n >= 24) ? 24 : (int)n;
    for (int i = 0; i < nbits; i++) {
        uint32_t high = sy[i].level0 ? sy[i].duration0 : sy[i].duration1;
        int bit = (high >= WS_ONE_THRESH_TICKS) ? 1 : 0;
        rgb[i / 8] = (uint8_t)((rgb[i / 8] << 1) | bit);
    }
}

static int read_status(uint32_t window_ms)
{
    if (!s_ws_chan) return -1;
    rmt_receive_config_t rc = { .signal_range_min_ns = 150, .signal_range_max_ns = 100000 };
    int64_t t0 = esp_timer_get_time();
    int result = -1;
    bool armed = false;
    while ((esp_timer_get_time() - t0) < (int64_t)window_ms * 1000) {
        if (!armed) {
            if (rmt_receive(s_ws_chan, s_ws_syms, sizeof(s_ws_syms), &rc) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            armed = true;
        }
        rmt_rx_done_event_data_t ev;
        if (xQueueReceive(s_ws_queue, &ev, pdMS_TO_TICKS(100)) != pdTRUE) continue;
        armed = false;
        if (ev.num_symbols < 24) continue;
        uint8_t rgb[3];
        ws_decode(ev.received_symbols, ev.num_symbols, rgb);
        if (rgb[0] > rgb[1]) { result = 1; break; }   /* G>R -> green -> success */
        if (rgb[1] > rgb[0]) result = 0;              /* red -> keep watching */
    }
    return result;
}


static int attempt(const uint8_t pin[PIN_LEN])
{
    reset_state();
    gpio_set_level(PIN_TRIG, 1);
    enter_pin(pin);
    int col = read_status(RESULT_WINDOW_MS);
    gpio_set_level(PIN_TRIG, 0);
    return col;
}

static bool try_and_confirm(const uint8_t pin[PIN_LEN])
{
    if (attempt(pin) != 1) return false;             /* not green */
    ESP_LOGI(TAG, "candidate %d%d%d%d showed GREEN -> re-verifying",
             pin[0], pin[1], pin[2], pin[3]);
    return attempt(pin) == 1;
}

static void decode(uint32_t code, uint8_t out[PIN_LEN])
{
    for (int i = PIN_LEN - 1; i >= 0; i--) { out[i] = code % RADIX; code /= RADIX; }
}

static bool same_pin(const uint8_t a[PIN_LEN], const uint8_t b[PIN_LEN])
{
    return memcmp(a, b, PIN_LEN) == 0;
}

static void report_found(const uint8_t pin[PIN_LEN])
{
    ESP_LOGW(TAG, "==================================================");
    ESP_LOGW(TAG, "  CODE FOUND: ");
    for (int i = 0; i < PIN_LEN; i++) ESP_LOGW(TAG, "    digit[%d] = %d", i, pin[i]);
    ESP_LOGW(TAG, "==================================================");
    gpio_set_level(PIN_TRIG, 1);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

#if MODE == MODE_SMOKE
static void run_smoke(void)
{
    uint8_t guess[PIN_LEN] = BELIEVED_PIN;
    ESP_LOGW(TAG, "SMOKE TEST (hard-reset RP2040 via RUN first)");
    reset_state();
    for (int i = 0; i < PIN_LEN; i++) {
        if (DIGIT_RESETS_EACH) g_value = 0;
        ESP_LOGI(TAG, "digit[%d] -> %d", i, guess[i]);
        dial_to(guess[i]);
        vTaskDelay(pdMS_TO_TICKS(300));
        press_confirm();
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    int col = read_status(RESULT_WINDOW_MS);
    ESP_LOGW(TAG, "done -> %s",
             col == 1 ? "GREEN (correct!)" : col == 0 ? "red (wrong)" : "no WS2812 frame");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
#endif

#if MODE == MODE_CALIB
static void run_calib(void)
{
    const int SPEED_US = 20000;
    ESP_LOGW(TAG, "CALIB confirm: reset the safe so the ring reads 0, now");
    for (int s = 3; s > 0; s--) { ESP_LOGI(TAG, "start in %d...", s); vTaskDelay(pdMS_TO_TICKS(1000)); }
    g_phase = 0; enc_write(0);
    vTaskDelay(pdMS_TO_TICKS(800));
    for (int c = 1; c <= 8; c++) {
        for (int e = 0; e < 4; e++) {
            g_phase = (g_phase + STEP_DIR + 4) & 3;
            enc_write(g_phase);
            esp_rom_delay_us(SPEED_US);
        }
        ESP_LOGW(TAG, ">> cycle %d @20ms: ring should read %d -- READ now", c, c % 10);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
    ESP_LOGW(TAG, "CALIB done. Report the ring number after each cycle.");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
#endif

#if MODE == MODE_RANGE
static void run_range(void)
{
    ESP_LOGW(TAG, "RANGE demo %04d..%04d (RP2040 hard-reset via RUN before each code)",
             RANGE_START, RANGE_END);
    vTaskDelay(pdMS_TO_TICKS(500));
    for (int code = RANGE_START; code <= RANGE_END; code++) {
        uint8_t pin[PIN_LEN];
        decode((uint32_t)code, pin);
        reset_state();
        ESP_LOGW(TAG, ">> attempt %04d  (dial %d %d %d %d)", code, pin[0], pin[1], pin[2], pin[3]);
        enter_pin(pin);
        int col = read_status(RESULT_WINDOW_MS);
        ESP_LOGW(TAG, "   %04d -> %s", code,
                 col == 1 ? "GREEN (correct!)" : col == 0 ? "red (wrong)" : "no frame");
        if (col == 1) { ESP_LOGW(TAG, "=== CODE FOUND: %04d ===", code); break; }
    }
    ESP_LOGW(TAG, "RANGE demo complete.");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
#endif

void app_main(void)
{
    gpio_init_all();
    vTaskDelay(pdMS_TO_TICKS(300));
    ws2812_init();
    ESP_LOGI(TAG, "CYBERSAFE brute-forcer online (single encoder, %d digits)", PIN_LEN);

#if MODE == MODE_CALIB
    run_calib();
#elif MODE == MODE_SMOKE
    run_smoke();
#elif MODE == MODE_RANGE
    run_range();
#endif

    uint8_t guess[PIN_LEN] = BELIEVED_PIN;
    ESP_LOGI(TAG, "trying believed code first");
    if (try_and_confirm(guess)) report_found(guess);

    uint32_t total = 1;
    for (int i = 0; i < PIN_LEN; i++) total *= RADIX;

    uint8_t pin[PIN_LEN];
    for (uint32_t code = 0; code < total; code++) {
        decode(code, pin);
        if (same_pin(pin, guess)) continue;
        if (try_and_confirm(pin)) report_found(pin);
        if (code % PROGRESS_EVERY == 0)
            ESP_LOGI(TAG, "progress %u / %u", (unsigned)code, (unsigned)total);
    }

    ESP_LOGW(TAG, "search space exhausted; nothing matched — recalibrate "
                  "detection / entry timing");
}
