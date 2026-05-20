#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_sensor.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "i2s_example_pins.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "esp_heap_caps.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include <sys/stat.h>
#include <errno.h>
#include "esp_sleep.h"
static const char *TAG = "NOTEBOOK_V1";
#define WS2812_GPIO 48

#define T0H 4
#define T0L 8
#define T1H 8
#define T1L 4

#define AUDIO_SAMPLE_RATE 16000
#define AUDIO_BUFFER_SAMPLES 256
#define PLAYBACK_GAIN 2
#define RECORD_FILE_PATH "/sdcard/record.raw"
#define SD_MOUNT_PATH "/sdcard"
#define RECORD_FILE_NAME_FORMAT "/sdcard/record_%d.raw"

static void get_record_file_path(int touch_pad, char *out_path, size_t out_size);
static void play_beep(void);

static void get_record_file_path(int touch_pad, char *out_path, size_t out_size)
{
    int pad_num = 0;
    switch (touch_pad)
    {
    case TOUCH_PAD_NUM2:
        pad_num = 2;
        break;
    case TOUCH_PAD_NUM3:
        pad_num = 3;
        break;
    case TOUCH_PAD_NUM4:
        pad_num = 4;
        break;
    case TOUCH_PAD_NUM5:
        pad_num = 5;
        break;
    case TOUCH_PAD_NUM6:
        pad_num = 6;
        break;
    case TOUCH_PAD_NUM7:
        pad_num = 7;
        break;
    case TOUCH_PAD_NUM8:
        pad_num = 8;
        break;
    default:
        pad_num = 0;
        break;
    }
    snprintf(out_path, out_size, RECORD_FILE_NAME_FORMAT, pad_num);
}

// touch_pressed is used for the WS2812 LED state.
volatile bool touch_pressed, touch_pressed2 = false;
// touch_record_pressed is true when pad 1 is touched.
volatile bool touch_record_pressed, touch_record_pressed2 = false;
// touch_play_pressed is true when any play pad (2..8) is touched.
volatile bool touch_play_pressed, touch_play_pressed2 = false;
// touch_play_pad stores the first detected play pad number.
volatile int touch_play_pad, touch_play_pad2 = -1;

#define TOUCH_PAD_RECORD TOUCH_PAD_NUM1
static const touch_pad_t TOUCH_PAD_PLAY_LIST[] = {
    TOUCH_PAD_NUM2,
    TOUCH_PAD_NUM3,
    TOUCH_PAD_NUM4,
    TOUCH_PAD_NUM5,
    TOUCH_PAD_NUM6,
    TOUCH_PAD_NUM7,
    TOUCH_PAD_NUM8,
};
#define PLAY_TOUCH_COUNT (sizeof(TOUCH_PAD_PLAY_LIST) / sizeof(TOUCH_PAD_PLAY_LIST[0]))

#define PIN_NUM_MISO 15 // DAT0
#define PIN_NUM_MOSI 13 // CMD
#define PIN_NUM_CLK 14  // CLK
#define PIN_NUM_CS 12   // DAT3

static i2s_chan_handle_t tx_chan;
static i2s_chan_handle_t rx_chan;
static volatile bool playback_in_progress = false;
static sdmmc_card_t *card;

static void play_beep(void)
{
    if (playback_in_progress)
    {
        return;
    }
    playback_in_progress = true;

    esp_err_t ret = i2s_channel_enable(tx_chan);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Beep enable failed: %s", esp_err_to_name(ret));
        playback_in_progress = false;
        return;
    }

    int16_t tone_buffer[AUDIO_BUFFER_SAMPLES];
    const int tone_freq = 800;
    const int samples_per_cycle = AUDIO_SAMPLE_RATE / tone_freq;
    const int amplitude = 12000;

    for (int i = 0; i < AUDIO_BUFFER_SAMPLES; i++)
    {
        tone_buffer[i] = (i % samples_per_cycle < samples_per_cycle / 2) ? amplitude : -amplitude;
    }

    size_t bytes_written = 0;
    for (int chunk = 0; chunk < 4; chunk++)
    {
        ret = i2s_channel_write(tx_chan,
                                tone_buffer,
                                sizeof(tone_buffer),
                                &bytes_written,
                                pdMS_TO_TICKS(1000));
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Beep write failed: %s", esp_err_to_name(ret));
            break;
        }
    }

    esp_err_t disable_ret = i2s_channel_disable(tx_chan);
    if (disable_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Beep disable returned %s", esp_err_to_name(disable_ret));
    }
    playback_in_progress = false;
}

// Initialize the SD card over SD-SPI and mount it at /sdcard.
esp_err_t sd_card_init(void)
{
    esp_err_t ret;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 4000;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    // IMPORTANT FIX: check if already initialized
    static bool spi_inited = false;

    if (!spi_inited)
    {
        ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG, "SPI init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        spi_inited = true;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config,
                                  &mount_config, &card);

    ESP_LOGE(TAG, "SD mount result: %s", esp_err_to_name(ret));
    return ret;
}
// Monitor touch sensor values for pad 1 (record) and pads 2..8 (play).
// This task computes a baseline threshold once at startup, then polls every 20ms.
static void touch_task(void *pvParameter)
{
    uint32_t raw_value;
    uint32_t record_threshold = 0;
    static uint32_t play_thresholds[PLAY_TOUCH_COUNT] = {0};

    // Allow the touch pad hardware to settle before sampling baseline values.
    vTaskDelay(pdMS_TO_TICKS(100));
    touch_pad_read_raw_data(TOUCH_PAD_RECORD, &record_threshold);

    record_threshold *= 2;

    for (size_t i = 0; i < PLAY_TOUCH_COUNT; i++)
    {
        touch_pad_read_raw_data(TOUCH_PAD_PLAY_LIST[i], &play_thresholds[i]);
        play_thresholds[i] *= 1.2;
    }

    ESP_LOGI(TAG, "Touch init: record threshold=%u", (unsigned)record_threshold);
    for (size_t i = 0; i < PLAY_TOUCH_COUNT; i++)
    {
        ESP_LOGI(TAG, "Touch init: play pad %d threshold=%u",
                 TOUCH_PAD_PLAY_LIST[i] + 1,
                 (unsigned)play_thresholds[i]);
    }

    while (1)
    {
        touch_pad_read_raw_data(TOUCH_PAD_RECORD, &raw_value);
        touch_record_pressed = raw_value > record_threshold;
        touch_pressed = touch_record_pressed;

        touch_play_pressed = false;
        touch_play_pad = -1;

        for (size_t i = 0; i < PLAY_TOUCH_COUNT; i++)
        {
            touch_pad_read_raw_data(TOUCH_PAD_PLAY_LIST[i], &raw_value);
            if (raw_value > play_thresholds[i])
            {
                // ESP_LOGI(TAG, "touch %u pressed",i);
                touch_play_pressed = true;
                if (touch_play_pad < 0)
                {
                    touch_play_pad = TOUCH_PAD_PLAY_LIST[i];
                }
                touch_pressed = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Record raw microphone data to SD card while TOUCH_PAD_NUM1 is pressed.
static void record_task(void *args)
{
    ESP_LOGI(TAG, "record_task started");

    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_33,
            .ws = GPIO_NUM_34,
            .din = GPIO_NUM_35,
            .dout = I2S_GPIO_UNUSED,
        },
    };

    ESP_LOGI(TAG, "Init RX");
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_LOGI(TAG, "RX enabled");

    int16_t pcm_buffer[AUDIO_BUFFER_SAMPLES];
    int32_t raw_sample;
    size_t bytes_read = 0;

    while (1)
    {
        ESP_LOGI(TAG, "Waiting for record touch on IO1...");
        while (!touch_record_pressed)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (touch_play_pressed && touch_play_pad >= 0)
        {
            if (playback_in_progress)
            {
                ESP_LOGW(TAG, "Playback in progress; deferring recording");
                while (playback_in_progress)
                {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }

            char record_path[64];
            get_record_file_path(touch_play_pad, record_path, sizeof(record_path));
            ESP_LOGI(TAG, "Record touch combo detected -> saving to %s", record_path);
            ESP_LOGI(TAG, "Free heap before recording: %u", (unsigned)esp_get_free_heap_size());
            ESP_LOGI(TAG, "Task %s stack high water mark: %u", pcTaskGetName(NULL), (unsigned)uxTaskGetStackHighWaterMark(NULL));

            FILE *f = fopen(record_path, "wb");
            if (!f)
            {
                ESP_LOGE(TAG, "Failed to open %s for write: errno=%d (%s)", record_path, errno, strerror(errno));
                struct stat sd_stat;
                int sd_stat_res = stat(SD_MOUNT_PATH, &sd_stat);
                ESP_LOGE(TAG, "stat(%s)=%d errno=%d (%s)", SD_MOUNT_PATH, sd_stat_res, errno, strerror(errno));
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            uint64_t recorded_samples = 0;
            int active_pad = touch_play_pad;

            int release_count = 0;
            const int RELEASE_MS = 100;
            const int LOOP_MS = 20;

            while (1)
            {

                if (touch_record_pressed && touch_play_pressed && touch_play_pad == active_pad)
                {
                    release_count = 0;
                    int count = 0;
                    while (count < AUDIO_BUFFER_SAMPLES && touch_record_pressed && touch_play_pressed && touch_play_pad == active_pad)
                    {
                        esp_err_t ret = i2s_channel_read(
                            rx_chan,
                            &raw_sample,
                            sizeof(raw_sample),
                            &bytes_read,
                            pdMS_TO_TICKS(500));

                        if (ret != ESP_OK)
                        {
                            ESP_LOGE(TAG, "i2s read failed: %s", esp_err_to_name(ret));
                            continue;
                        }

                        int32_t shifted = raw_sample >> 15;
                        if (raw_sample == 0 || raw_sample == -1)
                        {
                            continue;
                        }

                        if (shifted > INT16_MAX)
                        {
                            shifted = INT16_MAX;
                        }
                        else if (shifted < INT16_MIN)
                        {
                            shifted = INT16_MIN;
                        }

                        pcm_buffer[count++] = (int16_t)shifted;
                    }

                    if (count > 0)
                    {
                        size_t written = fwrite(pcm_buffer, sizeof(int16_t), count, f);
                        if (written != (size_t)count)
                        {
                            ESP_LOGE(TAG, "fwrite failed wrote=%zu expected=%d", written, count);
                            break;
                        }
                        recorded_samples += written;
                        ESP_LOGI(TAG, "Recorded chunk: %d samples total=%llu",
                                 (int)written, (unsigned long long)recorded_samples);
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                    touch_play_pressed2 = touch_play_pressed;

                    touch_record_pressed2 = touch_record_pressed;
                    touch_play_pad2 = touch_play_pad;
                }
                else
                {
                    release_count += LOOP_MS;
                    if (release_count >= RELEASE_MS)
                        break; // really released
                    vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
                    continue;
                }
            }

            fflush(f);
            fclose(f);

            ESP_LOGI(TAG, "Recording stopped for pad %d, because touch_play_pressed:%d , touch_record_pressed:%d, touch_play_pad:%d", active_pad - TOUCH_PAD_NUM0, touch_play_pressed2, touch_record_pressed2, touch_play_pad2);
            ESP_LOGI(TAG, "Recording stopped for pad %d, total samples=%llu",
                     active_pad - TOUCH_PAD_NUM0,
                     (unsigned long long)recorded_samples);

            while (touch_record_pressed && (!touch_play_pressed || touch_play_pad != active_pad))
            {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        else
        {
            ESP_LOGI(TAG, "Pad 1 pressed alone, playing beep");
            play_beep();
            while (touch_record_pressed && !touch_play_pressed)
            {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }
}

// Wait for a rising edge on any play pad (2..8) and stream the saved SD recording.
static void playback_task(void *args)
{
    bool prev_play_pressed = false;
    int16_t pcm_buffer[AUDIO_BUFFER_SAMPLES];
    size_t bytes_written = 0;

    while (1)
    {
        bool current_play = touch_play_pressed;

        if (!playback_in_progress && current_play && !prev_play_pressed)
        {
            if (touch_record_pressed)
            {
                ESP_LOGW(TAG, "Playback requested while recording; ignoring until record stops");
            }
            else if (touch_play_pad >= 0)
            {
                playback_in_progress = true;
                ESP_LOGI(TAG, "Playback rising edge detected on pad %d", touch_play_pad - TOUCH_PAD_NUM0);

                char record_path[64];
                get_record_file_path(touch_play_pad, record_path, sizeof(record_path));
                FILE *pf = fopen(record_path, "rb");
                if (!pf)
                {
                    ESP_LOGE(TAG, "No recording found for pad %d at %s: errno=%d (%s)",
                             touch_play_pad - TOUCH_PAD_NUM0,
                             record_path,
                             errno,
                             strerror(errno));
                    playback_in_progress = false;
                    prev_play_pressed = current_play;
                    vTaskDelay(pdMS_TO_TICKS(20));
                    continue;
                }

                ESP_LOGI(TAG, "Starting playback of %s", record_path);
                esp_err_t tx_ret = i2s_channel_enable(tx_chan);
                if (tx_ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "i2s_channel_enable tx failed: %s", esp_err_to_name(tx_ret));
                    fclose(pf);
                    playback_in_progress = false;
                    prev_play_pressed = current_play;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                while (!feof(pf))
                {
                    size_t samples_read = fread(pcm_buffer, sizeof(int16_t), AUDIO_BUFFER_SAMPLES, pf);
                    if (samples_read == 0)
                    {
                        break;
                    }

                    for (size_t i = 0; i < samples_read; i++)
                    {
                        int32_t scaled = (int32_t)pcm_buffer[i] * PLAYBACK_GAIN;
                        if (scaled > INT16_MAX)
                        {
                            scaled = INT16_MAX;
                        }
                        else if (scaled < INT16_MIN)
                        {
                            scaled = INT16_MIN;
                        }
                        pcm_buffer[i] = (int16_t)scaled;
                    }

                    esp_err_t ret = i2s_channel_write(
                        tx_chan,
                        pcm_buffer,
                        samples_read * sizeof(int16_t),
                        &bytes_written,
                        pdMS_TO_TICKS(1000));

                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "i2s write failed: %s", esp_err_to_name(ret));
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }

                fclose(pf);
                esp_err_t disable_ret = i2s_channel_disable(tx_chan);
                if (disable_ret != ESP_OK)
                {
                    ESP_LOGW(TAG, "i2s_channel_disable returned %s", esp_err_to_name(disable_ret));
                }

                ESP_LOGI(TAG, "Playback finished");
                playback_in_progress = false;
            }
        }

        prev_play_pressed = current_play;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// WS2812 task: cycle LED color while any touch is active, turn off when released.
void ws2812_task(void *pvParameters)
{
    rmt_channel_handle_t tx_chan = NULL;
    rmt_tx_channel_config_t tx_config_cfg = {
        .gpio_num = WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config_cfg, &tx_chan));
    ESP_ERROR_CHECK(rmt_enable(tx_chan));

    rmt_copy_encoder_config_t copy_encoder_config = {};
    rmt_encoder_handle_t copy_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &copy_encoder));

    rmt_symbol_word_t symbols[24];
    uint8_t colors[3][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};
    int current_color_idx = 0;
    bool was_pressed = false;
    uint32_t last_color_change = 0;

    while (1)
    {
        if (touch_pressed)
        {
            uint32_t now = xTaskGetTickCount();
            if (now - last_color_change > pdMS_TO_TICKS(300))
            {
                current_color_idx = (current_color_idx + 1) % 3;
                last_color_change = now;
            }

            uint8_t r = colors[current_color_idx][0];
            uint8_t g = colors[current_color_idx][1];
            uint8_t b = colors[current_color_idx][2];
            uint8_t grb[3] = {g, r, b};

            int idx = 0;
            for (int byte = 0; byte < 3; byte++)
            {
                for (int bit = 7; bit >= 0; bit--)
                {
                    if (grb[byte] & (1 << bit))
                    {
                        symbols[idx] = (rmt_symbol_word_t){.level0 = 1, .duration0 = T1H, .level1 = 0, .duration1 = T1L};
                    }
                    else
                    {
                        symbols[idx] = (rmt_symbol_word_t){.level0 = 1, .duration0 = T0H, .level1 = 0, .duration1 = T0L};
                    }
                    idx++;
                }
            }

            rmt_transmit_config_t tx_config = {.loop_count = 0};
            rmt_transmit(tx_chan, copy_encoder, symbols, sizeof(symbols), &tx_config);
            rmt_tx_wait_all_done(tx_chan, portMAX_DELAY);

            was_pressed = true;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        else if (was_pressed)
        {
            for (int i = 0; i < 24; i++)
            {
                symbols[i] = (rmt_symbol_word_t){.level0 = 1, .duration0 = T0H, .level1 = 0, .duration1 = T0L};
            }
            rmt_transmit_config_t tx_config = {.loop_count = 0};
            rmt_transmit(tx_chan, copy_encoder, symbols, sizeof(symbols), &tx_config);
            rmt_tx_wait_all_done(tx_chan, portMAX_DELAY);
            was_pressed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void i2s_init_tx(void)
{
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_38,
            .ws = GPIO_NUM_36,
            .dout = GPIO_NUM_37,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
}


void deep_sleep_test(void) {
    // Set wake-up timer (10 seconds)
    esp_sleep_enable_timer_wakeup(10 * 1000000);  // microseconds

    printf("Entering deep sleep in 1 second...\n");
    vTaskDelay(pdMS_TO_TICKS(10000));
    esp_deep_sleep_start();  // Will not return
}
void app_main(void)
{
    ESP_LOGI(TAG, "Reset reason: %d", esp_reset_reason());
    // xTaskCreate(deep_sleep_test, "test", 4096, NULL, 5, NULL);

    touch_pad_init();
    touch_pad_config(TOUCH_PAD_RECORD);
    // Enable hardware filter
    touch_filter_config_t filter_cfg = {
        .mode = TOUCH_PAD_FILTER_IIR_16,
        .debounce_cnt = 1,
        .noise_thr = 0,
        .jitter_step = 4,
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
    };
    touch_pad_filter_set_config(&filter_cfg);
    touch_pad_filter_enable();
    for (size_t i = 0; i < PLAY_TOUCH_COUNT; i++)
    {
        touch_pad_config(TOUCH_PAD_PLAY_LIST[i]);
    }

    i2s_init_tx();

    touch_pad_denoise_t denoise = {
        .grade = TOUCH_PAD_DENOISE_BIT4,
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    touch_pad_denoise_set_config(&denoise);
    touch_pad_denoise_enable();
    ESP_LOGI(TAG, "Denoise function init");

    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();

    esp_err_t err = sd_card_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "SD mount FAILED: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "SD mount OK");
    }

    xTaskCreate(touch_task, "touch", 4096, NULL, 5, NULL);
    if (err == ESP_OK)
    {
        xTaskCreate(record_task, "record", 8192, NULL, 5, NULL);
        xTaskCreate(playback_task, "playback", 8192, NULL, 6, NULL);
    }
    else
    {
        ESP_LOGW(TAG, "SD record/playback tasks disabled because SD mount failed");
    }
    xTaskCreate(ws2812_task, "ws2812", 4096, NULL, 5, NULL);
}
