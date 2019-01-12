#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <u8g2.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

#include "u8g2_esp32_hal.h"
#include "bitarray.h"

// SDA - GPIO5
#define PIN_SDA 5
// SCL - GPIO4
#define PIN_SCL 4

esp_err_t
event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

static const char *TAG = "ssd1306";

void
task_test_SSD1306i2c(void *ignore)
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = PIN_SDA;
    u8g2_esp32_hal.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
                                           &u8g2,
                                           U8G2_R0,
                                           //u8x8_byte_sw_i2c,
                                           u8g2_esp32_i2c_byte_cb,
                                           u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x3C << 1);

    ESP_LOGI(TAG, "u8g2_InitDisplay");
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

    ESP_LOGI(TAG, "u8g2_SetPowerSave");
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    ESP_LOGI(TAG, "u8g2_ClearBuffer");
    u8g2_ClearBuffer(&u8g2);
    u8g2_ClearDisplay(&u8g2);
    //ESP_LOGI(TAG, "u8g2_DrawBox");
    //u8g2_DrawBox(&u8g2, 0, 46, 80, 6);
    //u8g2_DrawFrame(&u8g2, 0, 46, 100, 6);

    ESP_LOGI(TAG, "u8g2_SetFont");
    //u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_SetFont(&u8g2, u8g2_font_logisoso24_tr);
    ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2,2, 8+24, "Hello");
    u8g2_DrawStr(&u8g2,2, 2*(8+24), "world!");
    ESP_LOGI(TAG, "u8g2_SendBuffer");
    u8g2_SendBuffer(&u8g2);

    ESP_LOGI(TAG, "All done!");

    vTaskDelete(NULL);
}

#define GRID_W 128
#define GRID_H 64

uint8_t grid_ab[2][(GRID_W * GRID_H) / 8];
uint8_t *grid;
uint8_t slot;

#define P(x,y) (y * GRID_W + x)

void
grid_init(void)
{
    srand(esp_random());
    slot = 0;
    grid = grid_ab[slot];
    uint8_t x, y;
    for (y = 0; y < GRID_H; y++) {
        for (x = 0; x < GRID_W; x++) {
            bitarray_setv(grid, P(x,y), rand() & 0x01);
        }
    }
}

void
grid_advance(void)
{
    uint8_t *grid0;
    uint8_t x, y, neighs;
    uint8_t xa, xb, ya, yb;
    uint8_t cell;

    grid0 = grid_ab[slot];
    slot = !slot;
    grid = grid_ab[slot];

    for (y = 0; y < GRID_H; y++) {
        ya = (y - 1) & (GRID_H - 1);
        yb = (y + 1) & (GRID_H - 1);
        for (x = 0; x < GRID_W; x++) {
            xa = (x - 1) & (GRID_W - 1);
            xb = (x + 1) & (GRID_W - 1);
            neighs = bitarray_get(grid0, P(xa, ya)) +
                     bitarray_get(grid0, P(x , ya)) +
                     bitarray_get(grid0, P(xb, ya)) +

                     bitarray_get(grid0, P(xa, y )) +
                     bitarray_get(grid0, P(xb, y )) +

                     bitarray_get(grid0, P(xa, yb)) +
                     bitarray_get(grid0, P(x , yb)) +
                     bitarray_get(grid0, P(xb, yb));
            cell = bitarray_get(grid0, P(x, y));
            if (neighs == 3) {
                bitarray_set(grid, P(x, y));
            } else if (neighs == 2) {
                bitarray_setv(grid, P(x, y), cell);
            } else {
                bitarray_clear(grid, P(x, y));
            }
        }
    }
}

void
task_gol_SSD1306i2c(void *ignore)
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = PIN_SDA;
    u8g2_esp32_hal.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
                                           &u8g2,
                                           U8G2_R0,
                                           u8g2_esp32_i2c_byte_cb,
                                           u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x3C << 1);

    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    u8g2_ClearBuffer(&u8g2);

    u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tr);

    // Intro
    int i;
    for (i = 128; i >= 26; i--) {
        u8g2_ClearBuffer(&u8g2);
        u8g2_DrawStr(&u8g2, 2, 4+16 + (i - 16), "Game of Life");
        u8g2_DrawStr(&u8g2, 26, 2*(4+16) + (i - 16), "by Dhole");
        u8g2_SendBuffer(&u8g2);

        vTaskDelay(16 / portTICK_PERIOD_MS);
    }
    vTaskDelay(600 / portTICK_PERIOD_MS);

    grid_init();
    uint8_t x, y;
    while (1) {
        u8g2_ClearBuffer(&u8g2);
        for (y = 0; y < GRID_H; y++) {
            for (x = 0; x < GRID_W; x++) {
                if (bitarray_get(grid_ab[slot], P(x,y))) {
                    u8g2_DrawPixel(&u8g2, x, y);
                }
            }
        }
        u8g2_SendBuffer(&u8g2);
        grid_advance();
        //vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void
app_main(void)
{
    nvs_flash_init();
    //xTaskCreate(&task_test_SSD1306i2c, "task_test_SSD1306i2c", 2048, NULL, 5, NULL);
    xTaskCreate(&task_gol_SSD1306i2c, "task_gol_SSD1306i2c", 2048, NULL, 5, NULL);
}

/*
void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "access_point_name",
            .password = "password",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_4, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
*/

