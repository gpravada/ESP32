
#include "ble_app.h"
#include "bme_app.h"
#include "led_app.h"
#include "debug_uart.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/task.h"

// Main pplication start.
void app_main(void)
{
    // LED init.
    led_init();
    xTaskCreate(&led_blink_task, "led_blink_task",  1024, NULL, 6, NULL);

    // bmp280 initialization.
#if CONFIG_IDF_TARGET_ESP32C3
	i2c_master_init();
    bme280_mutex_init();
    xTaskCreate(&bme280_reader_task, "bme280_reader_task",  2048, NULL, 6, NULL);
#endif

    // Nimble BLE intialization
    ble_init();

    //UART task
    // debug_uart_init();
    // xTaskCreate(debug_uart_rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(debug_uart_tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
