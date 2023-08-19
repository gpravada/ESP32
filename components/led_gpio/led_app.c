/*
****************************************************************************
* Copyright (C) 2023 - QualiZeal
*
* bme_app.c
* Date: 2016/07/04
* Revision: 2.0.5(Pressure and Temperature compensation code revision is 1.1
*               and Humidity compensation code revision is 1.0)
*
* Usage: Sensor Driver interface file for BME280 sensor
*
****************************************************************************/

#include "led_app.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_IDF_TARGET_ESP32C3
#define LED_GPIO_PIN 0
#else
#define LED_GPIO_PIN 5
#endif

void led_init()
{
    esp_rom_gpio_pad_select_gpio(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);
}

void led_blink_task(void *ignore)
{
    int led_state = 1u;

    while(true)
    {
        led_state = !led_state;
        gpio_set_level(LED_GPIO_PIN, led_state);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
