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
#include "freertos/queue.h"
#include "freertos/task.h"

#if CONFIG_IDF_TARGET_ESP32C3
#define LED_GPIO_PIN 0
#else
#define LED_GPIO_PIN 5
#endif

#define INPUT_PIN 7

int state = 0;
QueueHandle_t interputQueue;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

void led_init()
{
    esp_rom_gpio_pad_select_gpio(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INPUT_PIN);
    gpio_pullup_dis(INPUT_PIN);
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_ANYEDGE);

    interputQueue = xQueueCreate(10, sizeof(int));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);
}

void led_blink_task(void *ignore)
{
    int led_state = 1u;
    int pinNumber;

    while(true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            led_state = !led_state;
            gpio_set_level(LED_GPIO_PIN, led_state);
            // vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    }
}
