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

#include "bme_app.h"
#include "bme280.h"

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"

#if CONFIG_IDF_TARGET_ESP32C3
#define SDA_PIN GPIO_NUM_8
#define SCL_PIN GPIO_NUM_9
#else
#define SDA_PIN GPIO_NUM_22
#define SCL_PIN GPIO_NUM_23
#endif

#define TAG_BME280 "BME280"

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

SemaphoreHandle_t xMutex;

static bme_sensor_value_t bme280_data;

static s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static void BME280_delay_msek(u32 msek);

void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void bme280_mutex_init()
{
	xMutex = xSemaphoreCreateMutex();
}

void bme280_reader_task(void *ignore)
{
	struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write,
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS1,
		.delay_msec = BME280_delay_msek
	};

	s32 com_rslt;
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

	com_rslt = bme280_init(&bme280);

	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	if (com_rslt == SUCCESS) {
		while(true) {
			vTaskDelay(1000/portTICK_PERIOD_MS);

			if (xSemaphoreTake(xMutex, portMAX_DELAY))
    		{
				com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
					&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

				if (com_rslt == SUCCESS) {
					bme280_data.temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
					bme280_data.pressure    = bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100;
					bme280_data.humidity    = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
				} else {
					ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
				}
				xSemaphoreGive(xMutex);
			}
		}
	} else {
		ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
	}

	vTaskDelete(NULL);
}

void bme280_get_values(bme_sensor_value_t * bme_sensor_data)
{
	if (xSemaphoreTake(xMutex, portMAX_DELAY))
	{
		bme_sensor_data->humidity 	  = bme280_data.humidity;
		bme_sensor_data->pressure 	  = bme280_data.pressure;
		bme_sensor_data->temperature  = bme280_data.temperature;
		xSemaphoreGive(xMutex);
	}
}

double bme280_get_humidity()
{
	double humidity = 0.0;
	if (xSemaphoreTake(xMutex, portMAX_DELAY))
	{
		humidity = bme280_data.humidity;
		xSemaphoreGive(xMutex);
	}
	return humidity;
}

double bme280_get_pressure()
{
	double pressure = 0.0;
	if (xSemaphoreTake(xMutex, portMAX_DELAY))
	{
		pressure = bme280_data.pressure;
		xSemaphoreGive(xMutex);
	}
	return pressure;
}

double bme280_get_temperature()
{
	double temperature = 0.0;
	if (xSemaphoreTake(xMutex, portMAX_DELAY))
	{
		temperature = bme280_data.temperature;
		xSemaphoreGive(xMutex);
	}
	return temperature;
}

static s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = ESP_FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

static s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = ESP_FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

static void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}
