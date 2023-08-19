/*! \file bmeapp.h
    \brief BME280 Sensor interface support header file */
#ifndef __BMEAPP_H__
#define __BMEAPP_H__

typedef struct
{
	double humidity;
	double pressure;
	double temperature;
}bme_sensor_value_t;

void bme280_mutex_init();
void i2c_master_init();
void bme280_reader_task(void *ignore);
void bme280_get_values(bme_sensor_value_t * data);
double bme280_get_humidity();
double bme280_get_pressure();
double bme280_get_temperature();

#endif