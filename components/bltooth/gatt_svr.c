/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "bme_app.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "blehr_sens.h"

#include <math.h>
#include <assert.h>

static const char *manuf_name = "EspressIf ESP32 devkitC";
static const char *model_num = "ESP32-C3";

uint16_t temperature_handle;
uint16_t pressure_handle;
uint16_t humidity_handle;

#define BUFFER_SIZE 100
static char buffer[BUFFER_SIZE];

static int gatt_svr_chr_access_heart_rate(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Heart-rate */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_ENVIRONMENT_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                /* Characteristic: Body sensor location */
                .uuid = BLE_UUID16_DECLARE(GATT_TEMPERATURE_SENSOR_UUID),
                .access_cb = gatt_svr_chr_access_heart_rate,
                .val_handle = &temperature_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Characteristic: Body sensor location */
                .uuid = BLE_UUID16_DECLARE(GATT_PRESSURE_SENSOR_UUID),
                .access_cb = gatt_svr_chr_access_heart_rate,
                .val_handle = &pressure_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Characteristic: Body sensor location */
                .uuid = BLE_UUID16_DECLARE(GATT_HUMIDITY_SENSOR_UUID),
                .access_cb = gatt_svr_chr_access_heart_rate,
                .val_handle = &humidity_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, /* No more characteristics in this service */
            },
        }
    },

    {
        /* Service: Device Information */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_INFO_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: * Manufacturer name */
                .uuid = BLE_UUID16_DECLARE(GATT_MANUFACTURER_NAME_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                /* Characteristic: Model number string */
                .uuid = BLE_UUID16_DECLARE(GATT_MODEL_NUMBER_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },

    {
        0, /* No more services */
    },
};

static int
gatt_svr_chr_access_heart_rate(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == GATT_TEMPERATURE_SENSOR_UUID)
    {
        double temperature = bme280_get_temperature();
        // Format the data into a string and store it in the buffer
        snprintf(buffer, BUFFER_SIZE, "%.2f degC", temperature);

        ESP_LOGI("GAP_SRV", "BME280: Temperature: %.2f degC\n", temperature);
        rc = os_mbuf_append(ctxt->om, buffer, strlen(buffer));

        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (uuid == GATT_PRESSURE_SENSOR_UUID)
    {
        double pressure = bme280_get_pressure();

        // Format the data into a string and store it in the buffer
        snprintf(buffer, BUFFER_SIZE, "%.2f hPa", pressure);

        rc = os_mbuf_append(ctxt->om, buffer, strlen(buffer));

        ESP_LOGI("GAP", "BME280: Pressure: %.2f hPa\n", pressure);

        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (uuid == GATT_HUMIDITY_SENSOR_UUID)
    {
        double humidity = bme280_get_humidity();
        // Format the data into a string and store it in the buffer
        snprintf(buffer, BUFFER_SIZE, "%.2f %%", humidity);

        rc = os_mbuf_append(ctxt->om, buffer, strlen(buffer));

        ESP_LOGI("GAP", "BME280: Humidity: %.2f %%\n", humidity);

        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        assert(0);
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int
gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == GATT_MODEL_NUMBER_UUID) {
        rc = os_mbuf_append(ctxt->om, model_num, strlen(model_num));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (uuid == GATT_MANUFACTURER_NAME_UUID) {
        rc = os_mbuf_append(ctxt->om, manuf_name, strlen(manuf_name));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
