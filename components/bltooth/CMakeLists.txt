idf_component_register(SRCS "ble_app.c"
                            "gatt_svr.c"
                       INCLUDE_DIRS "include" "../bme280/include"
                       REQUIRES driver bt nvs_flash)
