idf_component_register(SRCS "bme_app.c"
                            "bme280.c"
                            "bme280_support.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)
