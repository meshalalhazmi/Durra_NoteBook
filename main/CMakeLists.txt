idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_driver_i2s esp_driver_gpio driver spiffs fatfs sdmmc
                    INCLUDE_DIRS "." "$ENV{IDF_PATH}/examples/peripherals/i2s/common")

