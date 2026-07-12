set(srcs "main.c"
         "esp_hid_gap.c")

set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES esp_hid bt esp_driver_spi esp_driver_gpio mpu6050
                    PRIV_REQUIRES nvs_flash)