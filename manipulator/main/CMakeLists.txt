idf_component_register(SRCS "firmware_update.cpp" "firmware_valid.c" "power.cpp" "main.cpp" "fast_as5048a.cpp" "fast_mt6701.cpp" "motor_task.cpp" "inputs.cpp" "rotary.cpp"
                            "link_task.cpp" "display_task.cpp" "encoder_test.cpp" "open_loop_test.cpp" "sensor_test.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_driver_mcpwm esp_timer esp_wifi
                                     esp_event nvs_flash esp_simplefoc lovyangfx esp_adc esp_hal_usb app_update
                                     esp_netif esp_http_client esp_https_ota esp_http_server mbedtls)
