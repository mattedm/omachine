idf_component_register(
    SRCS "machine_state.c" "strokes.c" "twai_odrive_control.c" "espnow_manipulator.cpp" "firmware_valid.c" "firmware_receiver.c" "log_async.c" "odrive_setup.c" "status_led.c"
    REQUIRES esp_driver_twai
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_wifi esp_event esp_timer nvs_flash app_update esp_ringbuf esp_driver_rmt
)
