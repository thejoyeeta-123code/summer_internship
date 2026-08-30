idf_component_register(
    SRCS
        "inference_spi_ble.cpp"
        "audio_driver.c"
        "microphone.c"
        "gatt_svr.c"
		"rf_model.c"
		

    PRIV_REQUIRES
        esp_driver_i2s
        fatfs
        esp32_BNO08x
    INCLUDE_DIRS
        "."
    REQUIRES bt nvs_flash esp_wifi_remote
)