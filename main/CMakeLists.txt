idf_component_register(SRCS "SHTC1Sensor.cpp" "main.cpp"
                       PRIV_REQUIRES spi_flash driver freertos
                       REQUIRES shtc1
                       INCLUDE_DIRS "")
