idf_component_register(SRCS "SHTC1Sensor.cpp"
                       PRIV_REQUIRES spi_flash driver
                       REQUIRES shtc1
                       INCLUDE_DIRS ""
                       )