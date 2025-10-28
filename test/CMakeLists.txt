idf_component_register(
    SRCS 
        "test_shtc3_sensor.cpp"
        "test_sensirion_i2c.c"
    REQUIRES 
        unity
        shtc3Sensor
        shtc1
        freertos
    INCLUDE_DIRS 
        "."
        "../sensors/shtc3Sensor"
        "../components/shtc1"
)

# Set C++ standard
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)