idf_component_register(
    SRCS
        "sensirion_common.c"
        "sensirion_i2c.c"
        "shtc1.c"
        "sht_git_version.c"
    REQUIRES driver
    INCLUDE_DIRS
        "."
)
