if(CONFIG_ST25R100)
    zephyr_library()
    zephyr_include_directories(.)
    zephyr_library_sources(
        platform.c
        st25r100_drv.c
        st25r100_spi.c
        st25r100_trigger.c
    )
endif()

add_subdirectory(st/rfal)
add_subdirectory(st/st25r100)