message(WARNING " -- Adding ZMK app module: ${ZEPHYR_ZMK_MODULE_DIR}/app")
zephyr_library()

zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)

# zephyr_library_sources_ifdef(CONFIG_BDKB_BATTERY_STATUS_LEDS src/battery_status_leds.c)
# zephyr_library_sources_ifdef(CONFIG_BDKB_SP3T_MODE_SWITCH src/sp3t_mode_switch.c)
