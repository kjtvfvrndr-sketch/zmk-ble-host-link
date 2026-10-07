zephyr_include_directories(include)

if(CONFIG_ZMK_BLE_HOST_LINK)
  target_sources(app PRIVATE src/ble_host_link.c src/search.c src/decisions.c)
endif()
target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_BLE_HOST_LINK app PRIVATE src/behaviors/behavior_ble_host_link.c)
