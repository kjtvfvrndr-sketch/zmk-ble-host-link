zephyr_include_directories(include)

if(CONFIG_ZMK_BLE_HOST_LINK)
  target_sources(app PRIVATE src/ble_host_link.c src/search.c src/decisions.c)
  target_sources_ifdef(CONFIG_ZMK_BLE_HOST_LINK_LOG_HOST_CONNECTIONS app PRIVATE
                       src/host_connections_log.c)
  if(CONFIG_ZMK_BLE_HOST_LINK_LOG_ADVERTISING_OUT_OF_STEP)
    target_sources(app PRIVATE src/advertising_out_of_step_log.c)
    # bt_dev: the stack's internal header.
    set_source_files_properties(src/advertising_out_of_step_log.c TARGET_DIRECTORY app PROPERTIES
                                INCLUDE_DIRECTORIES ${ZEPHYR_BASE}/subsys/bluetooth/host)
  endif()
  if(CONFIG_ZMK_BLE_HOST_LINK_CHECK_OWN_PAIRING_STATE)
    target_sources(app PRIVATE src/own_pairing_state_check.c)
    # bt_pub_key_get(), bt_dh_key_gen() and bt_dev: the stack's internal headers.
    set_source_files_properties(src/own_pairing_state_check.c TARGET_DIRECTORY app PROPERTIES
                                INCLUDE_DIRECTORIES ${ZEPHYR_BASE}/subsys/bluetooth/host)
  endif()
endif()
target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_BLE_HOST_LINK app PRIVATE src/behaviors/behavior_ble_host_link.c)
