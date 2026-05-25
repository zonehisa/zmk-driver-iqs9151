zephyr_library()

set(_zmk_app_include_candidates
  ${ZEPHYR_BASE}/../app/include
  ${ZEPHYR_BASE}/../zmk/app/include
)
foreach(_inc ${_zmk_app_include_candidates})
  if(EXISTS ${_inc})
    zephyr_library_include_directories(${_inc})
    break()
  endif()
endforeach()
zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR}/../include)

zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_DYNAMIC_SCALER input_processor_dynamic_scaler.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_BEHAVIORS_CONSUME input_processor_behaviors_consume.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_PINCH_ZOOM input_processor_pinch_zoom.c)
