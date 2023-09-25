cmake_minimum_required(VERSION 3.16)

# Add COLCON_IGNORE files into submodules at build time to avoid having to fork the repo
set(UROS_DIAGNOSTICS_BRIDGE_DIR extra_ros_packages/micro_ros_diagnostics/micro_ros_diagnostic_bridge)
set(UROS_DIAGNOSTICS_COMMON_DIR extra_ros_packages/micro_ros_diagnostics/micro_ros_common_diagnostics)
execute_process(COMMAND touch ${UROS_DIAGNOSTICS_BRIDGE_DIR}/COLCON_IGNORE ${UROS_DIAGNOSTICS_COMMON_DIR}/COLCON_IGNORE
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        RESULT_VARIABLE TOUCH_RESULT)
if (NOT ${TOUCH_RESULT} MATCHES "0")
    message(SEND_ERROR "Failed to create COLCON_IGNORE files. Build may fail.")
endif ()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS ./components/esp-idf-lib/components)
project(avr_pcc_2023)
