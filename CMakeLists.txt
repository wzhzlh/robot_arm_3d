cmake_minimum_required(VERSION 3.22)

project(robot_arm_3d C ASM)

add_compile_definitions(USE_HAL_DRIVER STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

set(APP_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/My_task
    ${CMAKE_CURRENT_SOURCE_DIR}/My_lib
)

set(APP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/My_task/start_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/My_task/task_init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/My_lib/commuction.c
    ${CMAKE_CURRENT_SOURCE_DIR}/My_lib/crc_ccitt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/My_lib/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/My_lib/k230.c
)

add_executable(${CMAKE_PROJECT_NAME} ${APP_SOURCES})

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${APP_INCLUDE_DIRS})

add_subdirectory(cmake/stm32cubemx)

target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx STM32_Drivers FreeRTOS m)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map
)
