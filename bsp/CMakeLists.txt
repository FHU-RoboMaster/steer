set(_bsp_sources
    ${CMAKE_CURRENT_LIST_DIR}/bsp_fdcan/bsp_fdcan.c
    ${CMAKE_CURRENT_LIST_DIR}/bsp_gpio/bsp_gpio.c
    ${CMAKE_CURRENT_LIST_DIR}/bsp_uart/bsp_uart.c
    ${CMAKE_CURRENT_LIST_DIR}/dwt/bsp_dwt.c
    ${CMAKE_CURRENT_LIST_DIR}/log/bsp_log.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_bsp_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/bsp_fdcan
    ${CMAKE_CURRENT_LIST_DIR}/bsp_gpio
    ${CMAKE_CURRENT_LIST_DIR}/bsp_uart
    ${CMAKE_CURRENT_LIST_DIR}/dwt
    ${CMAKE_CURRENT_LIST_DIR}/log

    # ${CMAKE_CURRENT_LIST_DIR}/bsp_can
    # ${CMAKE_CURRENT_LIST_DIR}/bsp_iic
    # ${CMAKE_CURRENT_LIST_DIR}/bsp_usb
)
