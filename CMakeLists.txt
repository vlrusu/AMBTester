cmake_minimum_required(VERSION 3.13)

set(NAME AMBTester)
set(PICO_SDK_PATH "/home/mu2e/pico/pico-sdk")
include(pico_sdk_import.cmake)

project(AMBTester C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Add a compilation subdirectory
add_subdirectory(lib/Config)
add_subdirectory(lib/Infrared)
add_subdirectory(lib/Icm20948)
add_subdirectory(lib/LCD)
add_subdirectory(lib/Fonts)
add_subdirectory(lib/GUI)



# Add a header directory
include_directories(./lib/Config)
include_directories(./lib/GUI)
include_directories(./lib/Icm20948)
include_directories(./lib/LCD)
include_directories(./lib/Infrared)


add_executable(${PROJECT_NAME}
    src/main.c
    src/ADG706.c
)

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_adc hardware_pio hardware_spi pico_multicore LCD Infrared Icm20948 GUI Fonts Config)



pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_uart(AMBTester 0)
pico_enable_stdio_usb(AMBTester 1)


install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)
