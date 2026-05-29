# ESP32 UART driver: POSIX serial + the id-matched request/response protocol,
# with a background I/O thread. POSIX + std only (no external deps).
find_package(Threads REQUIRED)

add_library(esp32_driver STATIC
  src/esp32_driver.cpp
  src/serial_port.cpp
  src/port_scan.cpp
  src/protocol.cpp
)
target_include_directories(esp32_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(esp32_driver PUBLIC Threads::Threads)
target_compile_features(esp32_driver PUBLIC cxx_std_17)

# Interactive bring-up tool.
add_executable(uart_probe tools/uart_probe.cpp)
target_link_libraries(uart_probe PRIVATE esp32_driver)
