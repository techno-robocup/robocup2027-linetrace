# Minimal UDP helpers shared by the collector and the control station.
add_library(netutil STATIC src/net.cpp)
target_include_directories(netutil PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(netutil PUBLIC cxx_std_17)
