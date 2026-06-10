add_executable(executor src/main.cpp)
target_link_libraries(executor PRIVATE common common_torch camera esp32_driver netutil)
target_compile_features(executor PRIVATE cxx_std_17)
