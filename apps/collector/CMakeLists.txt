add_executable(collector
  src/main.cpp
  src/dataset_writer.cpp
)
target_include_directories(collector PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(collector PRIVATE common camera esp32_driver netutil ${OpenCV_LIBS})
target_compile_features(collector PRIVATE cxx_std_17)
