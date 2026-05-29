# common_test needs Torch + OpenCV (via common_torch).
if(TARGET common_torch)
  add_executable(common_test common_test.cpp)
  target_link_libraries(common_test PRIVATE common common_torch)
  add_test(NAME common_test COMMAND common_test)
endif()

# transfer_client_test needs a running transfer_node server, so it is driven by
# an external script rather than a plain add_test.
if(TARGET transfer_client)
  add_executable(transfer_client_test transfer_client_test.cpp)
  target_link_libraries(transfer_client_test PRIVATE transfer_client)
endif()

# esp32_driver_test uses a pty loopback (links libutil for openpty).
if(TARGET esp32_driver)
  add_executable(esp32_driver_test esp32_driver_test.cpp)
  target_link_libraries(esp32_driver_test PRIVATE esp32_driver util)
  add_test(NAME esp32_driver_test COMMAND esp32_driver_test)
endif()
