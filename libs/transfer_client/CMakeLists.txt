# Reusable client for the transfer protocol (PUT/GET/LIST). Pure POSIX sockets,
# no external dependencies. Compatible with the transfer_node server.
add_library(transfer_client STATIC
  src/transfer_client.cpp
  src/wire.cpp
)
target_include_directories(transfer_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(transfer_client PUBLIC cxx_std_17)
