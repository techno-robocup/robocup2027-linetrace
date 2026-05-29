# Header-only, dependency-free shared types (config, label codec, mixing).
# Usable by the laptop control station without pulling in LibTorch.
add_library(common INTERFACE)
target_include_directories(common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(common INTERFACE cxx_std_17)
