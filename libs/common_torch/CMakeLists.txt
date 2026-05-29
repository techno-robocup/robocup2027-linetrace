# Torch + OpenCV shared code: preprocessing and the LineNet model definition.
# Both the trainer and the executor link this so they agree by construction.
add_library(common_torch STATIC
  src/preprocess.cpp
  src/model.cpp
)
target_include_directories(common_torch PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(common_torch PUBLIC
  common
  ${TORCH_LIBRARIES}
  ${OpenCV_LIBS}
)
target_compile_features(common_torch PUBLIC cxx_std_17)
