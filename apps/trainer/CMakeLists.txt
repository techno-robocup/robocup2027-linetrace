add_executable(trainer
  src/train.cpp
  src/dataset.cpp
  src/augment.cpp
)
target_link_libraries(trainer PRIVATE common common_torch)
target_compile_features(trainer PRIVATE cxx_std_17)

add_executable(model_probe src/model_probe.cpp)
target_link_libraries(model_probe PRIVATE common common_torch)
target_compile_features(model_probe PRIVATE cxx_std_17)
