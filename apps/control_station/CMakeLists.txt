add_executable(control_station
  src/main.cpp
  src/gamepad_factory.cpp
  src/gamepad_scripted.cpp
  src/gamepad_sdl.cpp
)
target_include_directories(control_station PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(control_station PRIVATE common netutil ${OpenCV_LIBS})
target_compile_features(control_station PRIVATE cxx_std_17)

# Preview window via OpenCV highgui (assumed present in a desktop OpenCV build).
target_compile_definitions(control_station PRIVATE HAVE_HIGHGUI)

# Optional real-gamepad backend via SDL2.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SDL2 QUIET sdl2)
  if(SDL2_FOUND)
    target_compile_definitions(control_station PRIVATE HAVE_SDL2)
    target_include_directories(control_station PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(control_station PRIVATE ${SDL2_LIBRARIES})
    message(STATUS "control_station: SDL2 ENABLED")
  else()
    message(STATUS "control_station: SDL2 not found; scripted input only")
  endif()
endif()
