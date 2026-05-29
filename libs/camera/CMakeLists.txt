# Camera frame source. OpenCV backend always; native libcamera when available.
add_library(camera STATIC
  src/frame_source.cpp
  src/opencv_source.cpp
  src/libcamera_source.cpp
)
target_include_directories(camera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(camera PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera PUBLIC ${OpenCV_LIBS})
target_compile_features(camera PUBLIC cxx_std_17)

# Optional native libcamera backend (preferred on the Raspberry Pi).
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBCAMERA QUIET libcamera)
  if(LIBCAMERA_FOUND)
    target_compile_definitions(camera PRIVATE HAVE_LIBCAMERA)
    target_include_directories(camera PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
    target_link_libraries(camera PUBLIC ${LIBCAMERA_LIBRARIES})
    message(STATUS "camera: native libcamera ENABLED")
  else()
    message(STATUS "camera: libcamera not found; OpenCV/GStreamer backend only")
  endif()
endif()
