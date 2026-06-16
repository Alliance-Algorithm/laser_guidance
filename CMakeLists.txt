cmake_minimum_required(VERSION 3.16)
project(laser_guidance LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

# All backends are unconditionally required (no compile-time options / #ifdef).
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime installation")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgcodecs imgproc videoio)
find_package(yaml-cpp REQUIRED)

find_library(ZMQ_LIBRARY zmq)
find_path(ZMQ_INCLUDE_DIR zmq.hpp PATH_SUFFIXES include)
if(NOT ZMQ_LIBRARY OR NOT ZMQ_INCLUDE_DIR)
  message(FATAL_ERROR "libzmq is required.")
endif()
add_library(zmq::zmq UNKNOWN IMPORTED)
set_target_properties(zmq::zmq PROPERTIES
  IMPORTED_LOCATION "${ZMQ_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ZMQ_INCLUDE_DIR}")
message(STATUS "ZMQ enabled")

file(GLOB_RECURSE ALL_SOURCES src/*.cpp)
file(GLOB_RECURSE SUPPORT_SOURCES
  src/core/*.cpp
  src/runtime/support.cpp
  src/runtime/runtime_command.cpp)
file(GLOB_RECURSE VISION_SOURCES src/vision/*.cpp)
file(GLOB_RECURSE TRACKING_SOURCES src/tracking/*.cpp)
file(GLOB_RECURSE GUIDANCE_SOURCES src/guidance/*.cpp)
file(GLOB_RECURSE BRIDGE_SOURCES src/streaming/*.cpp src/bridges/*.cpp)
set(CAPTURE_SOURCES
  src/capture/capture_device.cpp
  src/capture/v4l2_capture.cpp)
set(RUNTIME_SOURCES
  src/runtime/runtime.cpp
  src/runtime/control_loop.cpp
  src/runtime/guidance_ops_app.cpp
  src/runtime/guidance_session.cpp
  src/runtime/overlay_renderer.cpp
  src/runtime/perception_runner.cpp
  src/runtime/runtime_outputs.cpp
  src/io/ft4222_spi.cpp)

# ---- ONNX Runtime (required) ---------------------------------------------------
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime/core/session/onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT} ENV ONNXRUNTIME_ROOT PATH_SUFFIXES include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS ${ONNXRUNTIME_ROOT} ENV ONNXRUNTIME_ROOT PATH_SUFFIXES lib lib64)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime is required. Set ONNXRUNTIME_ROOT or install to system paths.")
endif()
add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR};${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime")
message(STATUS "ONNX Runtime enabled")

# ---- TensorRT (required) -------------------------------------------------------
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include /usr/include/x86_64-linux-gnu /usr/local/include)
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  PATHS /opt/cuda/include /usr/local/cuda/include)
find_library(TENSORRT_LIBRARY nvinfer)
find_library(TENSORRT_ONNX_PARSER nvonnxparser)
find_library(TENSORRT_PLUGIN nvinfer_plugin)
find_library(CUDA_LIBRARY cudart
  PATHS /opt/cuda/lib64 /usr/local/cuda/lib64 /usr/lib/x86_64-linux-gnu)
find_library(CUDA_RT_LIBRARY cuda
  PATHS /opt/cuda/lib64/stubs /usr/local/cuda/lib64/stubs)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT TENSORRT_ONNX_PARSER OR NOT TENSORRT_PLUGIN)
  message(FATAL_ERROR "TensorRT is required.")
endif()
if(NOT CUDA_INCLUDE_DIR)
  message(FATAL_ERROR "CUDA headers (cuda_runtime_api.h) are required.")
endif()
if(NOT CUDA_LIBRARY)
  message(FATAL_ERROR "CUDA runtime library (libcudart.so) is required.")
endif()
if(NOT CUDA_RT_LIBRARY)
  message(FATAL_ERROR "CUDA driver stub library (libcuda.so) is required.")
endif()
add_library(TensorRT::nvinfer UNKNOWN IMPORTED)
set_target_properties(TensorRT::nvinfer PROPERTIES
  IMPORTED_LOCATION "${TENSORRT_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${TENSORRT_INCLUDE_DIR};${CUDA_INCLUDE_DIR}")
add_library(TensorRT::nvonnxparser UNKNOWN IMPORTED)
set_target_properties(TensorRT::nvonnxparser PROPERTIES
  IMPORTED_LOCATION "${TENSORRT_ONNX_PARSER}"
  INTERFACE_INCLUDE_DIRECTORIES "${TENSORRT_INCLUDE_DIR};${CUDA_INCLUDE_DIR}")
add_library(TensorRT::nvinfer_plugin UNKNOWN IMPORTED)
set_target_properties(TensorRT::nvinfer_plugin PROPERTIES
  IMPORTED_LOCATION "${TENSORRT_PLUGIN}"
  INTERFACE_INCLUDE_DIRECTORIES "${TENSORRT_INCLUDE_DIR};${CUDA_INCLUDE_DIR}")
message(STATUS "TensorRT enabled")

# ---- FT4222H (required) --------------------------------------------------------
set(FT4222_HINT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222/lib/libft4222.so.1.4.4.232")
set(FT4222_RUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222/lib")
if(EXISTS "${FT4222_HINT}")
  set(FT4222_LIBRARY "${FT4222_HINT}")
else()
  find_library(FT4222_LIBRARY ft4222
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222/lib /opt/libft4222 /usr/local/lib /usr/lib
    NO_DEFAULT_PATH)
  if(NOT FT4222_LIBRARY)
    find_library(FT4222_LIBRARY ft4222)
  endif()
endif()
if(NOT FT4222_LIBRARY)
  message(FATAL_ERROR "libft4222.so is required. Place it in vendor/ft4222/lib, /opt/libft4222, or system paths.")
endif()
add_library(ft4222::ft4222 SHARED IMPORTED)
set_target_properties(ft4222::ft4222 PROPERTIES IMPORTED_LOCATION "${FT4222_LIBRARY}")
message(STATUS "FT4222H enabled (${FT4222_LIBRARY})")

# ---- ROS2 Bridge (always enabled) ----------------------------------------------
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
message(STATUS "ROS2 bridge enabled")

# ---- Hikcamera (always enabled) ------------------------------------------------
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/hikcamera/CMakeLists.txt")
  message(FATAL_ERROR "Hikcamera requires vendor/hikcamera submodule. "
    "Run: git submodule update --init")
endif()
add_subdirectory(vendor/hikcamera)
get_target_property(HIKCAMERA_RUNTIME_DIR hikcamera HIKCAMERA_RUNTIME_DIR)
get_target_property(HIKCAMERA_SDK_MODE hikcamera HIKCAMERA_SDK_MODE)
get_target_property(HIKCAMERA_SDK_ROOT hikcamera HIKCAMERA_SDK_ROOT)
string(REPLACE "'" "'\\''" HIKCAMERA_SDK_MODE_SHELL "${HIKCAMERA_SDK_MODE}")
string(REPLACE "'" "'\\''" HIKCAMERA_SDK_ROOT_SHELL "${HIKCAMERA_SDK_ROOT}")
string(REPLACE "'" "'\\''" HIKCAMERA_RUNTIME_DIR_SHELL "${HIKCAMERA_RUNTIME_DIR}")
set(HIKCAMERA_RUNTIME_ENV_CONTENT
  "HIKCAMERA_BUILD_SDK_MODE='${HIKCAMERA_SDK_MODE_SHELL}'\nHIKCAMERA_BUILD_SDK_ROOT='${HIKCAMERA_SDK_ROOT_SHELL}'\nHIKCAMERA_BUILD_RUNTIME_DIR='${HIKCAMERA_RUNTIME_DIR_SHELL}'\n")
file(CONFIGURE
  OUTPUT "${CMAKE_BINARY_DIR}/hikcamera-runtime.env"
  CONTENT "${HIKCAMERA_RUNTIME_ENV_CONTENT}")
message(STATUS "Hikcamera SDK mode: ${HIKCAMERA_SDK_MODE}")
message(STATUS "Hikcamera SDK root: ${HIKCAMERA_SDK_ROOT}")
message(STATUS "Hikcamera runtime dir: ${HIKCAMERA_RUNTIME_DIR}")
message(STATUS "Hikcamera enabled")

# ---- common settings -----------------------------------------------------------
set(COMMON_LIBS yaml-cpp::yaml-cpp ${OpenCV_LIBS} rt pthread ft4222::ft4222 hikcamera zmq::zmq)

set(COMMON_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/cuda)

# ---- shared library ------------------------------------------------------------
add_library(laser_guidance_support ${SUPPORT_SOURCES})
target_include_directories(laser_guidance_support PUBLIC ${COMMON_INCLUDES})
target_link_libraries(laser_guidance_support PUBLIC yaml-cpp::yaml-cpp ${OpenCV_LIBS})

add_library(laser_guidance_tracking ${TRACKING_SOURCES})
target_include_directories(laser_guidance_tracking PUBLIC ${COMMON_INCLUDES})
target_link_libraries(laser_guidance_tracking PUBLIC ${COMMON_LIBS})

target_link_libraries(laser_guidance_support PUBLIC laser_guidance_tracking)

add_library(laser_guidance_vision ${VISION_SOURCES})
target_include_directories(laser_guidance_vision PUBLIC ${COMMON_INCLUDES})
target_link_libraries(laser_guidance_vision PUBLIC
  laser_guidance_support ${COMMON_LIBS}
  onnxruntime::onnxruntime
  TensorRT::nvinfer TensorRT::nvonnxparser TensorRT::nvinfer_plugin
  ${CUDA_LIBRARY} ${CUDA_RT_LIBRARY})

add_library(laser_guidance_guidance ${GUIDANCE_SOURCES})
target_include_directories(laser_guidance_guidance PUBLIC ${COMMON_INCLUDES})
target_link_libraries(laser_guidance_guidance PUBLIC laser_guidance_support ${COMMON_LIBS})

add_library(laser_guidance_bridges ${BRIDGE_SOURCES})
target_include_directories(laser_guidance_bridges PRIVATE ${COMMON_INCLUDES}
  ${rclcpp_INCLUDE_DIRS} ${visualization_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(laser_guidance_bridges PUBLIC
  laser_guidance_support laser_guidance_tracking ${COMMON_LIBS}
  ${rclcpp_LIBRARIES} ${visualization_msgs_LIBRARIES} ${std_msgs_LIBRARIES})

add_library(laser_guidance_runtime ${RUNTIME_SOURCES} ${CAPTURE_SOURCES})
target_include_directories(laser_guidance_runtime PUBLIC ${COMMON_INCLUDES})
target_link_libraries(laser_guidance_runtime PUBLIC
  laser_guidance_support
  laser_guidance_vision
  laser_guidance_tracking
  laser_guidance_guidance
  laser_guidance_bridges
  ${COMMON_LIBS})

add_library(rmcs_laser_guidance_core INTERFACE)
target_include_directories(rmcs_laser_guidance_core INTERFACE ${COMMON_INCLUDES})
target_link_libraries(rmcs_laser_guidance_core INTERFACE
  laser_guidance_support
  laser_guidance_vision
  laser_guidance_tracking
  laser_guidance_guidance
  laser_guidance_bridges
  laser_guidance_runtime)

# FT4222 runtime link (always)
foreach(_target
    laser_guidance_guidance
    laser_guidance_runtime)
  set_property(TARGET ${_target} APPEND PROPERTY BUILD_RPATH "${FT4222_RUNTIME_DIR}")
endforeach()

add_custom_target(ft4222_runtime_link ALL
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    "${FT4222_LIBRARY}"
    "${CMAKE_BINARY_DIR}/libft4222.so"
  VERBATIM)

# Hikcamera runtime RPATH (always)
foreach(_target
    laser_guidance_runtime)
  set_property(TARGET ${_target} APPEND PROPERTY BUILD_RPATH "${HIKCAMERA_RUNTIME_DIR}")
endforeach()

# ---- tools ---------------------------------------------------------------------
enable_testing()

file(GLOB TOOL_SOURCES tools/*.cpp)
foreach(_src ${TOOL_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(tool_${_name} ${_src})
  target_include_directories(tool_${_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tools)
  target_link_libraries(tool_${_name} PRIVATE rmcs_laser_guidance_core)
  set_property(TARGET tool_${_name} APPEND PROPERTY BUILD_RPATH "${FT4222_RUNTIME_DIR}")
  add_dependencies(tool_${_name} ft4222_runtime_link)
  set_property(TARGET tool_${_name} APPEND PROPERTY BUILD_RPATH "${HIKCAMERA_RUNTIME_DIR}")
endforeach()

# ---- tests ---------------------------------------------------------------------
file(GLOB_RECURSE TEST_SOURCES tests/*_test.cpp)
foreach(_src ${TEST_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(${_name} ${_src})
  target_include_directories(${_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tools ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  if(_src MATCHES "tests/runtime/")
    target_link_libraries(${_name} PRIVATE
      laser_guidance_support
      laser_guidance_vision
      laser_guidance_tracking
      laser_guidance_guidance
      laser_guidance_bridges
      laser_guidance_runtime)
  else()
    target_link_libraries(${_name} PRIVATE rmcs_laser_guidance_core)
  endif()
  set_property(TARGET ${_name} APPEND PROPERTY BUILD_RPATH "${FT4222_RUNTIME_DIR}")
  add_dependencies(${_name} ft4222_runtime_link)
  set_property(TARGET ${_name} APPEND PROPERTY BUILD_RPATH "${HIKCAMERA_RUNTIME_DIR}")
  add_test(NAME ${_name} COMMAND ${_name})
  set_property(TEST ${_name} PROPERTY ENVIRONMENT
    "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}:${FT4222_RUNTIME_DIR}:${HIKCAMERA_RUNTIME_DIR}:$ENV{LD_LIBRARY_PATH}")
  set_property(TEST ${_name} PROPERTY WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endforeach()
