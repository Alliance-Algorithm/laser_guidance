cmake_minimum_required(VERSION 3.16)
project(laser_guidance LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

option(WITH_ONNXRUNTIME "Enable ONNX Runtime model backend." OFF)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime installation")
option(WITH_TENSORRT "Enable TensorRT engine backend." OFF)
option(WITH_FT4222 "Enable FT4222H USB-to-SPI support." ON)
option(WITH_HIKCAMERA "Enable Hikcamera industrial camera support." ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgcodecs imgproc videoio)
find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE ALL_SOURCES src/*.cpp)
file(GLOB_RECURSE SUPPORT_SOURCES
  src/core/*.cpp
  src/runtime/support.cpp
  src/runtime/runtime_command.cpp)
file(GLOB_RECURSE VISION_SOURCES src/vision/*.cpp)
list(FILTER VISION_SOURCES EXCLUDE REGEX "tensorrt_engine\.cpp$")
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

# ---- ONNX Runtime (optional) --------------------------------------------------
if(WITH_ONNXRUNTIME)
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime/core/session/onnxruntime_cxx_api.h
    HINTS ${ONNXRUNTIME_ROOT} ENV ONNXRUNTIME_ROOT PATH_SUFFIXES include)
  find_library(ONNXRUNTIME_LIBRARY onnxruntime
    HINTS ${ONNXRUNTIME_ROOT} ENV ONNXRUNTIME_ROOT PATH_SUFFIXES lib lib64)
  if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR "WITH_ONNXRUNTIME=ON requires ONNX Runtime.")
  endif()
  add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")
  add_compile_definitions(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=1)
  message(STATUS "ONNX Runtime enabled")
endif()

# ---- TensorRT (optional) -------------------------------------------------------
if(WITH_TENSORRT)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATHS /usr/include /usr/local/include)
  find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h PATHS /opt/cuda/include /usr/local/cuda/include)
  find_library(TENSORRT_LIBRARY nvinfer)
  find_library(TENSORRT_ONNX_PARSER nvonnxparser)
  find_library(TENSORRT_PLUGIN nvinfer_plugin)
  find_library(CUDA_LIBRARY cudart PATHS /opt/cuda/lib64)
  find_library(CUDA_RT_LIBRARY cuda PATHS /opt/cuda/lib64/stubs)
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT TENSORRT_ONNX_PARSER OR NOT TENSORRT_PLUGIN)
    message(FATAL_ERROR "WITH_TENSORRT=ON requires TensorRT.")
  endif()
  if(NOT CUDA_INCLUDE_DIR)
    message(FATAL_ERROR "WITH_TENSORRT=ON requires CUDA headers (cuda_runtime_api.h)")
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
  add_compile_definitions(RMCS_LASER_GUIDANCE_WITH_TENSORRT=1)
  list(APPEND VISION_SOURCES src/vision/tensorrt_engine.cpp)
  message(STATUS "TensorRT enabled")
endif()

# ---- FT4222H (optional) --------------------------------------------------------
if(WITH_FT4222)
  set(FT4222_HINT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222/lib/libft4222.so.1.4.4.232")
  set(FT4222_RUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222/lib")
  if(EXISTS "${FT4222_HINT}")
    set(FT4222_LIBRARY "${FT4222_HINT}")
  else()
    find_library(FT4222_LIBRARY ft4222
      PATHS ${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222/lib /usr/local/lib /usr/lib NO_DEFAULT_PATH)
    if(NOT FT4222_LIBRARY)
      find_library(FT4222_LIBRARY ft4222)
    endif()
  endif()
  if(NOT FT4222_LIBRARY)
    message(FATAL_ERROR "WITH_FT4222=ON requires libft4222.so in project root or /usr/local/lib")
  endif()
  add_library(ft4222::ft4222 SHARED IMPORTED)
  set_target_properties(ft4222::ft4222 PROPERTIES IMPORTED_LOCATION "${FT4222_LIBRARY}")
  add_compile_definitions(WITH_FT4222=1)
  message(STATUS "FT4222H enabled (${FT4222_LIBRARY})")
endif()

# ---- Hikcamera (optional) ------------------------------------------------------
if(WITH_HIKCAMERA)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/hikcamera/CMakeLists.txt")
    message(FATAL_ERROR "WITH_HIKCAMERA=ON requires vendor/hikcamera submodule. "
      "Run: git submodule update --init")
  endif()
  add_compile_definitions(WITH_HIKCAMERA=1)
  add_subdirectory(vendor/hikcamera)
  get_target_property(HIKCAMERA_RUNTIME_DIR hikcamera HIKCAMERA_RUNTIME_DIR)
  message(STATUS "Hikcamera enabled")
endif()

# ---- common settings -----------------------------------------------------------
set(COMMON_LIBS yaml-cpp::yaml-cpp ${OpenCV_LIBS} rt pthread)
if(WITH_FT4222)
  list(APPEND COMMON_LIBS ft4222::ft4222)
endif()
if(WITH_HIKCAMERA)
  list(APPEND COMMON_LIBS hikcamera)
endif()

set(COMMON_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/cuda)

set(DEFAULT_PATH_DEFS
  RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml"
  RMCS_LASER_GUIDANCE_DEFAULT_SAMPLE_REPLAY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test_data/sample_images"
  RMCS_LASER_GUIDANCE_DEFAULT_VIDEO_SESSION_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/videos")

# ---- shared library (compile once, link everywhere) ----------------------------
add_library(laser_guidance_support ${SUPPORT_SOURCES})
target_include_directories(laser_guidance_support PUBLIC ${COMMON_INCLUDES})
target_compile_definitions(laser_guidance_support PRIVATE ${DEFAULT_PATH_DEFS})
target_link_libraries(laser_guidance_support PUBLIC yaml-cpp::yaml-cpp ${OpenCV_LIBS})

add_library(laser_guidance_tracking ${TRACKING_SOURCES})
target_include_directories(laser_guidance_tracking PUBLIC ${COMMON_INCLUDES})
target_compile_definitions(laser_guidance_tracking PRIVATE ${DEFAULT_PATH_DEFS})
target_link_libraries(laser_guidance_tracking PUBLIC ${COMMON_LIBS})

target_link_libraries(laser_guidance_support PUBLIC laser_guidance_tracking)

add_library(laser_guidance_vision ${VISION_SOURCES})
target_include_directories(laser_guidance_vision PUBLIC ${COMMON_INCLUDES})
target_compile_definitions(laser_guidance_vision PRIVATE ${DEFAULT_PATH_DEFS})
target_link_libraries(laser_guidance_vision PUBLIC laser_guidance_support ${COMMON_LIBS})
if(WITH_ONNXRUNTIME)
  target_link_libraries(laser_guidance_vision PUBLIC onnxruntime::onnxruntime)
endif()
if(WITH_TENSORRT)
  target_link_libraries(laser_guidance_vision PUBLIC
    TensorRT::nvinfer TensorRT::nvonnxparser TensorRT::nvinfer_plugin
    ${CUDA_LIBRARY} ${CUDA_RT_LIBRARY})
endif()

add_library(laser_guidance_guidance ${GUIDANCE_SOURCES})
target_include_directories(laser_guidance_guidance PUBLIC ${COMMON_INCLUDES})
target_compile_definitions(laser_guidance_guidance PRIVATE ${DEFAULT_PATH_DEFS})
target_link_libraries(laser_guidance_guidance PUBLIC laser_guidance_support ${COMMON_LIBS})
if(WITH_FT4222)
  target_link_libraries(laser_guidance_guidance PUBLIC ft4222::ft4222)
endif()

add_library(laser_guidance_bridges ${BRIDGE_SOURCES})
target_include_directories(laser_guidance_bridges PUBLIC ${COMMON_INCLUDES})
target_compile_definitions(laser_guidance_bridges PRIVATE ${DEFAULT_PATH_DEFS})
target_link_libraries(laser_guidance_bridges PUBLIC
  laser_guidance_support laser_guidance_tracking ${COMMON_LIBS})

add_library(laser_guidance_runtime ${RUNTIME_SOURCES} ${CAPTURE_SOURCES})
target_include_directories(laser_guidance_runtime PUBLIC ${COMMON_INCLUDES})
target_compile_definitions(laser_guidance_runtime PRIVATE ${DEFAULT_PATH_DEFS})
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

if(WITH_FT4222)
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
endif()

if(WITH_HIKCAMERA)
  foreach(_target
      laser_guidance_runtime)
    set_property(TARGET ${_target} APPEND PROPERTY BUILD_RPATH "${HIKCAMERA_RUNTIME_DIR}")
  endforeach()
endif()

# ---- tools ---------------------------------------------------------------------
enable_testing()

file(GLOB TOOL_SOURCES tools/*.cpp)
foreach(_src ${TOOL_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(tool_${_name} ${_src})
  target_include_directories(tool_${_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tools)
  target_compile_definitions(tool_${_name} PRIVATE ${DEFAULT_PATH_DEFS})
  target_link_libraries(tool_${_name} PRIVATE rmcs_laser_guidance_core)
  if(WITH_FT4222)
    set_property(TARGET tool_${_name} APPEND PROPERTY BUILD_RPATH "${FT4222_RUNTIME_DIR}")
    add_dependencies(tool_${_name} ft4222_runtime_link)
  endif()
  if(WITH_HIKCAMERA)
    set_property(TARGET tool_${_name} APPEND PROPERTY BUILD_RPATH "${HIKCAMERA_RUNTIME_DIR}")
  endif()
endforeach()

# ---- tests ---------------------------------------------------------------------
file(GLOB_RECURSE TEST_SOURCES tests/*_test.cpp)
foreach(_src ${TEST_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(${_name} ${_src})
  target_include_directories(${_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tools ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_definitions(${_name} PRIVATE ${DEFAULT_PATH_DEFS})
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
  if(WITH_FT4222)
    set_property(TARGET ${_name} APPEND PROPERTY BUILD_RPATH "${FT4222_RUNTIME_DIR}")
    add_dependencies(${_name} ft4222_runtime_link)
  endif()
  if(WITH_HIKCAMERA)
    set_property(TARGET ${_name} APPEND PROPERTY BUILD_RPATH "${HIKCAMERA_RUNTIME_DIR}")
  endif()
  add_test(NAME ${_name} COMMAND ${_name})
endforeach()
