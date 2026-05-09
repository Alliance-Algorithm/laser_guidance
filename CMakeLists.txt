cmake_minimum_required(VERSION 3.16)
project(laser_guidance LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(WITH_ONNXRUNTIME "Enable ONNX Runtime model backend." OFF)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime installation")
option(WITH_TENSORRT "Enable TensorRT engine backend." OFF)
option(WITH_FT4222 "Enable FT4222H USB-to-SPI support." ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgcodecs imgproc videoio)
find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE ALL_SOURCES src/*.cpp)

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
  add_compile_definitions(WITH_ONNXRUNTIME=1)
  message(STATUS "ONNX Runtime enabled")
endif()

# ---- TensorRT (optional) -------------------------------------------------------
if(WITH_TENSORRT)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATHS /usr/include /usr/local/include)
  find_library(TENSORRT_LIBRARY nvinfer)
  find_library(TENSORRT_ONNX_PARSER nvonnxparser)
  find_library(TENSORRT_PLUGIN nvinfer_plugin)
  find_library(CUDA_LIBRARY cudart PATHS /opt/cuda/lib64)
  find_library(CUDA_RT_LIBRARY cuda PATHS /opt/cuda/lib64/stubs)
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT TENSORRT_ONNX_PARSER OR NOT TENSORRT_PLUGIN)
    message(FATAL_ERROR "WITH_TENSORRT=ON requires TensorRT.")
  endif()
  add_library(TensorRT::nvinfer UNKNOWN IMPORTED)
  set_target_properties(TensorRT::nvinfer PROPERTIES
    IMPORTED_LOCATION "${TENSORRT_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${TENSORRT_INCLUDE_DIR}")
  add_library(TensorRT::nvonnxparser UNKNOWN IMPORTED)
  set_target_properties(TensorRT::nvonnxparser PROPERTIES
    IMPORTED_LOCATION "${TENSORRT_ONNX_PARSER}" INTERFACE_INCLUDE_DIRECTORIES "${TENSORRT_INCLUDE_DIR}")
  add_library(TensorRT::nvinfer_plugin UNKNOWN IMPORTED)
  set_target_properties(TensorRT::nvinfer_plugin PROPERTIES
    IMPORTED_LOCATION "${TENSORRT_PLUGIN}" INTERFACE_INCLUDE_DIRECTORIES "${TENSORRT_INCLUDE_DIR}")
  add_compile_definitions(WITH_TENSORRT=1)
  message(STATUS "TensorRT enabled")
endif()

# ---- FT4222H (optional) --------------------------------------------------------
if(WITH_FT4222)
  find_library(FT4222_LIBRARY ft4222
    PATHS ${CMAKE_CURRENT_SOURCE_DIR} /usr/local/lib /usr/lib NO_DEFAULT_PATH)
  if(NOT FT4222_LIBRARY)
    find_library(FT4222_LIBRARY ft4222)
  endif()
  if(NOT FT4222_LIBRARY)
    message(FATAL_ERROR "WITH_FT4222=ON requires libft4222.so in project root or /usr/local/lib")
  endif()
  add_compile_definitions(WITH_FT4222=1)
  message(STATUS "FT4222H enabled (${FT4222_LIBRARY})")
endif()

# ---- common settings -----------------------------------------------------------
set(COMMON_LIBS yaml-cpp::yaml-cpp ${OpenCV_LIBS} rt pthread)
if(WITH_FT4222)
  list(APPEND COMMON_LIBS ${FT4222_LIBRARY})
endif()

set(COMMON_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(DEFAULT_PATH_DEFS
  RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml"
  RMCS_LASER_GUIDANCE_DEFAULT_SAMPLE_REPLAY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test_data/sample_images"
  RMCS_LASER_GUIDANCE_DEFAULT_VIDEO_SESSION_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/videos")

# ---- tools ---------------------------------------------------------------------
enable_testing()

file(GLOB TOOL_SOURCES tools/*.cpp)
foreach(_src ${TOOL_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(tool_${_name} ${_src} ${ALL_SOURCES})
  target_include_directories(tool_${_name} PRIVATE ${COMMON_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/tools)
  target_compile_definitions(tool_${_name} PRIVATE ${DEFAULT_PATH_DEFS})
  target_link_libraries(tool_${_name} PRIVATE ${COMMON_LIBS})
  if(WITH_ONNXRUNTIME)
    target_link_libraries(tool_${_name} PRIVATE onnxruntime::onnxruntime)
  endif()
  if(WITH_TENSORRT)
    target_link_libraries(tool_${_name} PRIVATE
      TensorRT::nvinfer TensorRT::nvonnxparser TensorRT::nvinfer_plugin
      ${CUDA_LIBRARY} ${CUDA_RT_LIBRARY})
  endif()
endforeach()

# ---- tests ---------------------------------------------------------------------
file(GLOB_RECURSE TEST_SOURCES tests/*_test.cpp)
foreach(_src ${TEST_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(${_name} ${_src} ${ALL_SOURCES})
  target_include_directories(${_name} PRIVATE ${COMMON_INCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}/tools ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_definitions(${_name} PRIVATE ${DEFAULT_PATH_DEFS})
  target_link_libraries(${_name} PRIVATE ${COMMON_LIBS})
  if(WITH_ONNXRUNTIME)
    target_link_libraries(${_name} PRIVATE onnxruntime::onnxruntime)
  endif()
  if(WITH_TENSORRT)
    target_link_libraries(${_name} PRIVATE
      TensorRT::nvinfer TensorRT::nvonnxparser TensorRT::nvinfer_plugin
      ${CUDA_LIBRARY} ${CUDA_RT_LIBRARY})
  endif()
  add_test(NAME ${_name} COMMAND ${_name})
endforeach()
