cmake_minimum_required(VERSION 3.20)
project(laser_guidance LANGUAGES CXX CUDA)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

# CUDA: target the deployment GPU (Ada / sm_89). Adjust if the target changes.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 89)
endif()
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
# nvcc-generated host stubs trip -Wpedantic (# line directives); silence for CUDA only.
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -Wno-pedantic")

option(LASER_LTO        "Enable link-time optimization" OFF)
option(LASER_UNITY      "Enable unity (jumbo) builds"  OFF)
option(LASER_LOOSE_LINK "Allow undefined symbols (for ROS2 tarball installs)" ON)
if(LASER_LTO)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()
if(LASER_UNITY)
  set(CMAKE_UNITY_BUILD ON)
endif()
if(LASER_LOOSE_LINK)
  foreach(_flag EXE SHARED MODULE)
    set(CMAKE_${_flag}_LINKER_FLAGS
        "${CMAKE_${_flag}_LINKER_FLAGS} -Wl,--disable-new-dtags -Wl,--allow-shlib-undefined")
  endforeach()
endif()

set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime installation")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

# ---- dependencies -----------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio dnn)
if(OpenCV_VERSION VERSION_GREATER_EQUAL "5.0")
  find_package(OpenCV REQUIRED COMPONENTS calib geometry)
else()
  find_package(OpenCV REQUIRED COMPONENTS calib3d)
endif()
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ZMQ (no cmake config shipped by distro)
find_library(ZMQ_LIBRARY zmq)
find_path(ZMQ_INCLUDE_DIR zmq.hpp
  PATH_SUFFIXES include
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/vendor)
if(NOT ZMQ_LIBRARY OR NOT ZMQ_INCLUDE_DIR)
  message(FATAL_ERROR "libzmq is required.")
endif()
add_library(zmq::zmq UNKNOWN IMPORTED)
set_target_properties(zmq::zmq PROPERTIES
  IMPORTED_LOCATION "${ZMQ_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ZMQ_INCLUDE_DIR}")
message(STATUS "ZMQ enabled")

# ---- nlohmann_json ----------------------------------------------------------
find_package(nlohmann_json REQUIRED)
message(STATUS "nlohmann_json enabled")

# ---- ONNX Runtime (required) -------------------------------------------------
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

# ---- TensorRT + CUDA (required) ----------------------------------------------
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include /usr/include/x86_64-linux-gnu /usr/local/include)
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  PATHS /usr/local/cuda/include /opt/cuda/include
        /opt/cuda/targets/x86_64-linux/include)
find_library(TENSORRT_LIBRARY nvinfer)
find_library(TENSORRT_ONNX_PARSER nvonnxparser)
find_library(TENSORRT_PLUGIN nvinfer_plugin)
find_library(CUDA_LIBRARY cudart
  PATHS /opt/cuda/lib64 /opt/cuda/targets/x86_64-linux/lib
        /usr/local/cuda/lib64 /usr/lib/x86_64-linux-gnu)
find_library(CUDA_RT_LIBRARY cuda
  PATHS /opt/cuda/lib64/stubs /opt/cuda/targets/x86_64-linux/lib/stubs
        /usr/local/cuda/lib64/stubs /usr/lib)
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
add_library(CUDA::cudart UNKNOWN IMPORTED)
set_target_properties(CUDA::cudart PROPERTIES
  IMPORTED_LOCATION "${CUDA_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${CUDA_INCLUDE_DIR}")
add_library(CUDA::cuda UNKNOWN IMPORTED)
set_target_properties(CUDA::cuda PROPERTIES IMPORTED_LOCATION "${CUDA_RT_LIBRARY}")
message(STATUS "TensorRT enabled")

# ---- FT4222H (required) ------------------------------------------------------
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
get_filename_component(FT4222_SO_NAME "${FT4222_LIBRARY}" NAME)
add_library(ft4222::ft4222 SHARED IMPORTED)
set_target_properties(ft4222::ft4222 PROPERTIES IMPORTED_LOCATION "${FT4222_LIBRARY}")
message(STATUS "FT4222H enabled (${FT4222_LIBRARY})")

# ---- ROS2 Bridge (always enabled) --------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
message(STATUS "ROS2 bridge enabled")

# ---- Ceres Solver (calibration tool only) ----------------------------------
find_package(Ceres REQUIRED)
message(STATUS "Ceres enabled")

# ---- Hikcamera (always enabled) ----------------------------------------------
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
set_property(TARGET hikcamera APPEND PROPERTY BUILD_RPATH "$ORIGIN/hik-sdk")

# ---- INTERFACE hub: common deps, flags, includes (replaces COMMON_LIBS) -------
add_library(laser_guidance_common INTERFACE)
target_link_libraries(laser_guidance_common INTERFACE
  yaml-cpp::yaml-cpp ${OpenCV_LIBS} rt pthread dl hikcamera zmq::zmq)
target_include_directories(laser_guidance_common INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/ft4222)
target_compile_features(laser_guidance_common INTERFACE cxx_std_23)

# RPATH helper + library macro
function(add_runtime_rpaths target)
  set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/.runtime-compat/lib")
  set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "${FT4222_RUNTIME_DIR}")
  add_dependencies(${target} ft4222_runtime_link)
  set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "${HIKCAMERA_RUNTIME_DIR}")
  set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "$ORIGIN/../lib:$ORIGIN/../lib/hik-sdk")
endfunction()

macro(add_lg_lib name)
  add_library(${name} STATIC ${${name}_SOURCES})
  target_link_libraries(${name} PUBLIC laser_guidance_common)
endmacro()

# ---- source globs ------------------------------------------------------------
set(laser_guidance_support_SOURCES
  src/core/config.cpp
  src/core/debug_renderer.cpp
  src/core/frame_format.cpp
  src/core/replay.cpp
  src/runtime/support.cpp
  src/runtime/runtime_command.cpp)
file(GLOB_RECURSE laser_guidance_tracking_SOURCES CONFIGURE_DEPENDS src/tracking/*.cpp)
file(GLOB_RECURSE laser_guidance_vision_SOURCES   CONFIGURE_DEPENDS src/vision/*.cpp src/vision/*.cu)
file(GLOB_RECURSE laser_guidance_guidance_SOURCES CONFIGURE_DEPENDS src/guidance/*.cpp)
file(GLOB_RECURSE laser_guidance_bridges_SOURCES  CONFIGURE_DEPENDS src/streaming/*.cpp src/bridges/*.cpp)

set(laser_guidance_runtime_SOURCES
  src/capture/capture_device.cpp
  src/capture/v4l2_capture.cpp
  src/capture/hik_backend.cpp
  src/runtime/runtime.cpp
  src/runtime/control_loop.cpp
  src/runtime/guidance_ops_app.cpp
  src/runtime/guidance_session.cpp
  src/runtime/overlay_renderer.cpp
  src/runtime/perception_runner.cpp
  src/runtime/runtime_outputs.cpp
  src/runtime/referee_link.cpp
  src/io/ft4222_spi.cpp)

# ---- libraries ---------------------------------------------------------------
add_lg_lib(laser_guidance_support)
target_link_libraries(laser_guidance_support PUBLIC laser_guidance_tracking)

add_lg_lib(laser_guidance_tracking)

add_lg_lib(laser_guidance_vision)
target_link_libraries(laser_guidance_vision PUBLIC
  laser_guidance_support
  onnxruntime::onnxruntime
  TensorRT::nvinfer TensorRT::nvonnxparser TensorRT::nvinfer_plugin
  CUDA::cudart CUDA::cuda)

add_lg_lib(laser_guidance_guidance)
target_link_libraries(laser_guidance_guidance PUBLIC laser_guidance_support Eigen3::Eigen)

add_lg_lib(laser_guidance_bridges)
target_include_directories(laser_guidance_bridges PRIVATE
  ${rclcpp_INCLUDE_DIRS} ${visualization_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(laser_guidance_bridges PUBLIC
  laser_guidance_support laser_guidance_tracking
  ${rclcpp_LIBRARIES} ${visualization_msgs_LIBRARIES} ${std_msgs_LIBRARIES}
  nlohmann_json::nlohmann_json)

add_lg_lib(laser_guidance_runtime)
target_link_libraries(laser_guidance_runtime PUBLIC
  laser_guidance_support laser_guidance_vision laser_guidance_tracking
  laser_guidance_guidance laser_guidance_bridges)

add_library(rmcs_laser_guidance_core INTERFACE)
target_link_libraries(rmcs_laser_guidance_core INTERFACE
  laser_guidance_support laser_guidance_vision laser_guidance_tracking
  laser_guidance_guidance laser_guidance_bridges laser_guidance_runtime)

# ---- libusb claim shim (LD_PRELOAD, 修复 MVS SDK U3V 控制传输不 claim 接口) ----
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBUSB1 QUIET IMPORTED_TARGET libusb-1.0)
endif()
if(LIBUSB1_FOUND)
  add_library(libusb_claim_shim SHARED src/io/libusb_claim_shim.cpp)
  target_include_directories(libusb_claim_shim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src ${LIBUSB1_INCLUDE_DIRS})
  target_link_libraries(libusb_claim_shim PRIVATE dl)
  target_compile_features(libusb_claim_shim PRIVATE cxx_std_23)
else()
  message(WARNING "libusb-1.0 未找到，跳过 libusb_claim_shim（不影响主流程）")
endif()

# ---- RPATH assignments -------------------------------------------------------
add_runtime_rpaths(laser_guidance_guidance)
add_runtime_rpaths(laser_guidance_runtime)

add_custom_target(ft4222_runtime_link ALL
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    "${FT4222_LIBRARY}"
    "${CMAKE_BINARY_DIR}/libft4222.so"
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    "${FT4222_SO_NAME}"
    "${FT4222_RUNTIME_DIR}/libft4222.so"
  VERBATIM)

# ---- tools -------------------------------------------------------------------
enable_testing()

add_library(laser_guidance_calibration STATIC tools/calib/rotation_calibrator.cpp)
target_include_directories(laser_guidance_calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
target_link_libraries(laser_guidance_calibration PUBLIC
  rmcs_laser_guidance_core Eigen3::Eigen Ceres::ceres)
add_runtime_rpaths(laser_guidance_calibration)

file(GLOB TOOL_SOURCES CONFIGURE_DEPENDS tools/*.cpp)
foreach(_src ${TOOL_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(tool_${_name} ${_src})
  target_include_directories(tool_${_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tools)
  target_link_libraries(tool_${_name} PRIVATE rmcs_laser_guidance_core)
  if(_name STREQUAL "calib_solve")
    target_link_libraries(tool_${_name} PRIVATE laser_guidance_calibration)
  endif()
  if(_name STREQUAL "galvo_smoke" OR _name STREQUAL "dac8568_smoke")
    target_link_libraries(tool_${_name} PRIVATE ft4222::ft4222)
  endif()
  if(_name STREQUAL "cam_params_dump" OR _name STREQUAL "cam_grab")
    target_link_libraries(tool_${_name} PRIVATE hikcamera)
  endif()
  if(_name STREQUAL "cam_list")
    target_include_directories(tool_${_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vendor/hikcamera/src/sdk/include)
    target_link_libraries(tool_${_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/vendor/hikcamera/src/sdk/lib/libMvCameraControl.so)
  endif()
  add_runtime_rpaths(tool_${_name})
endforeach()

# ---- tests -------------------------------------------------------------------
file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS tests/*_test.cpp)
foreach(_src ${TEST_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(${_name} ${_src})
  target_include_directories(${_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tools ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  if(_src MATCHES "tests/runtime/")
    target_link_libraries(${_name} PRIVATE
      laser_guidance_support laser_guidance_vision laser_guidance_tracking
      laser_guidance_guidance laser_guidance_bridges laser_guidance_runtime)
  else()
    target_link_libraries(${_name} PRIVATE rmcs_laser_guidance_core)
  endif()
  if(_src MATCHES "tests/tools/")
    target_link_libraries(${_name} PRIVATE laser_guidance_calibration)
  endif()
  if(_name STREQUAL "calib_solve_cli_test")
    target_compile_definitions(${_name} PRIVATE CALIB_SOLVE_PATH="$<TARGET_FILE:tool_calib_solve>")
    add_dependencies(${_name} tool_calib_solve)
  endif()
  add_runtime_rpaths(${_name})
  add_test(NAME ${_name} COMMAND ${_name})
  set_property(TEST ${_name} PROPERTY ENVIRONMENT
    "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}:${FT4222_RUNTIME_DIR}:${HIKCAMERA_RUNTIME_DIR}:$ENV{LD_LIBRARY_PATH}")
  set_property(TEST ${_name} PROPERTY WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endforeach()
