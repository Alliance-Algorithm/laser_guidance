cmake_minimum_required(VERSION 3.16)
project(rmcs_laser_guidance)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME "Enable ONNX Runtime model backend support." OFF)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to an ONNX Runtime installation root")
option(RMCS_LASER_GUIDANCE_WITH_TENSORRT "Enable TensorRT engine backend support." OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)
find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgcodecs imgproc videoio)
find_package(yaml-cpp REQUIRED)

include(GNUInstallDirs)

set(RMCS_LASER_GUIDANCE_USE_AMENT ${ament_cmake_FOUND})

if(RMCS_LASER_GUIDANCE_USE_AMENT)
  message(STATUS "Configuring rmcs_laser_guidance with ament_cmake")
else()
  message(STATUS "Configuring rmcs_laser_guidance in standalone CMake mode")
endif()

# ---- ONNX Runtime (optional) --------------------------------------------------
if(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
  find_path(
    RMCS_LASER_GUIDANCE_ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime/core/session/onnxruntime_cxx_api.h
    HINTS ${ONNXRUNTIME_ROOT} ENV ONNXRUNTIME_ROOT
    PATH_SUFFIXES include
  )
  find_library(
    RMCS_LASER_GUIDANCE_ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    HINTS ${ONNXRUNTIME_ROOT} ENV ONNXRUNTIME_ROOT
    PATH_SUFFIXES lib lib64
  )

  if(NOT RMCS_LASER_GUIDANCE_ONNXRUNTIME_INCLUDE_DIR OR NOT RMCS_LASER_GUIDANCE_ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR
      "RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=ON requires ONNX Runtime. "
      "Set ONNXRUNTIME_ROOT or export ONNXRUNTIME_ROOT to point at the installation root.")
  endif()

  add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
  set_target_properties(
    onnxruntime::onnxruntime
    PROPERTIES
      IMPORTED_LOCATION "${RMCS_LASER_GUIDANCE_ONNXRUNTIME_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${RMCS_LASER_GUIDANCE_ONNXRUNTIME_INCLUDE_DIR}"
  )

  message(STATUS "ONNX Runtime model backend enabled")
endif()

# ---- TensorRT (optional) ------------------------------------------------------
if(RMCS_LASER_GUIDANCE_WITH_TENSORRT)
  find_path(
    RMCS_LASER_GUIDANCE_TENSORRT_INCLUDE_DIR
    NAMES NvInfer.h NvOnnxParser.h
    PATHS /usr/include /usr/local/include
  )
  find_library(
    RMCS_LASER_GUIDANCE_TENSORRT_LIBRARY
    NAMES nvinfer
    PATHS /usr/lib /usr/local/lib
  )
  find_library(
    RMCS_LASER_GUIDANCE_TENSORRT_ONNX_PARSER_LIBRARY
    NAMES nvonnxparser
    PATHS /usr/lib /usr/local/lib
  )
  find_library(
    RMCS_LASER_GUIDANCE_TENSORRT_PLUGIN_LIBRARY
    NAMES nvinfer_plugin
    PATHS /usr/lib /usr/local/lib
  )

  if(NOT RMCS_LASER_GUIDANCE_TENSORRT_INCLUDE_DIR
     OR NOT RMCS_LASER_GUIDANCE_TENSORRT_LIBRARY
     OR NOT RMCS_LASER_GUIDANCE_TENSORRT_ONNX_PARSER_LIBRARY
     OR NOT RMCS_LASER_GUIDANCE_TENSORRT_PLUGIN_LIBRARY)
    message(FATAL_ERROR
      "RMCS_LASER_GUIDANCE_WITH_TENSORRT=ON requires TensorRT. "
      "Install tensorrt or set CMAKE_PREFIX_PATH to the installation root.")
  endif()

  add_library(TensorRT::nvinfer UNKNOWN IMPORTED)
  set_target_properties(TensorRT::nvinfer PROPERTIES
    IMPORTED_LOCATION "${RMCS_LASER_GUIDANCE_TENSORRT_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${RMCS_LASER_GUIDANCE_TENSORRT_INCLUDE_DIR}")

  add_library(TensorRT::nvonnxparser UNKNOWN IMPORTED)
  set_target_properties(TensorRT::nvonnxparser PROPERTIES
    IMPORTED_LOCATION "${RMCS_LASER_GUIDANCE_TENSORRT_ONNX_PARSER_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${RMCS_LASER_GUIDANCE_TENSORRT_INCLUDE_DIR}")

  add_library(TensorRT::nvinfer_plugin UNKNOWN IMPORTED)
  set_target_properties(TensorRT::nvinfer_plugin PROPERTIES
    IMPORTED_LOCATION "${RMCS_LASER_GUIDANCE_TENSORRT_PLUGIN_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${RMCS_LASER_GUIDANCE_TENSORRT_INCLUDE_DIR}")

  message(STATUS "TensorRT engine backend enabled")
endif()

# ---- default paths ------------------------------------------------------------
set(default_config_path "${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml")
set(default_sample_replay_path "${CMAKE_CURRENT_SOURCE_DIR}/test_data/sample_images")
set(default_video_session_root "${CMAKE_CURRENT_SOURCE_DIR}/videos")

# ---- core library -------------------------------------------------------------
file(GLOB RMCS_LASER_GUIDANCE_CORE_SOURCES src/*.cpp)
if(NOT RMCS_LASER_GUIDANCE_WITH_TENSORRT)
  list(REMOVE_ITEM RMCS_LASER_GUIDANCE_CORE_SOURCES src/tensorrt_engine.cpp)
endif()
add_library(${PROJECT_NAME}_core SHARED ${RMCS_LASER_GUIDANCE_CORE_SOURCES})

target_include_directories(
  ${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${OpenCV_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}_core PUBLIC yaml-cpp::yaml-cpp ${OpenCV_LIBS})

if(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
  target_link_libraries(${PROJECT_NAME}_core PRIVATE onnxruntime::onnxruntime)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=1)
endif()

if(RMCS_LASER_GUIDANCE_WITH_TENSORRT)
  target_link_libraries(${PROJECT_NAME}_core
    PRIVATE TensorRT::nvinfer TensorRT::nvonnxparser TensorRT::nvinfer_plugin)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC RMCS_LASER_GUIDANCE_WITH_TENSORRT=1)
endif()

# ---- shared helpers -----------------------------------------------------------
define_property(GLOBAL PROPERTY RMCS_LASER_GUIDANCE_EXAMPLE_TARGETS
  BRIEF_DOCS "Example targets" FULL_DOCS "Accumulated example executable names")

set_property(GLOBAL PROPERTY RMCS_LASER_GUIDANCE_EXAMPLE_TARGETS "")

function(rmcs_laser_guidance_add_default_paths target_name)
  target_compile_definitions(
    ${target_name}
    PRIVATE
      RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH="${default_config_path}"
      RMCS_LASER_GUIDANCE_DEFAULT_SAMPLE_REPLAY_PATH="${default_sample_replay_path}"
      RMCS_LASER_GUIDANCE_DEFAULT_VIDEO_SESSION_ROOT="${default_video_session_root}"
  )
endfunction()

function(rmcs_laser_guidance_add_example target_name source_path)
  add_executable(${target_name} ${source_path})
  rmcs_laser_guidance_add_default_paths(${target_name})
  target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${target_name} PRIVATE ${PROJECT_NAME}_core ${OpenCV_LIBS})
  set_property(GLOBAL APPEND PROPERTY
    RMCS_LASER_GUIDANCE_EXAMPLE_TARGETS ${target_name})
endfunction()

function(rmcs_laser_guidance_add_test target_name source_path)
  add_executable(${target_name} ${source_path})
  rmcs_laser_guidance_add_default_paths(${target_name})
  target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${target_name} PRIVATE ${PROJECT_NAME}_core ${OpenCV_LIBS})
  add_test(NAME ${target_name} COMMAND ${target_name})
endfunction()

# ---- examples -----------------------------------------------------------------
enable_testing()

set(RMCS_LASER_GUIDANCE_EXAMPLES
    example_offline_smoke              examples/offline_smoke.cpp
    example_v4l2_preview               examples/v4l2_preview.cpp
    example_v4l2_capture               examples/v4l2_capture.cpp
    example_v4l2_record_session        examples/v4l2_record_session.cpp
    example_export_training_frames     examples/export_training_frames.cpp
    example_replay_preview             examples/replay_preview.cpp
    example_detector_benchmark         examples/detector_benchmark.cpp
    example_model_infer                examples/model_infer.cpp
    example_transcode_recorded_session examples/transcode_recorded_session.cpp
    example_camera_calibration         examples/camera_calibration.cpp
)

list(LENGTH RMCS_LASER_GUIDANCE_EXAMPLES _num_pairs)
math(EXPR _last "${_num_pairs} - 1")
foreach(_i RANGE 0 ${_last} 2)
  math(EXPR _j "${_i} + 1")
  list(GET RMCS_LASER_GUIDANCE_EXAMPLES ${_i} _target)
  list(GET RMCS_LASER_GUIDANCE_EXAMPLES ${_j} _source)
  rmcs_laser_guidance_add_example(${_target} ${_source})
endforeach()

# ---- tests --------------------------------------------------------------------
set(RMCS_LASER_GUIDANCE_TESTS
    config_test              tests/config_test.cpp
    pipeline_test            tests/pipeline_test.cpp
    frame_format_test        tests/frame_format_test.cpp
    detector_test            tests/detector_test.cpp
    debug_renderer_test      tests/debug_renderer_test.cpp
    sample_images_test       tests/sample_images_test.cpp
    model_infer_test         tests/model_infer_test.cpp
    model_adapter_test       tests/model_adapter_test.cpp
    red_target_refiner_test  tests/red_target_refiner_test.cpp
    training_data_test       tests/training_data_test.cpp
    freshness_queue_test     tests/freshness_queue_test.cpp
    runtime_metrics_test     tests/runtime_metrics_test.cpp
    hit_state_test           tests/hit_state_test.cpp
    mock_runtime_test        tests/mock_runtime_test.cpp
)

list(LENGTH RMCS_LASER_GUIDANCE_TESTS _num_pairs)
math(EXPR _last "${_num_pairs} - 1")
foreach(_i RANGE 0 ${_last} 2)
  math(EXPR _j "${_i} + 1")
  list(GET RMCS_LASER_GUIDANCE_TESTS ${_i} _target)
  list(GET RMCS_LASER_GUIDANCE_TESTS ${_j} _source)
  rmcs_laser_guidance_add_test(${_target} ${_source})
endforeach()

# ---- install ------------------------------------------------------------------
get_property(_examples GLOBAL PROPERTY RMCS_LASER_GUIDANCE_EXAMPLE_TARGETS)

install(
  TARGETS ${PROJECT_NAME}_core ${_examples}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

foreach(_dir config models test_data docs)
  install(DIRECTORY ${_dir} DESTINATION share/${PROJECT_NAME})
endforeach()

install(
  FILES README.md AGENTS.md plan.md
  DESTINATION share/${PROJECT_NAME}
)

if(RMCS_LASER_GUIDANCE_USE_AMENT)
  ament_export_include_directories(${CMAKE_INSTALL_INCLUDEDIR})
  ament_export_dependencies(OpenCV yaml-cpp)
  ament_package()
endif()
