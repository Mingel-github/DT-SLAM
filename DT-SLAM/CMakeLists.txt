cmake_minimum_required(VERSION 2.8)
project(DT_SLAM)

# Fix Eigen 3.4 + g2o alignment mismatch (double-free crash)
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# DT-SLAM: 使用C++14（Pangolin sigslot需要std::decay_t等C++14特性）
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DCOMPILEDWITHC11)
message(STATUS "Using C++14.")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
      endif()
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
# The bundled FindEigen3.cmake only exposes EIGEN3_FOUND. Pangolin's package
# config uses the canonical Eigen3_FOUND spelling when resolving dependencies.
if(EIGEN3_FOUND AND NOT Eigen3_FOUND)
    set(Eigen3_FOUND TRUE)
endif()
# Fix: newer cmake may not create Eigen3::Eigen target from find_package
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
find_package(Pangolin REQUIRED)

set(ONNXRUNTIME_ROOT
    "${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime"
    CACHE PATH "ONNX Runtime C/C++ package root")
if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h" OR
   NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
    message(FATAL_ERROR
        "Invalid ONNXRUNTIME_ROOT=${ONNXRUNTIME_ROOT}: expected include/ and lib/libonnxruntime.so")
endif()
message(STATUS "ONNX Runtime root: ${ONNXRUNTIME_ROOT}")

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${ONNXRUNTIME_ROOT}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/YOLOSegment.cc
src/GeometricDynamicDetector.cc
src/JiGeometryBaseline.cc
src/RGBDInputRectifier.cc
src/SInStyleDynamicDetector.cc
src/SInStyleInitialRegionClusterer.cc
src/SInStyleGradientRegionSplitter.cc
src/SInStylePlaneEdgeRegionSplitter.cc
src/SInStyleRAGRegionMerger.cc
src/SInStyleDenseFlowResidualEstimator.cc
src/SInStyleRegionDynamicClassifier.cc
src/SInStyleDepthFilter.cc
)

find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${CMAKE_THREAD_LIBS_INIT}
${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

add_executable(semantic_review_export
Examples/RGB-D/semantic_review_export.cc)
target_link_libraries(semantic_review_export ${PROJECT_NAME})

add_executable(geometric_warp_test
Examples/RGB-D/geometric_warp_test.cc)
target_link_libraries(geometric_warp_test ${PROJECT_NAME})

add_executable(depth_region_partition_audit
Examples/RGB-D/depth_region_partition_audit.cc)
target_link_libraries(depth_region_partition_audit ${PROJECT_NAME})

add_executable(box_region_partition_audit
Examples/RGB-D/box_region_partition_audit.cc)
target_link_libraries(box_region_partition_audit ${PROJECT_NAME})

add_executable(ji_geometry_test
Examples/RGB-D/ji_geometry_test.cc)
target_link_libraries(ji_geometry_test ${PROJECT_NAME})

add_executable(sin_style_shadow_test
Examples/RGB-D/sin_style_shadow_test.cc)
target_link_libraries(sin_style_shadow_test ${PROJECT_NAME})

add_executable(sin_style_dense_flow_replay_test
Examples/RGB-D/sin_style_dense_flow_replay_test.cc)
target_link_libraries(sin_style_dense_flow_replay_test ${PROJECT_NAME})

add_executable(sin_style_dense_flow_native_parity_test
Examples/RGB-D/sin_style_dense_flow_native_parity_test.cc)
target_link_libraries(sin_style_dense_flow_native_parity_test ${PROJECT_NAME})

add_executable(sin_style_region_dynamic_parity_test
Examples/RGB-D/sin_style_region_dynamic_parity_test.cc)
target_link_libraries(sin_style_region_dynamic_parity_test ${PROJECT_NAME})

add_executable(sin_style_depth_filter_test
Examples/RGB-D/sin_style_depth_filter_test.cc)
target_link_libraries(sin_style_depth_filter_test ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})
