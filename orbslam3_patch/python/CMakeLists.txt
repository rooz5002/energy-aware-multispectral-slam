cmake_minimum_required(VERSION 3.15)
project(orbslam3_ms LANGUAGES CXX)

# Must match the flags ORB-SLAM3 itself is built with (Eigen alignment depends on -march).
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

set(ORBSLAM3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ORB_SLAM3" CACHE PATH "patched ORB-SLAM3 tree")
find_package(pybind11 REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 REQUIRED)

pybind11_add_module(orbslam3_ms orbslam3_ms.cc)
target_compile_definitions(orbslam3_ms PRIVATE MS_SLAM)
target_include_directories(orbslam3_ms PRIVATE
    ${ORBSLAM3_DIR} ${ORBSLAM3_DIR}/include ${ORBSLAM3_DIR}/include/CameraModels ${ORBSLAM3_DIR}/Thirdparty/Sophus
    ${ORBSLAM3_DIR}/headless ${EIGEN3_INCLUDE_DIR})
target_link_libraries(orbslam3_ms PRIVATE
    ${ORBSLAM3_DIR}/lib/libORB_SLAM3.so ${ORBSLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORBSLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS})
set_target_properties(orbslam3_ms PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../harness)
