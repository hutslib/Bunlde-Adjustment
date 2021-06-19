cmake_minimum_required(VERSION 2.8)

project(bundle_adjustment)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O0 -std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(CSparse REQUIRED)


include_directories(3rdparty ${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

add_library(bal_common common/common.cpp ceres/BAReprojectionError.cpp)
add_executable(bundle_adjustment_ceres ceres/bundle_adjustment.cpp)

target_link_libraries(bundle_adjustment_ceres ${CERES_LIBRARIES} bal_common)
