cmake_minimum_required(VERSION 3.14)
project(linefit_ground_segmentation
	VERSION 1.0.0
	DESCRIPTION "linefit_ground_segmentation"
	LANGUAGES CXX
)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# find_package(Eigen3 REQUIRED)
include(cmake/CompilerOptions.cmake)
include(cmake/dependencies.cmake)
add_library(gm_lib STATIC)

target_sources(gm_lib PRIVATE linefit/segment.cpp linefit/ground_segmentation.cpp)
target_link_libraries(gm_lib PUBLIC Eigen3::Eigen)
set_global_target_properties(gm_lib)
