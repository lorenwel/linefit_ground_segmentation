cmake_minimum_required(VERSION 3.8)
project(linefit_ground_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  PCL
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES} # it fails to find vtk without this
)

ament_auto_package()