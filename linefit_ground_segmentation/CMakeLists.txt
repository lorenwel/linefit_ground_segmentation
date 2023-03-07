cmake_minimum_required(VERSION 2.8.3)
project(linefit_ground_segmentation)

find_package(catkin)

find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)

IF(catkin_FOUND)

    catkin_package(
        INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME}
        DEPENDS PCL
    )

ENDIF()

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/ground_segmentation.cc
    src/segment.cc
    src/bin.cc
    src/viewer.cc
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

add_definitions(${PCL_DEFINITIONS})


IF(catkin_FOUND)

    install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

    install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
    )

ENDIF()


#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
