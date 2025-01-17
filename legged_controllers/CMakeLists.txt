cmake_minimum_required(VERSION 3.10)
project(legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_path(RBDL_INCLUDE_DIR rbdl/rbdl.h
    HINTS
    $ENV{HOME}/local/include
    $ENV{RBDL_PATH}/src
    $ENV{RBDL_PATH}/include
    $ENV{RBDL_INCLUDE_PATH}
    /usr/local/include
    /usr/include
)

find_library(RBDL_LIBRARY rbdl
    PATHS
    $ENV{HOME}/local/lib
    $ENV{HOME}/local/lib/x86_64-linux-gnu
    $ENV{RBDL_PATH}/lib
    $ENV{RBDL_LIBRARY_PATH}
    /usr/local/lib
    /usr/local/lib/x86_64-linux-gnu
    /usr/lib
    /usr/lib/x86_64-linux-gnu
)
find_path (RBDL_URDFREADER_INCLUDE_DIR rbdl/addons/urdfreader/urdfreader.h
    HINTS
    $ENV{HOME}/local/include
    $ENV{RBDL_PATH}/src
    $ENV{RBDL_PATH}/include
    $ENV{RBDL_INCLUDE_PATH}
    /usr/local/include
    /usr/include
)

find_library (RBDL_URDFREADER_LIBRARY rbdl_urdfreader
    PATHS
    $ENV{HOME}/local/lib
    $ENV{HOME}/local/lib/x86_64-linux-gnu
    $ENV{RBDL_PATH}
    $ENV{RBDL_LIBRARY_PATH}
    /usr/local/lib
    /usr/local/lib/x86_64-linux-gnu
    /usr/lib
    /usr/lib/x86_64-linux-gnu
)
## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        dynamic_reconfigure
        rospy
        std_msgs
        message_generation
        grid_map_ros
        )

find_package(PkgConfig REQUIRED)
find_package (RBDL COMPONENTS URDFREADER REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(  
        cfg/Tutorials.cfg
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        dynamic_reconfigure
        rospy
        std_msgs
        message_generation
        grid_map_ros
)


###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${RBDL_INCLUDE_DIR}
        ${RBDL_URDFREADER_INCLUDE_DIR}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)


## Declare a cpp library
add_library(${PROJECT_NAME}
        src/LeggedController.cpp
        src/utilities.cpp
        src/synchronized_module/PlanarTerrainReceiver.cpp
        src/visualization/FootPlacementVisualization.cpp
        src/visualization/SphereVisualization.cpp
        )
        
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# Target trajectories publish node
add_executable(legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )

add_dependencies(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
install(FILES legged_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
