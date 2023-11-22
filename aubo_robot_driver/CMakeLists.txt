cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_CXX_STANDARD 17)
cmake_policy(SET CMP0097 NEW)
project(aubo_robot_driver)

add_definitions( -DROS_BUILD )

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(ENV{CPM_SOURCE_CACHE} $ENV{HOME}/.cache/CPM)


# 下载 aubo_sdk 发布包(二进制方式引入)
    if(NOT aubo_sdk_POPULATED)
        include(FetchContent)
        FetchContent_Declare(
            aubo_sdk
            #URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.19.2-Linux_x86_64+a90aefb.zip"
            URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.22.1-Linux_x86_64+2ed0c14.zip"
        )
        FetchContent_Populate(aubo_sdk)
    endif()
    set(aubo_sdk_DIR ${aubo_sdk_SOURCE_DIR}/lib/cmake/aubo_sdk)
message("aubo_sdk_DIR" ${aubo_sdk_SOURCE_DIR}/lib/cmake/aubo_sdk)
# 下载 aubo_sdk 发布包(源码方式引入)
#CPMAddPackage(
#          NAME aubo_sdk
#          GIT_REPOSITORY "http://git.aubo-robotics.cn:8001/arcs/aubo_sdk.git"
#          GIT_TAG master
#        )


message("aubo_sdk_SOURCE_DIR ${aubo_sdk_SOURCE_DIR}")

find_package(aubo_sdk REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    control_msgs
    controller_manager
    controller_manager_msgs
    geometry_msgs
    hardware_interface
    industrial_robot_status_interface
    pluginlib
    roscpp
    scaled_joint_trajectory_controller
    sensor_msgs
    speed_scaling_interface
    speed_scaling_state_controller
    std_srvs
    tf
    tf2_geometry_msgs
    tf2_msgs
    pass_through_controllers
    kdl_parser
    aubo_dashboard_msgs
    aubo_msgs
)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    actionlib
    control_msgs
    controller_manager
    controller_manager_msgs
    geometry_msgs
    hardware_interface
    kdl_parser
    pass_through_controllers
    pluginlib
    roscpp
    scaled_joint_trajectory_controller
    sensor_msgs
    speed_scaling_interface
    speed_scaling_state_controller
    std_srvs
    tf
    tf2_geometry_msgs
    tf2_msgs
    trajectory_msgs
    aubo_dashboard_msgs
    aubo_msgs
  DEPENDS
    Boost
)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${aubo_sdk_SOURCE_DIR}/include
  
)


add_library(aubo_robot_driver_plugin
  src/aubo_hardware_interface.cpp
  src/aubo_robot_driver_node.cpp
)
target_link_libraries(aubo_robot_driver_plugin  PUBLIC ${catkin_LIBRARIES} aubo_sdk::aubo_sdk  Threads::Threads )
add_dependencies(aubo_robot_driver_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(aubo_robot_driver_node
  src/aubo_hardware_interface.cpp
  src/aubo_robot_driver_node.cpp
)
add_executable(aubo_client_node 
  src/aubo_client.cpp
  src/aubo_client_node.cpp
)

target_link_libraries(aubo_robot_driver_node PUBLIC ${catkin_LIBRARIES} aubo_sdk::aubo_sdk Threads::Threads )
add_dependencies(aubo_robot_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(aubo_client_node PUBLIC ${catkin_LIBRARIES} aubo_sdk::aubo_sdk Threads::Threads )
add_dependencies(aubo_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS aubo_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS aubo_robot_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

