cmake_minimum_required(VERSION 3.0.2)
project(antobot_heading)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    roscpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    tf2_geometry_msgs
    tf2
    geonav_transform
    antobot_platform_msgs
    antobot_devices_msgs
    geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS 
        roscpp
        std_msgs
        std_srvs
        nav_msgs
        sensor_msgs
        tf2_geometry_msgs
        tf2
        geonav_transform
        antobot_platform_msgs
        antobot_devices_msgs
        geometry_msgs
)

###########
## Build ##
###########

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(heading
  src/heading.cpp
)

add_dependencies(heading ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(heading_node src/heading_node.cpp)

add_dependencies(heading_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(heading_node
  heading
  ${catkin_LIBRARIES}
)

