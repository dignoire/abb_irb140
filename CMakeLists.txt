cmake_minimum_required(VERSION 2.8.3)
project(irb140)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs kdl_parser rtt_ros_kdl_tools)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(OROCOS-RTT)

if(OROCOS-RTT_FOUND)
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
    orocos_use_package(rtt_ros_kdl_tools)
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})

add_executable(irb140_kdl_node src/irb140_kdl.cpp)

target_link_libraries(irb140_kdl_node
  ${catkin_LIBRARIES}
${USE_OROCOS_LIBRARIES}
)
