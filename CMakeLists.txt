cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  rosgraph_msgs
)

catkin_package(

)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nautonomous_tf_node src/nautonomous_tf_node.cpp)

target_link_libraries(nautonomous_tf_node ${catkin_LIBRARIES})

