cmake_minimum_required(VERSION 2.8.3)
project(course_made_good)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  marine_msgs
  geographic_msgs
  project11
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES range_and_bearing
#  CATKIN_DEPENDS std_msgs sensor_msgs project11 roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/course_made_good_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

