cmake_minimum_required(VERSION 2.8.3)
project(pviz)

find_package(catkin REQUIRED COMPONENTS
  kdl
  kdl_parser
  roscpp
  tf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pviz
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pviz src/pviz.cpp)
add_dependencies(pviz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pviz ${catkin_LIBRARIES})

add_executable(pviz_example src/pviz_example.cpp)
add_dependencies(pviz_example pviz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pviz_example pviz ${catkin_LIBRARIES})
