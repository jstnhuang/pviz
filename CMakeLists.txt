cmake_minimum_required(VERSION 2.8.3)
project(pviz)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kdl_parser
  roscpp
  tf
  visualization_msgs
)
find_package(orocos_kdl)

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

add_library(pviz_leatherman_utils src/leatherman_utils.cpp)
add_dependencies(pviz_leatherman_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pviz_leatherman_utils ${catkin_LIBRARIES})

add_library(pviz_leatherman_viz src/leatherman_viz.cpp)
add_dependencies(pviz_leatherman_viz pviz_leatherman_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pviz_leatherman_viz pviz_leatherman_utils ${catkin_LIBRARIES})

add_library(pviz src/pviz.cpp)
add_dependencies(pviz pviz_leatherman_utils pviz_leatherman_viz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pviz pviz_leatherman_utils pviz_leatherman_viz ${catkin_LIBRARIES})

add_executable(pviz_example src/pviz_example.cpp)
add_dependencies(pviz_example pviz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pviz_example pviz ${catkin_LIBRARIES})
