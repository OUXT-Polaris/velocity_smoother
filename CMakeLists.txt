cmake_minimum_required(VERSION 2.8.3)
project(velocity_smoother)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
)

generate_dynamic_reconfigure_options(
  cfg/VelocitySmoother.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES velocity_smoother
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(velocity_smoother src/velocity_smoother.cpp)
target_link_libraries(velocity_smoother ${catkin_LIBRARIES})
add_dependencies(velocity_smoother ${catkin_EXPORTED_TARGETS})

add_executable(velocity_smoother_node src/velocity_smoother_node.cpp)
target_link_libraries(velocity_smoother_node ${catkin_LIBRARIES} velocity_smoother)
add_dependencies(velocity_smoother_node velocity_smoother ${catkin_EXPORTED_TARGETS})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library/executable
install(TARGETS velocity_smoother velocity_smoother_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)