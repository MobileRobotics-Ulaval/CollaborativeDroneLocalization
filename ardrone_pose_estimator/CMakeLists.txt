cmake_minimum_required(VERSION 2.4.6)
project(ardrone_pose_estimator)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  geometry_msgs
  std_msgs
  ardrone_autonomy
  tf
)

find_package(Eigen REQUIRED)


# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/ardrone_pose_estimator.cfg)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  dynamic_reconfigure 
  geometry_msgs 
  std_msgs
  ardrone_autonomy
  tf
  DEPENDS eigen
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# make the executable
add_executable(ardrone_pose_estimator
  cfg/ardrone_pose_estimator.cfg
  include/ardrone_pose_estimator/ardrone_pose_estimator_node.h
  src/ardrone_pose_estimator_node.cpp
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(ardrone_pose_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_dependencies(ardrone_pose_estimator ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencf)

# link the executable to the necessary libs
target_link_libraries(ardrone_pose_estimator
   yaml-cpp
   ${catkin_LIBRARIES}
 )
