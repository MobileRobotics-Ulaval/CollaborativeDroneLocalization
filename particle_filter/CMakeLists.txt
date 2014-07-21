cmake_minimum_required(VERSION 2.4.6)
project(particle_filter)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  geometry_msgs
  dot_finder
)

# since we need eigen search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Eigen REQUIRED)


# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/ParticleFilter.cfg)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  dynamic_reconfigure 
  geometry_msgs 
  dot_finder
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
add_executable(particle_filter
  cfg/ParticleFilter.cfg
  mesh/quadrotor_4.dae
  mesh/quadrotor_3.dae


  include/particle_filter/particle_filter_node.h 
  include/particle_filter/mutual_pose_estimation.h
  include/particle_filter/pose_filter.h

  src/particle_filter_node.cpp 
  src/mutual_pose_estimation.cpp
  src/pose_filter.cpp
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(particle_filter ${catkin_EXPORTED_TARGETS}  tf ${PROJECT_NAME}_gencf)

# link the executable to the necessary libs
target_link_libraries(particle_filter
   yaml-cpp
   ${catkin_LIBRARIES}
 )
