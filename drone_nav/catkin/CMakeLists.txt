cmake_minimum_required(VERSION 2.4.6)
project(drone_nav)
#add_definitions(-std=c++0x)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  ardrone_autonomy
)

find_package(Eigen REQUIRED)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  geometry_msgs 
  std_msgs
  ardrone_autonomy
  DEPENDS eigen
)


# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# make the executable
add_executable(dronenav 
src/main.cpp
src/drone_nav_node.cpp
include/drone_nav/drone_nav_node.h
src/autonomous_control.cpp
include/drone_nav/autonomous_control.h
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
#_dependencies(dronenav ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_dependencies(dronenav ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencf)

# link the executable to the necessary libs
target_link_libraries(dronenav
   yaml-cpp
   ${catkin_LIBRARIES}
 )
