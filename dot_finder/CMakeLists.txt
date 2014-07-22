cmake_minimum_required(VERSION 2.4.6)
project(dot_finder)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  dynamic_reconfigure
  geometry_msgs
  tf
  message_generation
)

# Add custom message
add_message_files(
  FILES
  DuoDot.msg
)

# Add dependencies of custom message
generate_messages(   
  DEPENDENCIES  
  geometry_msgs  
  std_msgs
)  

# since we need eigen search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Eigen REQUIRED)

# Add opencv
find_package(OpenCV REQUIRED)

# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/DotFinder.cfg)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge image_transport dynamic_reconfigure geometry_msgs
  DEPENDS eigen OpenCV
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# make the executable
add_executable(dot_finder 
  cfg/DotFinder.cfg

  include/dot_finder/dot_finder_node.h
  include/dot_finder/dot_detector.h
  include/dot_finder/visualization.h

  src/dot_finder_node.cpp 
  src/dot_detector.cpp
  src/visualization.cpp
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(dot_finder ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(dot_finder ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencf)

# link the executable to the necessary libs
target_link_libraries(dot_finder
   yaml-cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
