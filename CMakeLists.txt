cmake_minimum_required(VERSION 2.8.3)
project(seline)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  pcl_ros
  pcl_conversions
)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS system)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES seline
#  CATKIN_DEPENDS other_catkin_pkg
   DEPENDS system_lib Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(seline src/seline.cc)
target_link_libraries(seline ${catkin_LIBRARIES})
add_dependencies(seline ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
