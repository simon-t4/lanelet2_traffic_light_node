cmake_minimum_required(VERSION 2.8.3)
project(lanelet_tl_node)

set(AW_DIR /home/simon/Autoware)

find_package(autoware_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geodesy
#  message_generation
)


## from lanelet2/lanelet2_examples CMakeLists.txt
set (MRT_PKG_VERSION 2.2.1)

find_package(
  mrt_cmake_modules REQUIRED
  lanelet2 REQUIRED
  )


include(UseMrtStdCompilerFlags)
include(UseMrtAutoTarget)
include(GatherDeps)


find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})
mrt_add_to_ide(README.md .gitlab-ci.yml)


find_package(Eigen3 REQUIRED)

add_definitions(-DPKG_DIR="${CMAKE_CURRENT_LIST_DIR}")


catkin_package(
 INCLUDE_DIRS
CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geodesy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


find_package (GeographicLib 1.34 REQUIRED)

#find_package (PugiXML REQUIRED)

include_directories (${GeographicLib_INCLUDE_DIRS})
    add_definitions (${GeographicLib_DEFINITIONS})
    
find_package(Boost COMPONENTS serialization system)

include_directories(
include
/usr/include/eigen3
${mrt_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${AW_DIR}/ros/install/autoware_msgs/include
src
)


link_directories(
${mrt_LIBRARY_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geodesy
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  lanelet2_msgs
)

catkin_package(
  #  INCLUDES_DIR include
  LIBRARIES lanelet2_extension_lib
  CATKIN_DEPENDS roscpp geodesy lanelet2_core lanelet2_io lanelet2_maps lanelet2_projection lanelet2_routing lanelet2_traffic_rules lanelet2_validation lanelet2_msgs
)

include_directories(
  include
  ${GeographLib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(lanelet_tl_node
  src/lanelet_tl_node.cpp
  )
target_link_libraries(lanelet_tl_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${Boost_LIBRARIES}
  lanelet2_extension_lib
  )
target_include_directories(lanelet_tl_node
  PRIVATE /home/simon/work/catkin_ws/src/lanelet2_extension/include
  )

