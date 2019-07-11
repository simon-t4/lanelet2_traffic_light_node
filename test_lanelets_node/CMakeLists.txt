cmake_minimum_required(VERSION 2.8.3)
project(test_lanelet_node)



set(LL_DIR /home/simon/work/catkin_ws/src/lanelet2)
set(AW_DIR /home/simon/Autoware)
set(autoware_msgs_DIR ${AW_DIR}/ros/install/autoware_msgs/share/autoware_msgs/cmake)

find_package(autoware_msgs REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
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

find_package(Boost COMPONENTS serialization system timer)
    

include_directories(
# include
/usr/include/eigen3
${LL_DIR}/lanelet2_core/include
${LL_DIR}/lanelet2_core/include/lanelet2_core
${LL_DIR}/lanelet2_io/include
${LL_DIR}/lanelet2_io/include/lanelet2_io
${LL_DIR}/lanelet2_projection/include
${LL_DIR}/lanelet2_projection/include/lanelet2_projection
${LL_DIR}/lanelet2_traffic_rules/include
${LL_DIR}/lanelet2_traffic_rules/include/lanelet2_traffic_rules
${LL_DIR}/lanelet2_routing/include
${LL_DIR}/lanelet2_routing/include/lanelet2_routing
${mrt_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${AW_DIR}/ros/install/autoware_msgs/include
)


link_directories(
${mrt_LIBRARY_DIRS}
)


file (GLOB_RECURSE ll_io_src ${LL_DIR}/lanelet2_io/src/*.cpp)
file (GLOB_RECURSE ll_core_src ${LL_DIR}/lanelet2_core/src/*.cpp)
file (GLOB_RECURSE ll_core_h ${LL_DIR}/lanelet2_core/include/*.h)
file (GLOB_RECURSE ll_tfr_src ${LL_DIR}/lanelet2_traffic_rules/src/*.cpp)
file (GLOB_RECURSE ll_tfr_h ${LL_DIR}/lanelet2_traffic_rules/include/*.h)
file (GLOB_RECURSE ll_rt_src ${LL_DIR}/lanelet2_routing/src/*.cpp)
file (GLOB_RECURSE ll_rt_h ${LL_DIR}/lanelet2_routing/include/*.h)

file (GLOB PUGI_DEFS /usr/include/pugi*.hpp)
set(PUGI_LIB /usr/lib/x86_64-linux-gnu/libpugixml.so)

add_executable(test_lanelet_node
  src/test_lanelet_node.cpp src/rosUTM.cpp
  ${LL_DIR}/lanelet2_projection/src/UTM.cpp
  ${ll_io_src}
  ${ll_core_src}
  ${ll_core_h}
  ${ll_tfr_src}
  ${ll_tfr_h}
  ${ll_rt_src}
  ${ll_rt_h}
  ${PUGI_DEFS}
  )
target_link_libraries(test_lanelet_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PUGI_LIB}
  ${Boost_LIBRARIES}
  )

add_executable(test_lr_node
  src/test_lr_node.cpp src/rosUTM.cpp
  ${LL_DIR}/lanelet2_projection/src/UTM.cpp
  ${ll_io_src}
  ${ll_core_src}
  ${ll_core_h}
  ${ll_tfr_src}
  ${ll_tfr_h}
  ${ll_rt_src}
  ${ll_rt_h}
  ${PUGI_DEFS}
  )
target_link_libraries(test_lr_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PUGI_LIB}
  ${Boost_LIBRARIES}
  )

