cmake_minimum_required(VERSION 3.12.0)
project(wolf_unitree_interface)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  realtime_tools
  wolf_hardware_interface
)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

catkin_package(
 CATKIN_DEPENDS
   controller_manager
   realtime_tools
   wolf_hardware_interface
 LIBRARIES unitree_hal
)

##########
# Build ##
##########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(unitree_hal src/unitree_hal.cpp)
target_link_libraries(unitree_hal ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(unitree_ros_node
    src/unitree_robot_hw.cpp
    src/unitree_ros_control.cpp
    src/unitree_ros_node.cpp)
target_link_libraries(unitree_ros_node ${catkin_LIBRARIES} unitree_hal)

#############
## Install ##
#############

install(
  TARGETS unitree_hal unitree_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)