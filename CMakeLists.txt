cmake_minimum_required(VERSION 2.8.3)
project(imu_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system timer)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)
catkin_package(
)
include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

add_library(imu_module
   src/${PROJECT_NAME}/main.cpp
   src/${PROJECT_NAME}/RMG146.cpp
   src/${PROJECT_NAME}/MadgwickAHRS.cpp
)

add_executable(imu_module_node
  src/${PROJECT_NAME}/main.cpp
  src/${PROJECT_NAME}/RMG146.cpp
  src/${PROJECT_NAME}/MadgwickAHRS.cpp
)
target_link_libraries(imu_module_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math ")
