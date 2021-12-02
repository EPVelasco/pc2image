cmake_minimum_required(VERSION 3.0.2)
project(pc2image)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(

  CATKIN_DEPENDS
  DEPENDS PCL

)

include_directories(
 #include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_executable(pc2image_node src/pc2image.cpp)
target_link_libraries(pc2image_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)
