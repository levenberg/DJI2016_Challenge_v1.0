cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_read_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  #opencv2
  roscpp
  sensor_msgs
  std_msgs
  dji_sdk
)
catkin_package()
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    MESSAGE("openMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
include_directories(
include
  include/AprilTags
  include/DJI
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3)

#add_message_files(
#FILES
#DetectionPoints.msg
#)
# file(GLOB SOURCE_FILES "src/AprilTags/*.cc")
# 
# add_library(apriltags STATIC ${SOURCE_FILES} )#Seems SHARED libraries are not fine with the project.
#  
 link_directories(${PROJECT_SOURCE_DIR}/lib)
link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/lib)
add_executable(dji_sdk_read_cam src/nv_cam.cpp src/apriltagdetector.cpp  )
#add_executable(track_cam src/track_cam.cpp)

add_dependencies(dji_sdk_read_cam dji_sdk_generate_messages_cpp)


#link_directories(${PROJECT_SOURCE_DIR}/lib/arm)
#link_directories(/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_read_cam/lib/arm)


target_link_libraries(
  dji_sdk_read_cam
  apriltags
  dcam

  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  
)
# link_libraries(apriltags)
# target_link_libraries(
#  #   track_cam
#     dcam
#     ${catkin_LIBRARIES})

