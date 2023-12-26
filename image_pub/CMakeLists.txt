cmake_minimum_required(VERSION 2.8.3)

project(image_pub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${T507_SDK_INCLUDE_DIRS}
	${CMAKE_CURRENT_LIST_DIR}/include
)
link_directories(
	${T507_SDK_LINK_DIRS}
	${CMAKE_CURRENT_LIST_DIR}/lib
)

add_executable(image_publisher src/image_publisher.cpp)
target_link_libraries(image_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(image_publisher TTARtspPusher DjiRtspImageSource)