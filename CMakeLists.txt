cmake_minimum_required(VERSION 2.8.3)
project(water_ext)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #positioning_system
  roscpp
  cv_bridge
  image_transport
  genmsg
  sensor_msgs
  std_msgs
  common_srv
  message_generation
  rospy
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  temp_range.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime common_srv
  DEPENDS
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  include
  ${OpenCV_INCLUDE_DIRS}
)


FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
