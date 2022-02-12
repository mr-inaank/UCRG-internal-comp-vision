cmake_minimum_required(VERSION 3.0.2)
project(uav_water_detection)

# find_package(catkin REQUIRED COMPONENTS
#   cv_bridge
#   image_transport
#   roscpp
#   std_msgs
# )

find_package(OpenCV REQUIRED)

# catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
# )

include_directories(
  include
  # ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_executable(uav_water_detection src/main.cpp src/VisionBrain.cpp src/PoolDetector.cpp src/CheckerboardDetector.cpp)
# add_dependencies(uav_water_detection ${uav_water_detection_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(uav_water_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(uav_water_detection_vision src/VisionMain.cpp src/PoolDetector.cpp src/CheckerboardDetector.cpp)
target_link_libraries(uav_water_detection_vision ${OpenCV_LIBRARIES})
add_executable(colorPicker src/colorPicker.cpp)
target_link_libraries(colorPicker ${OpenCV_LIBRARIES})