cmake_minimum_required(VERSION 3.0.2)
project(uav_water_sim)

# find_package(catkin REQUIRED COMPONENTS
#   cv_bridge
#   image_transport
#   roscpp
#   std_msgs
# )

# catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
# )

find_package(OpenCV REQUIRED)

include_directories(
  include
  # ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_executable(uav_water_sim src/main.cpp src/VisionBrain.cpp src/PoolDetector.cpp src/CheckerboardDetector.cpp)
# add_dependencies(uav_water_sim ${uav_water_sim_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(uav_water_sim ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(uav_water_sim_vision src/visionMain.cpp src/PoolDetector.cpp src/CheckerboardDetector.cpp)
target_link_libraries(uav_water_sim_vision ${OpenCV_LIBRARIES})