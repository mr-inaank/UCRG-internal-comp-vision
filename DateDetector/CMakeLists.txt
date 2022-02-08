# CMakeLists.txt

cmake_minimum_required(VERSION "3.17")
project(DateDetector)

# set OpenCV_DIR variable equal to the path to the cmake
# files within the previously installed opencv program
set(OpenCV_DIR /usr/local/include/opencv4)

# Tell compiler to use C++ 14 features which is needed because
# Clang version is often behind in the XCode installation
set(CMAKE_CXX_STANDARD 17)

# configure the necessary common CMake environment variables
# needed to include and link the OpenCV program into this
# demo project, namely OpenCV_INCLUDE_DIRS and OpenCV_LIBS
find_package( OpenCV REQUIRED )

# tell the build to include the headers from OpenCV
include_directories( ${OpenCV_INCLUDE_DIRS}, classes )

# specify the executable target to be built
add_executable(dateDetector dateDetector.cpp)
add_executable(dateDetectorClass classes/DateDetector.cpp)

# tell it to link the executable target against OpenCV
target_link_libraries(dateDetector ${OpenCV_LIBS} )
target_link_libraries(dateDetectorClass ${OpenCV_LIBS} )
