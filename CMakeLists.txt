cmake_minimum_required(VERSION 3.1)
project(ORB_SLAM3)

# ---------------------------------------------------------
# Set the build type to Release
# ---------------------------------------------------------
set(CMAKE_BUILD_TYPE Release)

# ---------------------------------------------------------
# Use C++11 standard (ORB-SLAM3 requires at least C++11)
# ---------------------------------------------------------
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ---------------------------------------------------------
# Find required packages: OpenCV and Pangolin
# ---------------------------------------------------------
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)

# ---------------------------------------------------------
# Include directories
# ---------------------------------------------------------
include_directories(
    ${PROJECT_SOURCE_DIR}                             # Root of the project
    ${PROJECT_SOURCE_DIR}/include                     # Project headers
    ${OpenCV_INCLUDE_DIRS}                            # OpenCV includes
    ${Pangolin_INCLUDE_DIRS}                          # Pangolin includes
    ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/include
)

# ---------------------------------------------------------
# Build the ORB_SLAM3 library
# ---------------------------------------------------------
# Collect all source files in the src/ directory.
file(GLOB_RECURSE ORB_SLAM3_SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.cc"
)

# Create a shared library called ORB_SLAM3_LIB from the collected sources.
add_library(ORB_SLAM3_LIB SHARED ${ORB_SLAM3_SOURCE_FILES})

target_link_libraries(ORB_SLAM3_LIB
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    # Add additional libraries here if needed, e.g. DBow2, g2o, etc.
)

# ---------------------------------------------------------
# Set runtime output directory (absolute path)
# ---------------------------------------------------------
# All executables will be placed in <repo_root>/bin.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# ---------------------------------------------------------
# Build the rgbd_kinect example (using libfreenect and pthread)
# ---------------------------------------------------------
# The source file is located at: <repo_root>/Examples/RGB-D/rgbd_kinect.cc
add_executable(rgbd_kinect ${PROJECT_SOURCE_DIR}/Examples/RGB-D/rgbd_kinect.cc)

target_link_libraries(rgbd_kinect
    ORB_SLAM3_LIB
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    freenect         # Make sure libfreenect is installed
    pthread
)

# ---------------------------------------------------------
# Install targets and configuration files
# ---------------------------------------------------------
# Install the rgbd_kinect executable into <repo_root>/bin
install(TARGETS rgbd_kinect
    RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/bin
)

# Install the KinectV1.yaml file into <repo_root>/bin (or another folder of your choice)
install(FILES ${PROJECT_SOURCE_DIR}/Examples/RGB-D/KinectV1.yaml
    DESTINATION ${CMAKE_SOURCE_DIR}/bin
)

# ---------------------------------------------------------
# For ROS2 integration, register this package with ament
# ---------------------------------------------------------
ament_package()


