cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  image_transport
  cv_bridge
  g2o
  dbow2
)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Cholmod REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
    cv_bridge
    sensor_msgs
    image_transport
    g2o
    dbow2
  DEPENDS
    Cholmod
    Eigen3
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    orb_slam
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

##################### Build libs ######################
add_library(orb_slam
  src/wrapper.cpp
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FramePublisher.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapPublisher.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
)

target_link_libraries(orb_slam
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
)

##################### Build exec ######################
add_executable(orb_slam_node
  src/main.cc
)
target_link_libraries(orb_slam_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  orb_slam
)
