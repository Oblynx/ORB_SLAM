cmake_minimum_required(VERSION 2.8.3)
project(akaze_features)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  cv_bridge
  sensor_msgs
  image_transport
  )

# ============================================================================ #
# Build Type. By default is release
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug, Release" FORCE)


add_library(${PROJECT_NAME}_lib 
  src/AKAZE.cpp
  src/fed.cpp
  src/nldiffusion_functions.cpp
  src/utils.cpp
  )
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  )

# Feature detector program
add_executable(${PROJECT_NAME}_detector src/akaze_features.cpp)
target_link_libraries(${PROJECT_NAME}_detector 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  )

# Image matching program
add_executable(${PROJECT_NAME}_match src/akaze_match.cpp)
target_link_libraries(${PROJECT_NAME}_match
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib)

# Program that compares AKAZE with ORB and BRISK
add_executable(${PROJECT_NAME}_compare src/akaze_compare.cpp)
target_link_libraries(${PROJECT_NAME}_compare
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_lib
  )

add_executable(${PROJECT_NAME}_extractor
  src/akaze_feature_extraction_node.cpp
  src/akaze_extract.cpp)
target_link_libraries(${PROJECT_NAME}_extractor
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_lib
  )
