cmake_minimum_required(VERSION 2.8.3)
project(pandora_slam_3d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  orb_slam
  state_manager
  roslint
)

catkin_package(
	CATKIN_DEPENDS
		roscpp
		orb_slam
		state_manager
		roslint
	INCLUDE_DIRS
		include
 )

include_directories(include
  ${catkin_INCLUDE_DIRS}
 )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")


############################  SLAM 3D  ###################################
add_library(pandora_slam_3d src/pandora_slam_3d.cpp)
target_link_libraries(pandora_slam_3d
	${catkin_LIBRARIES}
)
add_executable(pandora_slam_3d_node src/pandora_slam_3d_node.cpp)
target_link_libraries(pandora_slam_3d_node
  ${catkin_LIBRARIES}
  pandora_slam_3d
)

set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line,-build/include_order,-whitespace/blank_line,
    -whitespace/parens,-whitespace/comments,-whitespace/braces")
FILE(GLOB_RECURSE pandora_slam_3d_LINT_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR}
            include/pandora_slam_3d/*.h
            src/*.cpp
            )
roslint_cpp(${pandora_slam_3d_LINT_SRCS})
