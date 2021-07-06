project(i3dr_halcon_surface_matcher)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        rospy
        std_msgs
        genmsg
        object_recognition_msgs
        tf
)

enable_testing()
find_package(GTest REQUIRED)

find_package(asr_halcon_bridge REQUIRED)

find_package(Halcon)
if(NOT ${Halcon_FOUND})
   message(FATAL_ERROR  "Skip processing ${PROJECT_NAME}, because HALCON library is missing!")
   return()
endif()

include(GNUInstallDirs)

enable_testing()
find_package(GTest REQUIRED)

catkin_package( 
    INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs
	LIBRARIES ${PROJECT_NAME}
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
    ${Halcon_INCLUDE_DIRS}
    ${asr_halcon_bridge_INCLUDE_DIRS}
)

# Build surface matching library
add_library(halcon_surface_match src/halconmatcher.cpp)
target_link_libraries(
    halcon_surface_match
	${catkin_LIBRARIES}
    ${Halcon_LIBRARIES}
)

add_executable(match_surface src/match_node.cpp)

target_link_libraries(
    match_surface
    halcon_surface_match
	${catkin_LIBRARIES}
	${asr_halcon_bridge_LIBRARIES}
)

# Tests
catkin_add_gtest(match_surface_tests test/test_match.cpp)
if(TARGET match_surface_tests)
    target_link_libraries(match_surface_tests 
        halcon_surface_match
        ${catkin_LIBRARIES}
        ${asr_halcon_bridge_LIBRARIES}
    )
endif()