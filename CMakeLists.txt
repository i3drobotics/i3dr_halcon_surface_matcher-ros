project(halcon_match)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs genmsg)
find_package(asr_halcon_bridge REQUIRED)

include(GNUInstallDirs)

enable_testing()
find_package(GTest REQUIRED)

catkin_package()

add_subdirectory(halconmatcher)
add_subdirectory(src)