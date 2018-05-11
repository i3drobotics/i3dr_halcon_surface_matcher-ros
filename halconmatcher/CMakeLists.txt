project(halconmatcher LANGUAGES CXX)

add_library(halconmatcher src/halconmatcher.cpp)

target_include_directories(halconmatcher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(halconmatcher PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(halconmatcher PRIVATE $ENV{HALCONROOT}/include)
target_link_libraries(halconmatcher $ENV{HALCONROOT}/lib/$ENV{HALCONARCH}/libhalconcpp.so)
target_link_libraries(halconmatcher ${catkin_LIBRARIES})

# Vanilla export from Halcon:

find_package(X11 REQUIRED)

add_executable(halcon_vanilla src/vanilla_halcon.cpp)

target_include_directories(halcon_vanilla PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(halcon_vanilla PRIVATE ${X11_INCLUDE_DIRS})
target_include_directories(halcon_vanilla PUBLIC $ENV{HALCONROOT}/include)
target_link_libraries(halcon_vanilla $ENV{HALCONROOT}/lib/$ENV{HALCONARCH}/libhalconcpp.so)
target_link_libraries(halcon_vanilla ${X11_LIBRARIES})

# Tests
add_executable(halconMatcherTests test/test_match.cpp)

target_include_directories(halconMatcherTests PUBLIC ${GTEST_INCLUDE_DIRS})

target_include_directories(halconMatcherTests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(halconMatcherTests PUBLIC $ENV{HALCONROOT}/include)

target_link_libraries(halconMatcherTests ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(halconMatcherTests halconmatcher)
add_test(NAME halconmatcher
         COMMAND halconMatcherTests)
