project(HALCON_MATCH)

set_source_files_properties( tags PROPERTIES GENERATED true)
add_custom_command ( OUTPUT tags
    COMMAND ctags -R --c++-kinds=+p --fields=+iaS --extra=+q . 
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} )

add_executable(match match_node.cpp tags)

target_include_directories(match PUBLIC ${asr_halcon_bridge_INCLUDE_DIRS})
target_include_directories(match PUBLIC ${CURRENT_SOURCE_DIR})

target_link_libraries(match $ENV{HALCONROOT}/lib/$ENV{HALCONARCH}/libhalconcpp.so)
target_link_libraries(match ${catkin_LIBRARIES})
target_link_libraries(match ${asr_halcon_bridge_LIBRARIES})

target_link_libraries(match halconmatcher)

