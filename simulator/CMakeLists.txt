message("Including binary \"${BoldMagenta}SIMULATOR${ColorReset}\".")


#### pybind11 module ####

# include pybind11
add_subdirectory(${CMAKE_SOURCE_DIR}/../pybind11 pybind11)

# link binary file
pybind11_add_module(ilqgamespy example.cpp)
target_link_libraries(ilqgamespy PRIVATE ilqgames ${ilqgames_LIBRARIES})
set_target_properties(ilqgamespy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/simulator)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#



#### test compiling for executable ####

#file(GLOB local_src ${CMAKE_SOURCE_DIR}/simulator/example.cpp)
#add_executable(ilqgamespy ${local_src})
#target_link_libraries(ilqgamespy PRIVATE ilqgames ${ilqgames_LIBRARIES})
#ilqgames_set_runtime_directory(ilqgamespy ${CMAKE_SOURCE_DIR}/simulator)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#





# Define the path to the log directory.
add_definitions(-DILQGAMES_LOG_DIR="${CMAKE_SOURCE_DIR}/logs")
