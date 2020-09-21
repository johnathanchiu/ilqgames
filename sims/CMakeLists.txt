# set(executable ${CMAKE_SOURCE_DIR}/sims)
ilqgames_subdir_list(executables ${CMAKE_SOURCE_DIR}/sims)
foreach(executable ${executables})
  message("Including binary \"${BoldMagenta}${executable}${ColorReset}\".")
  add_subdirectory(${CMAKE_SOURCE_DIR}/../pybind11 pybind11)
  pybind11_add_module(example ${executable}/main.cpp) 
  file(GLOB_RECURSE local_src ${executable}/*.cpp)
  add_executable(${executable} ${local_src})
  target_link_libraries(${executable} ilqgames ${ilqgames_LIBRARIES} imgui glfw)
  ilqgames_set_runtime_directory(${executable} ${EXECUTABLE_OUTPUT_PATH})
endforeach()



# Define a variable storing the path to executable directory.
add_definitions(-DILQGAMES_EXEC_DIR="${CMAKE_SOURCE_DIR}/sims")

# Define the path to the log directory.
add_definitions(-DILQGAMES_LOG_DIR="${CMAKE_SOURCE_DIR}/logs")
