message("Including binary \"${BoldMagenta}SIMULATOR${ColorReset}\".")

# include pybind11
add_subdirectory(${CMAKE_SOURCE_DIR}/../pybind11 pybind11)

pybind11_add_module(example example.cpp)
target_link_libraries(example PRIVATE ${ilqgames_LIBRARIES})

# Define the path to the log directory.
add_definitions(-DILQGAMES_LOG_DIR="${CMAKE_SOURCE_DIR}/logs")
