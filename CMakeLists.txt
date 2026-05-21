cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mazegen_ign_gazebo
  VERSION 1.0.0
  DESCRIPTION "Ignition Gazebo Fortress world plugin that builds micromouse mazes from text files"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Ignition Fortress = ignition-gazebo6, ignition-plugin1, sdformat12,
# ignition-common4, ignition-math6.
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(sdformat12 REQUIRED)

add_library(gazebo_mazegen_plugin SHARED
  src/gazebo_mazegen_plugin.cpp
  src/maze_parse.cpp)
set_property(TARGET gazebo_mazegen_plugin PROPERTY CXX_STANDARD 17)

target_include_directories(gazebo_mazegen_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(gazebo_mazegen_plugin
  PRIVATE
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::register
    ignition-common4::ignition-common4
    ignition-math6::ignition-math6
    ignition-msgs8::ignition-msgs8
    ignition-transport11::ignition-transport11
    sdformat12::sdformat12)

install(TARGETS gazebo_mazegen_plugin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)


# -----------------
# Unit tests
# -----------------

include(CTest)
if(BUILD_TESTING)
  add_executable(test_maze_parser
    test/test_maze_parser.cpp
    src/maze_parse.cpp)
  target_include_directories(test_maze_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME maze_parser COMMAND test_maze_parser)
endif()
