cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mazegen_ign_gazebo
  VERSION 1.0.0
  DESCRIPTION "Ignition Gazebo Fortress world plugin that builds micromouse mazes from text files"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


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
