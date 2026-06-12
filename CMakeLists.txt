cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mazegen_gazebo
  VERSION 1.0.0
  DESCRIPTION "Gazebo world plugin that builds micromouse mazes from text files"
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

add_library(mazegen_plugin SHARED
  src/mazegen_plugin.cpp
  src/maze_parse.cpp
  src/maze_params.cpp
  src/maze_sdf_builder.cpp
  src/maze_spawn_utils.cpp)
set_property(TARGET mazegen_plugin PROPERTY CXX_STANDARD 17)

target_include_directories(mazegen_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(mazegen_plugin
  PRIVATE
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::register
    ignition-common4::ignition-common4
    ignition-math6::ignition-math6
    ignition-msgs8::ignition-msgs8
    ignition-transport11::ignition-transport11
    sdformat12::sdformat12)

install(TARGETS mazegen_plugin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)


# ------------------------------------------
# Doxygen
#
# build : cmake --build build --target doc
# Output: build/docs/html/index.html.
# ------------------------------------------
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_IN  ${CMAKE_CURRENT_SOURCE_DIR}/documentation/Doxyfile)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

  add_custom_target(doc
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)
else()
  message(STATUS "Doxygen not found - 'doc' target disabled")
endif()


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
