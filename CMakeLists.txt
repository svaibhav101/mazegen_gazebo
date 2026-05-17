cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mazegen_ign_gazebo
  VERSION 1.0.0
  DESCRIPTION "Ignition Gazebo Fortress world plugin that builds micromouse mazes from text files"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
