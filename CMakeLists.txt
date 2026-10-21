cmake_minimum_required(VERSION 3.22)
project(infinity_gazebo_plugin VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Trova i pacchetti CMake nativi su Ubuntu 26.04
find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED)

include_directories(include)

add_library(InfinityBridge SHARED
  src/InfinityBridge.cc
)

target_link_libraries(InfinityBridge
  PUBLIC
    gz-sim::gz-sim
    gz-plugin::gz-plugin
)

set_target_properties(InfinityBridge PROPERTIES
  PREFIX "lib"
  SUFFIX ".so"
)