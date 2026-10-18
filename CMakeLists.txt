cmake_minimum_required(VERSION 3.22)
project(my_gazebo_plugin VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Per Gazebo Jetty si usano gz-sim10 e gz-plugin4
find_package(gz-sim10 REQUIRED)
find_package(gz-plugin4 REQUIRED)

include_directories(include)

add_library(MySystem SHARED
  src/MySystem.cpp
)

target_link_libraries(MySystem
  PUBLIC
	gz-sim10::gz-sim10
	gz-plugin4::gz-plugin4
)

set_target_properties(MySystem PROPERTIES
  PREFIX "lib"
  SUFFIX ".so"
)