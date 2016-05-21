project(mpel-demo)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "--std=c++11 -g -O0 -Wall")
# set(CMAKE_CXX_FLAGS "--std=c++11 -Wall")

# compile all demo files
add_executable(simple_planner simple_planner.cpp)
target_link_libraries(simple_planner mpl)

add_executable(voronoi_planner voronoi_planner.cpp)
target_link_libraries(voronoi_planner mpl)
