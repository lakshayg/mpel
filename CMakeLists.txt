project(nav2d)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "--std=c++11 -g -O0 -Wall")
# set(CMAKE_CXX_FLAGS "--std=c++11 -Wall")
add_subdirectory(mpl)
add_executable(main main.cpp)
target_link_libraries(main mpl)
