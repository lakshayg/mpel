project(mpl)
cmake_minimum_required(VERSION 2.8)

# set(CMAKE_CXX_FLAGS "--std=c++11 -g -O0 -Wall")
set(CMAKE_CXX_FLAGS "--std=c++11 -Wall")

include_directories(lib)

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE mpl_SRC lib "*.cpp")

add_library(mpl ${mpl_SRC})
target_link_libraries(mpl ${OpenCV_LIBRARIES})
target_include_directories (mpl PUBLIC include lib)
