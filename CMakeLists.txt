project(mpel)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "--std=c++11 -Wall -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE mpel_SRC "lib/*.cpp")
file(GLOB_RECURSE mpel_HDR "lib/*.h")
add_library(mpel ${mpel_SRC} ${mpel_HDR})
target_link_libraries(mpel ${OpenCV_LIBRARIES})
target_include_directories(mpel PUBLIC include lib)

# add demo, user and test files
add_subdirectory(test)
add_subdirectory(demo)
add_subdirectory(user)

