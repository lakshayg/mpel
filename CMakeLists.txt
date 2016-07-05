project(MPEL)

# The project version number.
set(VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(VERSION_MINOR   2   CACHE STRING "Project minor version number.")
set(VERSION_PATCH   0   CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "--std=c++11 -Wall -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(doxy_main_page ${PROJECT_SOURCE_DIR}/doc/index.md)

find_package(OpenCV REQUIRED)

include_directories(include/mpel)
file(GLOB_RECURSE mpel_SRC "lib/*.cpp")
file(GLOB_RECURSE mpel_HDR "lib/*.hpp")
add_library(mpel ${mpel_SRC} ${mpel_HDR})
target_link_libraries(mpel ${OpenCV_LIBRARIES})
target_include_directories(mpel PUBLIC include lib)

# add demo, user and test files
add_subdirectory(test)
add_subdirectory(demo)
add_subdirectory(user)
add_subdirectory(doc)

