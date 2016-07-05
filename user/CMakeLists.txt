project(MPEL-user)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "--std=c++11 -Wall -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# compile all user files
file(GLOB APP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
foreach(sourcefile ${APP_SOURCES})
	string(REPLACE ".cpp" "" exename ${sourcefile})
	add_executable(${exename} ${sourcefile} )
	target_link_libraries(${exename} mpel)
endforeach(sourcefile ${APP_SOURCES})
