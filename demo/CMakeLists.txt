project(mpel-demo)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "--std=c++11 -g -O0 -Wall")

# compile all demo files
file(GLOB APP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
foreach(sourcefile ${APP_SOURCES})
	string(REPLACE ".cpp" "" exename ${sourcefile})
	add_executable(${exename} ${sourcefile} )
	target_link_libraries(${exename} mpl)
endforeach(sourcefile ${APP_SOURCES})
