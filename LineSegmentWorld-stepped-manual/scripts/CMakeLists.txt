cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(YTapeGenerator)


#find_package( Eigen3 REQUIRED )

include_directories ( src )
include_directories( eigen )


add_library( y_tape src/YTapeGenerator.cpp)
add_executable( test_y_tape src/test_y_tape.cpp)
target_link_libraries( test_y_tape y_tape)
