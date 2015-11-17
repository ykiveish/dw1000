cmake_minimum_required (VERSION 2.8)
project (dw1000)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories (
        ${PROJECT_SOURCE_DIR}/include
)

add_subdirectory (src)

add_executable (dw1000-hw-test dw1000-hw-test.cpp)
target_link_libraries (dw1000-hw-test dw1000)