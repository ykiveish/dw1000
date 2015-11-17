include_directories (${PROJECT_SOURCE_DIR}/include)

add_library (dw1000 SHARED ../platforms/edison/dw1000.cpp)
target_link_libraries (dw1000 mraa)