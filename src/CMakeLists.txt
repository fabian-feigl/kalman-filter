cmake_minimum_required(VERSION 3.13)

add_library(kalman_filter_lib STATIC kalman_filter.cpp motion_model.cpp)
target_include_directories(kalman_filter_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_executable(kalman_filter kalman_filter_main.cpp)
target_link_libraries(kalman_filter PRIVATE kalman_filter_lib)
