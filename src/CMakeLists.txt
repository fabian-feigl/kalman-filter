cmake_minimum_required(VERSION 3.13)

add_library(kalman_filter_lib STATIC kalman_filter.cpp motion_model.cpp)
target_include_directories(kalman_filter_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_executable(kalman_filter_example kalman_filter_example.cpp)
target_link_libraries(kalman_filter_example PRIVATE kalman_filter_lib)

add_executable(kalman_filter_server kalman_filter_server.cpp kalman_filter_impl.cpp)
target_include_directories(kalman_filter_server PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/build/include
    )

target_link_libraries(kalman_filter_server PUBLIC
    kalman_filter_lib
    kalman_filter_proto
    )
