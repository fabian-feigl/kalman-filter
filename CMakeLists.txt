cmake_minimum_required(VERSION 3.1...3.24)

project(
  KalmanFilter
  VERSION 1.0
  LANGUAGES CXX)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/protocol")



