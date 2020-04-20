cmake_minimum_required(VERSION 3.16)

project(scanner VERSION 1.0
        DESCRIPTION "Slamtec RPLIDAR A3 simple scanner"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# compile options
add_compile_options(-Wall -Wextra -Wpedantic)

find_package (Threads REQUIRED)

add_executable(scanner src/main.cpp)
target_link_libraries(scanner
        PUBLIC Threads::Threads
        PUBLIC "${CMAKE_SOURCE_DIR}/extern/rplidar_sdk/bin/librplidar_sdk.a")

target_include_directories(scanner PUBLIC "${CMAKE_SOURCE_DIR}/extern/rplidar_sdk/include")
