project(task_1)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME} src/main.cpp src/algorithms.cpp src/utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)