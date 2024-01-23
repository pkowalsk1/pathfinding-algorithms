project(task_1)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME} src/main.cpp src/algorithms.cpp src/utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  include(CTest)

  add_executable(${PROJECT_NAME}_test_utils ./test/utils_test.cpp ./src/utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_utils PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_test_utils gtest)
  add_test(NAME ${PROJECT_NAME}_test_utils COMMAND ${PROJECT_NAME}_test_utils)

  add_executable(${PROJECT_NAME}_test_a_start ./test/a_star_test.cpp ./src/utils.cpp
    ./src/algorithms.cpp)
  target_include_directories(${PROJECT_NAME}_test_a_start PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_test_a_start gtest)
  add_test(NAME ${PROJECT_NAME}_test_a_start COMMAND ${PROJECT_NAME}_test_a_start)
endif()