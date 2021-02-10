cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(spinaltap-os C CXX)
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(external)

enable_testing()
add_executable(fixed_queue test/fixed_queue.cpp)
target_link_libraries(fixed_queue Catch2::Catch2WithMain)
target_include_directories(fixed_queue PUBLIC src)
add_test(fixed_queue fixed_queue)