cmake_minimum_required(VERSION 3.14)
project(hop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header-only interface library
add_library(hop INTERFACE)
target_include_directories(hop INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Tests
option(HOP_BUILD_TESTS "Build hop tests" ON)
if(HOP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
