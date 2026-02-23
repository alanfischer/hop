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

# Examples
option(HOP_BUILD_EXAMPLES "Build example programs" OFF)
if(HOP_BUILD_EXAMPLES)
    find_package(raylib REQUIRED)
    add_executable(demo_bounce examples/demo_bounce.cpp)
    target_link_libraries(demo_bounce PRIVATE hop raylib)
endif()

# WebAssembly bindings (build with emcmake)
option(HOP_BUILD_WEB "Build WebAssembly bindings" OFF)
if(HOP_BUILD_WEB)
    add_executable(hop_web web/hop_bindings.cpp)
    target_link_libraries(hop_web PRIVATE hop)
    set_target_properties(hop_web PROPERTIES
        OUTPUT_NAME "hop"
        SUFFIX ".js"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/web"
    )
    target_link_options(hop_web PRIVATE
        --bind
        -sWASM=1
        -sALLOW_MEMORY_GROWTH=1
        -sMODULARIZE=1
        -sEXPORT_NAME=createHopModule
        -sENVIRONMENT=web
        -O2
    )
endif()
