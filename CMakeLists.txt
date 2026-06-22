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
    add_executable(demo_stress examples/demo_stress.cpp)
    target_link_libraries(demo_stress PRIVATE hop raylib)

    add_executable(demo_settling examples/demo_settling.cpp)
    target_link_libraries(demo_settling PRIVATE hop raylib)

    add_executable(demo_pile examples/demo_pile.cpp)
    target_link_libraries(demo_pile PRIVATE hop raylib)

    add_executable(demo_static_rotation examples/demo_static_rotation.cpp)
    target_link_libraries(demo_static_rotation PRIVATE hop raylib)
endif()

# ReactPhysics3D port of demo_bounce — side-by-side performance comparison.
# Fetches ReactPhysics3D from GitHub at configure time.
option(HOP_BUILD_RP3D_DEMO "Build ReactPhysics3D port of demo_bounce" OFF)
if(HOP_BUILD_RP3D_DEMO)
    find_package(raylib REQUIRED)
    include(FetchContent)
    FetchContent_Declare(
        reactphysics3d
        GIT_REPOSITORY https://github.com/DanielChappuis/reactphysics3d.git
        GIT_TAG        v0.10.2
    )
    FetchContent_MakeAvailable(reactphysics3d)
    add_executable(demo_bounce_rp3d examples/demo_bounce_rp3d.cpp)
    target_link_libraries(demo_bounce_rp3d PRIVATE reactphysics3d raylib)
    add_executable(demo_stress_rp3d examples/demo_stress_rp3d.cpp)
    target_link_libraries(demo_stress_rp3d PRIVATE reactphysics3d raylib)
endif()

# Benchmarks — informational, not part of ctest.
option(HOP_BUILD_BENCHMARKS "Build hop benchmarks" OFF)
if(HOP_BUILD_BENCHMARKS)
    add_executable(hop_bench bench/main.cpp)
    target_link_libraries(hop_bench PRIVATE hop)
    target_compile_options(hop_bench PRIVATE -O2)
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
    target_compile_options(hop_web PRIVATE -O2)
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
