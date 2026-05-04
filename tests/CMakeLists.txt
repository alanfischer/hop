# Tests rely on assert() — keep it live regardless of CMAKE_BUILD_TYPE.
add_compile_options(-UNDEBUG)

add_executable(test_fixed16 test_fixed16.cpp)
target_link_libraries(test_fixed16 PRIVATE hop)
add_test(NAME test_fixed16 COMMAND test_fixed16)

add_executable(test_fixed32 test_fixed32.cpp)
target_link_libraries(test_fixed32 PRIVATE hop)
add_test(NAME test_fixed32 COMMAND test_fixed32)

add_executable(test_vec3 test_vec3.cpp)
target_link_libraries(test_vec3 PRIVATE hop)
add_test(NAME test_vec3 COMMAND test_vec3)

add_executable(test_intersect test_intersect.cpp)
target_link_libraries(test_intersect PRIVATE hop)
add_test(NAME test_intersect COMMAND test_intersect)

add_executable(test_simulator test_simulator.cpp)
target_link_libraries(test_simulator PRIVATE hop)
add_test(NAME test_simulator COMMAND test_simulator)

add_executable(test_collision test_collision.cpp)
target_link_libraries(test_collision PRIVATE hop)
add_test(NAME test_collision COMMAND test_collision)

add_executable(test_bvh test_bvh.cpp)
target_link_libraries(test_bvh PRIVATE hop)
add_test(NAME test_bvh COMMAND test_bvh)

add_executable(test_support test_support.cpp)
target_link_libraries(test_support PRIVATE hop)
add_test(NAME test_support COMMAND test_support)

add_executable(test_compound test_compound.cpp)
target_link_libraries(test_compound PRIVATE hop)
add_test(NAME test_compound COMMAND test_compound)

add_executable(test_mat3 test_mat3.cpp)
target_link_libraries(test_mat3 PRIVATE hop)
add_test(NAME test_mat3 COMMAND test_mat3)

add_executable(test_quat test_quat.cpp)
target_link_libraries(test_quat PRIVATE hop)
add_test(NAME test_quat COMMAND test_quat)
