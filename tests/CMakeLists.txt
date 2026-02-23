add_executable(test_fixed16 test_fixed16.cpp)
target_link_libraries(test_fixed16 PRIVATE hop)
add_test(NAME test_fixed16 COMMAND test_fixed16)

add_executable(test_vec3 test_vec3.cpp)
target_link_libraries(test_vec3 PRIVATE hop)
add_test(NAME test_vec3 COMMAND test_vec3)

add_executable(test_intersect test_intersect.cpp)
target_link_libraries(test_intersect PRIVATE hop)
add_test(NAME test_intersect COMMAND test_intersect)

add_executable(test_simulator test_simulator.cpp)
target_link_libraries(test_simulator PRIVATE hop)
add_test(NAME test_simulator COMMAND test_simulator)
