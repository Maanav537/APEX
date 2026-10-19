list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

add_executable(apex_tests
    test_smoke.cpp
    test_types.cpp
    test_rk4.cpp
    test_domain.cpp
)
target_link_libraries(apex_tests PRIVATE apex Catch2::Catch2WithMain)
target_compile_options(apex_tests PRIVATE -Wall -Wextra -Wpedantic)

catch_discover_tests(apex_tests)
