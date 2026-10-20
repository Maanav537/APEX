cmake_minimum_required(VERSION 3.28)
project(APEX VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)

# --- Dependencies (pinned; downloaded at configure time) ---------------------
include(FetchContent)

# Eigen: header-only; skip its own tests, docs, and extras.
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_BLAS OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_LAPACK OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(Eigen3
    URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_Declare(Catch2
    URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.16.1.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

# Rerun: EXCLUDE_FROM_ALL so it (and Arrow) only compiles once a target links rerun_sdk.
FetchContent_Declare(rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/download/0.39.0/rerun_cpp_sdk.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL
)

# cpp-httplib: header-only HTTP server for the web front end (apex_web).
set(HTTPLIB_USE_OPENSSL_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
set(HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
FetchContent_Declare(httplib
    URL https://github.com/yhirose/cpp-httplib/archive/refs/tags/v0.57.1.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(Eigen3 Catch2 rerun_sdk httplib)

# --- Core library: all physics code compiles once here; tests and apps link to it.
add_library(apex
    src/core/version.cpp
    src/gnc/observer.cpp
    src/gnc/tracker.cpp
    src/physics/atmosphere.cpp
    src/sim/simulation.cpp
)
target_include_directories(apex PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(apex PUBLIC Eigen3::Eigen)
target_compile_options(apex PRIVATE
    "$<IF:$<CXX_COMPILER_ID:MSVC>,/W4;/permissive-,-Wall;-Wextra;-Wpedantic>"
)

# --- Week 1 demo: dummy mouse circle drawn in Rerun. First build compiles Arrow (slow).
add_executable(apex_sim apps/sim_main.cpp)
target_link_libraries(apex_sim PRIVATE apex rerun_sdk)

# --- Web front end: local server that runs preset scenarios; page in web/.
add_executable(apex_web apps/web_main.cpp)
target_link_libraries(apex_web PRIVATE apex httplib::httplib)
target_compile_definitions(apex_web PRIVATE APEX_WEB_DIR="${PROJECT_SOURCE_DIR}/web")

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
