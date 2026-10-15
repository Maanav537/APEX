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

FetchContent_MakeAvailable(Eigen3 Catch2 rerun_sdk)

# --- Core library: all physics code compiles once here; tests and apps link to it.
add_library(apex
    src/core/version.cpp
)
target_include_directories(apex PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(apex PUBLIC Eigen3::Eigen)
target_compile_options(apex PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
