cmake_minimum_required(VERSION 3.14)
project(ScopicTestTask)

add_compile_definitions(_USE_MATH_DEFINES)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(MSVC)
    string(REPLACE "/W3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    add_compile_options(/W4)
elseif(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(SFML 2.6 COMPONENTS graphics QUIET)
if(NOT SFML_FOUND)
    include(FetchContent)
    FetchContent_Declare(SFML URL https://github.com/SFML/SFML/archive/refs/tags/2.6.1.zip)
    option(SFML_BUILD_AUDIO "Build audio" OFF)
    option(SFML_BUILD_NETWORK "Build network" OFF)
    FetchContent_MakeAvailable(SFML)
endif()

include_directories(include)

file(GLOB_RECURSE SOURCES "src/*.c*")
file(GLOB_RECURSE HEADERS "include/*.h*")

add_executable(scopic_test_task ${SOURCES} ${HEADERS})

if(TARGET sfml-graphics)
    message(STATUS "SFML was found and is available")
    target_compile_definitions(scopic_test_task PRIVATE SFML_AVAILABLE)
    target_link_libraries(scopic_test_task sfml-graphics)
else()
    message(STATUS "SFML was not found")
endif()

if(EXISTS "tests/CMakeLists.txt")
    message(STATUS "Tests are enabled")
    enable_testing()
    add_subdirectory(tests)
endif()
