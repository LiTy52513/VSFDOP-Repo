cmake_minimum_required(VERSION 3.16)

project(VSFDOP
    VERSION 0.1.0
    DESCRIPTION "VSF-DOP matrix-fracture intersection detection"
    LANGUAGES CXX)

option(VSFDOP_BUILD_EXAMPLES "Build VSF-DOP examples" ON)
option(VSFDOP_BUILD_TESTS "Build VSF-DOP tests" ON)

add_library(vsfdop
    src/vsf_dop.cpp)

target_include_directories(vsfdop
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(vsfdop PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(vsfdop PRIVATE /W4 /permissive-)
else()
    target_compile_options(vsfdop PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(VSFDOP_BUILD_EXAMPLES)
    add_executable(basic_vsf_dop examples/basic_vsf_dop.cpp)
    target_link_libraries(basic_vsf_dop PRIVATE vsfdop)
endif()

if(VSFDOP_BUILD_TESTS)
    enable_testing()

    add_executable(vsfdop_basic_test tests/vsfdop_basic_test.cpp)
    target_link_libraries(vsfdop_basic_test PRIVATE vsfdop)

    add_test(NAME vsfdop_basic_test COMMAND vsfdop_basic_test)
endif()
