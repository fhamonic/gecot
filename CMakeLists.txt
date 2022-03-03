cmake_minimum_required(VERSION 3.12)

project(
    Landscape_Opt
    VERSION 1.0
    DESCRIPTION
        "Library for handling ecological landscape datas and optimize their connectivity according to the PC indicator .. and more incomming"
    LANGUAGES CXX)

# set(CMAKE_VERBOSE_MAKEFILE ON)

# ################### Options ####################
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CONAN_DISABLE_CHECK_COMPILER ON)

option(WARNINGS "" OFF)
option(HARDCORE_WARNINGS "" OFF)
option(COMPILE_FOR_NATIVE "" OFF)
option(COMPILE_WITH_LTO "" OFF)
option(WITH_GUROBI "" OFF)

set(WARNINGS_LIST "-Wall;-Wextra;")
set(HARDCORE_WARNINGS_LIST
    "-Wall;-Wextra;-Wshadow;-Wpedantic;-Wnon-virtual-dtor;-Wold-style-cast;-Wcast-align;-Wunused;-Woverloaded-virtual;-Wpedantic;-Wconversion;-Wsign-conversion;-Wmisleading-indentation;-Wduplicated-cond;-Wduplicated-branches;-Wlogical-op;-Wnull-dereference;-Wuseless-cast;-Wdouble-promotion;-Wformat=2;-Wlifetime"
)

# ################### Modules ####################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(CMakeBuildTypeUtils)

# ################### Packages ####################
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

find_package(LEMON REQUIRED)
find_package(COIN REQUIRED)
if(${WITH_GUROBI} STREQUAL "ON")
    find_package(GUROBI REQUIRED)
endif()


# ################### LIB target ####################
add_library(landscape_opt INTERFACE)
target_include_directories(landscape_opt INTERFACE src/landscape_opt)

target_link_libraries(landscape_opt INTERFACE CONAN_PKG::fast-cpp-csv-parser)
target_link_libraries(landscape_opt INTERFACE CONAN_PKG::nlohmann_json)
target_link_libraries(landscape_opt INTERFACE CONAN_PKG::parallel-hashmap)
target_link_libraries(landscape_opt INTERFACE CONAN_PKG::tbb)
target_link_libraries(landscape_opt INTERFACE CONAN_PKG::boost)
target_link_libraries(landscape_opt INTERFACE CONAN_PKG::eigen)
target_link_libraries(landscape_opt INTERFACE CONAN_PKG::range-v3)

target_include_directories(landscape_opt INTERFACE ${COIN_INCLUDE_DIR})
target_link_libraries(landscape_opt INTERFACE ${COIN_LIBRARIES})

if(${WITH_GUROBI} STREQUAL "ON")
    target_include_directories(landscape_opt INTERFACE ${GUROBI_INCLUDE_DIR})
    target_link_libraries(landscape_opt INTERFACE ${GUROBI_LIBRARIES})
endif()


# ################## TEST target ####################
# enable_testing()

# add_executable(test_test test/test.cpp)
# target_include_directories(test_test PUBLIC include)
# target_link_libraries(test_test CONAN_PKG::gtest)
# target_link_libraries(test_test landscape_opt)

# include(GoogleTest)
# gtest_discover_tests(test_test)

# ################## EXEC target ####################

add_executable(solve src/solve.cpp)
target_include_directories(solve PUBLIC include melon/single-header mippp/single-header)
target_link_libraries(solve PUBLIC landscape_opt)

target_compile_options_if_exists(solve PRIVATE ${WARNINGS}
                                 ${WARNINGS_LIST})
target_compile_options_if_exists(solve PRIVATE ${HARDCORE_WARNINGS}
                                 ${HARDCORE_WARNINGS_LIST})
target_compile_options_if_exists(solve PRIVATE ${COMPILE_FOR_NATIVE}
                                 -march=native)
target_compile_options_if_exists(solve PRIVATE ${COMPILE_WITH_LTO}
                                 -flto)
