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

option(WARNINGS "Enable warnings" OFF)
option(OPTIMIZE_FOR_NATIVE "Build with -march=native" OFF)

# ################### Modules ####################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(GNUInstallDirs)
include(CompilerWarnings)
include(CompilerOptimizations)
include(Conan)
run_conan()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}/dependencies/melon" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}/dependencies/mippp" ${CMAKE_MODULE_PATH})

add_subdirectory(dependencies/melon)
add_subdirectory(dependencies/mippp)

# ################### Packages ####################
find_package(fast-cpp-csv-parser REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(phmap REQUIRED)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(range-v3 REQUIRED)

find_package(COIN REQUIRED)
find_package(GUROBI REQUIRED)

# ################### LIB target ####################
add_library(landscape_opt INTERFACE)
target_include_directories(landscape_opt INTERFACE src/landscape_opt)

target_link_libraries(landscape_opt INTERFACE TBB::tbb)
target_link_libraries(landscape_opt INTERFACE Boost::Boost)
target_link_libraries(landscape_opt INTERFACE Eigen3::Eigen3)
target_link_libraries(landscape_opt INTERFACE range-v3::range-v3)

target_link_libraries(landscape_opt INTERFACE melon)
target_link_libraries(landscape_opt INTERFACE mippp)

# target_include_directories(landscape_opt INTERFACE ${COIN_INCLUDE_DIR})
# target_link_libraries(landscape_opt INTERFACE ${COIN_LIBRARIES})

# if(${WITH_GUROBI} STREQUAL "ON")
#     target_include_directories(landscape_opt INTERFACE ${GUROBI_INCLUDE_DIR})
#     target_link_libraries(landscape_opt INTERFACE ${GUROBI_LIBRARIES})
# endif()


# # ################## TEST target ####################
# # enable_testing()

# # add_executable(test_test test/test.cpp)
# # target_include_directories(test_test PUBLIC include)
# # target_link_libraries(test_test CONAN_PKG::gtest)
# # target_link_libraries(test_test landscape_opt)

# # include(GoogleTest)
# # gtest_discover_tests(test_test)

# # ################## EXEC target ####################

add_executable(lscp_solve src/solve.cpp)
target_include_directories(lscp_solve PUBLIC src include)
target_link_libraries(lscp_solve PUBLIC landscape_opt)
target_link_libraries(lscp_solve PUBLIC fast-cpp-csv-parser::fast-cpp-csv-parser)
target_link_libraries(lscp_solve PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(lscp_solve PUBLIC phmap::phmap)


