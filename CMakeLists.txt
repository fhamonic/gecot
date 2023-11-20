cmake_minimum_required(VERSION 3.20)

project(
  Gecot
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

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}/dependencies/melon"
                      ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}/dependencies/mippp"
                      ${CMAKE_MODULE_PATH})

add_subdirectory(dependencies/mippp)

# ################### Packages ####################
find_package(fast-cpp-csv-parser REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator REQUIRED)
find_package(phmap REQUIRED)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options log log_setup)
find_package(Eigen3 REQUIRED)
# find_package(range-v3 REQUIRED)

find_package(melon REQUIRED)

# ################### LIB target ####################
add_library(gecot INTERFACE)
target_include_directories(gecot
                           INTERFACE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(gecot INTERFACE TBB::tbb)
target_link_libraries(gecot INTERFACE Boost::program_options Boost::log Boost::log_setup)
target_link_libraries(gecot INTERFACE Eigen3::Eigen)
# target_link_libraries(gecot INTERFACE range-v3::range-v3)

target_link_libraries(gecot INTERFACE melon::melon)
target_link_libraries(gecot INTERFACE mippp)
target_compile_definitions(gecot INTERFACE)
target_compile_definitions(gecot INTERFACE MIPPP_PREFERED_SOLVER=MIPPP_GUROBI)
# target_compile_definitions(gecot INTERFACE MIPPP_PREFERED_SOLVER=MIPPP_SCIP)
# target_compile_definitions(gecot INTERFACE MIPPP_PREFERED_SOLVER=MIPPP_COINOR)

# ################### EXEC target ####################
add_executable(gecot_solve src/solve.cpp)
target_link_libraries(gecot_solve PUBLIC gecot)
target_link_libraries(gecot_solve
                      PUBLIC fast-cpp-csv-parser::fast-cpp-csv-parser)
target_link_libraries(gecot_solve PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(
  gecot_solve
  PUBLIC nlohmann_json_schema_validator)
target_link_libraries(gecot_solve PUBLIC phmap)

target_compile_options(gecot_solve PUBLIC -fconcepts-diagnostics-depth=4 -Wall)

# target_compile_options(gecot_solve PRIVATE -fsanitize=address)
# target_link_options(gecot_solve PRIVATE -fsanitize=address -lpthread -static-libasan)

add_executable(gecot_rank src/rank.cpp)
target_link_libraries(gecot_rank PUBLIC gecot) 
target_link_libraries(gecot_rank PUBLIC fast-cpp-csv-parser::fast-cpp-csv-parser) 
target_link_libraries(gecot_rank PUBLIC nlohmann_json::nlohmann_json) 
target_link_libraries(gecot_rank PUBLIC nlohmann_json_schema_validator)
target_link_libraries(gecot_rank PUBLIC phmap)

target_compile_options(gecot_rank PUBLIC -fconcepts-diagnostics-depth=4 -Wall)

add_executable(gecot_test src/test.cpp)
target_link_libraries(gecot_test PUBLIC gecot)
target_link_libraries(gecot_test PUBLIC fast-cpp-csv-parser::fast-cpp-csv-parser)
target_link_libraries(gecot_test PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(
  gecot_test
  PUBLIC nlohmann_json_schema_validator)
target_link_libraries(gecot_test PUBLIC phmap)

target_compile_options(gecot_test PUBLIC -fconcepts-diagnostics-depth=4 -Wall)

# ################### TESTS ######################
if(ENABLE_TESTING)
enable_testing()
message("Building Tests.")
set_project_warnings(gecot)
add_subdirectory(test) 
endif()
