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

if(NOT WIN32)
  set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
endif()

# ################### Packages ####################
find_package(fast-cpp-csv-parser REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator REQUIRED)
find_package(phmap REQUIRED)
if(NOT WIN32)
  find_package(TBB REQUIRED)
endif()
find_package(Boost REQUIRED COMPONENTS program_options log log_setup)
find_package(Eigen3 REQUIRED)

find_package(melon REQUIRED)
find_package(mippp REQUIRED)

# ################### LIB target ####################
add_library(gecot INTERFACE)
target_compile_definitions(gecot INTERFACE PROJECT_VERSION="${CMAKE_PROJECT_VERSION_MAJOR}.${CMAKE_PROJECT_VERSION_MINOR}")
target_include_directories(gecot
                           INTERFACE ${PROJECT_SOURCE_DIR}/include)

if(WIN32)
  target_include_directories(gecot INTERFACE "C:/Users/Plaiseek/.conan2/p/b/onetb4e7bf2dd4d1ab/p/include")
  target_link_libraries(gecot INTERFACE "C:/Users/Plaiseek/.conan2/p/b/onetb4e7bf2dd4d1ab/p/lib/libtbb12.dll")
else()
  target_link_libraries(gecot INTERFACE TBB::tbb)
endif()
target_link_libraries(gecot INTERFACE Boost::program_options Boost::log Boost::log_setup)
target_link_libraries(gecot INTERFACE Eigen3::Eigen)
# target_link_libraries(gecot INTERFACE range-v3::range-v3)

target_link_libraries(gecot INTERFACE melon::melon)
target_link_libraries(gecot INTERFACE mippp::mippp)

# ################### MIP solver ####################
# find_package(SCIP REQUIRED)
# target_compile_definitions(gecot INTERFACE MIPPP_USE_SCIP=1)
# target_include_directories(gecot INTERFACE ${SCIP_INCLUDE_DIR})
# target_link_libraries(gecot INTERFACE ${SCIP_LIBRARIES})

# find_package(GUROBI REQUIRED)
# target_compile_definitions(gecot INTERFACE MIPPP_USE_GUROBI=1)
# target_include_directories(gecot INTERFACE ${GUROBI_INCLUDE_DIR})
# target_link_libraries(gecot INTERFACE ${GUROBI_LIBRARIES})

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

if(WIN32)
  target_link_libraries(gecot_solve PUBLIC "C:/Users/Plaiseek/.conan2/p/b/onetb4e7bf2dd4d1ab/p/lib/libtbb12.dll.a")
endif()

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

# ################## INSTALL #####################
install(TARGETS gecot_solve gecot_rank RUNTIME DESTINATION bin LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib)

if(NOT WIN32)
  install(CODE [[
    file(GET_RUNTIME_DEPENDENCIES
      EXECUTABLES $<TARGET_FILE:gecot_solve> $<TARGET_FILE:gecot_rank>
      RESOLVED_DEPENDENCIES_VAR _r_deps
      UNRESOLVED_DEPENDENCIES_VAR _u_deps
      DIRECTORIES bin
    )
    foreach(_file ${_r_deps})
      file(INSTALL
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
        TYPE SHARED_LIBRARY
        FILES "${_file}" FOLLOW_SYMLINK_CHAIN
      )
    endforeach()
    list(LENGTH _u_deps _u_length)
    if("${_u_length}" GREATER 0)
      message(WARNING "Unresolved dependencies detected! (" ${_u_deps} ")")
    endif()
  ]])
else()
install(FILES 
  "C:/Users/Plaiseek/.conan2/p/b/onetb4e7bf2dd4d1ab/p/bin/libtbb12.dll" 
  "C:/Users/Plaiseek/.conan2/p/mingw69dfc08609d40/p/x86_64-w64-mingw32/lib/libgcc_s_seh-1.dll" 
  "C:/Users/Plaiseek/.conan2/p/mingw69dfc08609d40/p/x86_64-w64-mingw32/lib/libwinpthread-1.dll" 
  "C:/Users/Plaiseek/.conan2/p/mingw69dfc08609d40/p/x86_64-w64-mingw32/lib/libstdc++-6.dll" 
  DESTINATION bin)
  set(CPACK_NSIS_MODIFY_PATH ON)
  set(CPACK_NSIS_DISPLAY_NAME Gecot)
  set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
  set(CPACK_NSIS_MUI_ICON "${PROJECT_SOURCE_DIR}/cmake/misc/gecot.ico")
  set(CPACK_NSIS_INSTALLED_ICON_NAME "${PROJECT_SOURCE_DIR}/cmake/misc/gecot.ico")
endif()
set(CPACK_PACKAGE_ICON "${PROJECT_SOURCE_DIR}/cmake/misc/gecot.ico") 
if(WIN32)
  set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/misc/windows_copyrights.txt")
else()
  set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/misc/linux_copyrights.txt")
endif()



include(CPack)
