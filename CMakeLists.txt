cmake_minimum_required(VERSION 3.22)

project(
  Gecot
  VERSION 1.0
  DESCRIPTION
    "Graph-based Ecological Connectivity Optimization Tool"
  LANGUAGES CXX)

# set(CMAKE_VERBOSE_MAKEFILE ON)

# ################### Options ####################
# set(CMAKE_CXX_STANDARD 26)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_TESTING "Enable unit tests" OFF)
option(WARNINGS "Enable warnings" OFF)
option(OPTIMIZE_FOR_NATIVE "Build with -march=native" OFF)

# ################### Modules ####################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(GNUInstallDirs)
include(CompilerWarnings)
include(CompilerOptimizations)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}" ${CMAKE_MODULE_PATH})

if(WIN32)
  file(TO_CMAKE_PATH "${CONAN_TBB_INCLUDE_DIR}" CONAN_TBB_INCLUDE_DIR)
  file(TO_CMAKE_PATH "${CONAN_TBB_LIB_DIR}" CONAN_TBB_LIB_DIR)
  file(TO_CMAKE_PATH "${CONAN_MINGW_LIB_DIR}" CONAN_MINGW_LIB_DIR)
endif()
if(NOT WIN32)
  set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
endif()

if(WIN32)
  set(WINDRES_ICON_O "misc/gecot_icon.o")
  execute_process(COMMAND "${CONAN_MINGW_LIB_DIR}/../bin/windres" "misc/gecot_icon.rc" -o ${WINDRES_ICON_O}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    RESULT_VARIABLE WINDRES_RESULT
    OUTPUT_VARIABLE WINDRES_OUTPUT)
  message(STATUS "WINDRES [${WINDRES_RESULT}]: ${WINDRES_OUTPUT}")
endif()

# ################### Packages ####################
find_package(vincentlaucsb-csv-parser REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator REQUIRED)
find_package(phmap REQUIRED)
if(NOT WIN32)
  find_package(TBB REQUIRED)
endif()
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(melon REQUIRED)
find_package(mippp REQUIRED)


# ################### LIB target ####################
add_library(gecot INTERFACE)
target_compile_definitions(gecot INTERFACE PROJECT_VERSION="${CMAKE_PROJECT_VERSION_MAJOR}.${CMAKE_PROJECT_VERSION_MINOR}")
target_include_directories(gecot
                           INTERFACE ${PROJECT_SOURCE_DIR}/include)

if(WIN32)
  target_include_directories(gecot INTERFACE ${CONAN_TBB_INCLUDE_DIR})
  target_link_directories(gecot INTERFACE ${CONAN_TBB_LIB_DIR} "${CONAN_TBB_LIB_DIR}\\..\\bin")
  target_link_libraries(gecot INTERFACE "${CONAN_TBB_LIB_DIR}/../bin/libtbb12.dll")
else()
  list(APPEND CMAKE_INSTALL_RPATH ${CONAN_TBB_LIB_DIR})
  target_link_libraries(gecot INTERFACE TBB::tbb)
endif()
target_link_libraries(gecot INTERFACE Boost::program_options)
target_link_libraries(gecot INTERFACE Eigen3::Eigen)
target_link_libraries(gecot INTERFACE fmt::fmt)
target_link_libraries(gecot INTERFACE spdlog::spdlog)

target_link_libraries(gecot INTERFACE melon::melon)
target_link_libraries(gecot INTERFACE mippp::mippp)

# ################### EXEC target ####################
add_executable(gecot_optimize src/instance.cpp src/parse_instance.cpp src/gecot_optimize.cpp ${WINDRES_ICON_O})
target_link_libraries(gecot_optimize PUBLIC gecot)
target_link_libraries(gecot_optimize PUBLIC vincentlaucsb-csv-parser::vincentlaucsb-csv-parser)
target_link_libraries(gecot_optimize PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(gecot_optimize PUBLIC nlohmann_json_schema_validator)
target_link_libraries(gecot_optimize PUBLIC phmap)
# target_compile_options(gecot_optimize PUBLIC -fconcepts-diagnostics-depth=30 -Wall)

# target_compile_options(gecot_optimize PRIVATE -fsanitize=address)
# target_link_options(gecot_optimize PRIVATE -fsanitize=address -lpthread -static-libasan)

# add_executable(gecot_rank_options src/rank_options.cpp ${WINDRES_ICON_O})
# target_link_libraries(gecot_rank_options PUBLIC gecot) 
# target_link_libraries(gecot_rank_options PUBLIC nlohmann_json::nlohmann_json) 
# target_link_libraries(gecot_rank_options PUBLIC nlohmann_json_schema_validator)
# target_link_libraries(gecot_rank_options PUBLIC phmap)
# target_compile_options(gecot_rank_options PUBLIC -fconcepts-diagnostics-depth=4 -Wall)



find_package(TIFF REQUIRED)
add_executable(gecot_hexagonify src/gecot_hexagonify.cpp ${WINDRES_ICON_O})
target_link_libraries(gecot_hexagonify PUBLIC gecot)
target_link_libraries(gecot_hexagonify PUBLIC TIFF::TIFF)
target_compile_options(gecot_hexagonify PUBLIC -fconcepts-diagnostics-depth=4 -Wall)

# ################### TESTS ######################
if(ENABLE_TESTING)
  enable_testing()
  message("Building Tests.")
  set_project_warnings(gecot)
  add_subdirectory(test) 
endif()

# ################## INSTALL #####################
install(TARGETS gecot_optimize RUNTIME DESTINATION bin LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib)

if(NOT WIN32)
  install(CODE [[
    file(GET_RUNTIME_DEPENDENCIES
      EXECUTABLES $<TARGET_FILE:gecot_optimize>
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
# install(FILES 
# "c:/libs/cbc/bin/cbc.exe" 
# "${CONAN_TBB_LIB_DIR}/../bin/libtbb12.dll" 
# "${CONAN_MINGW_LIB_DIR}/../x86_64-w64-mingw32/lib/libgcc_s_seh-1.dll" 
# "${CONAN_MINGW_LIB_DIR}/../x86_64-w64-mingw32/lib/libwinpthread-1.dll" 
# "${CONAN_MINGW_LIB_DIR}/../x86_64-w64-mingw32/lib/libstdc++-6.dll" 
# DESTINATION bin)
# set(CPACK_NSIS_MODIFY_PATH 2)
# set(CPACK_NSIS_DISPLAY_NAME Gecot)
# set(CPACK_NSIS_URL_INFO_ABOUT "https://github.com/fhamonic/gecot")
# set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
# set(CPACK_NSIS_INSTALLED_ICON_NAME "bin/gecot_optimize.exe")
# set(CPACK_NSIS_MUI_ICON "${PROJECT_SOURCE_DIR}/misc/gecot.ico")
# set(CPACK_NSIS_MUI_HEADERIMAGE "${PROJECT_SOURCE_DIR}/misc\\nsis_gecot_header.bmp")
install(FILES 
"c:/libs/cbc/bin/cbc.exe" 
"${CONAN_TBB_LIB_DIR}/../bin/libtbb12.dll" 
"${CONAN_MINGW_LIB_DIR}/../x86_64-w64-mingw32/lib/libgcc_s_seh-1.dll" 
"${CONAN_MINGW_LIB_DIR}/../x86_64-w64-mingw32/lib/libwinpthread-1.dll" 
"${CONAN_MINGW_LIB_DIR}/../x86_64-w64-mingw32/lib/libstdc++-6.dll" 
DESTINATION bin)

set(CPACK_GENERATOR WIX)
set(CPACK_WIX_VERSION 3)

# set(CPACK_WIX_PRODUCT_GUID 7315e975-bdb5-4797-b9e0-628fe7c89ee6)
set(CPACK_WIX_UPGRADE_GUID 504AB3DC-9C1E-4E24-B4D5-E3D3732DBD46)

set(CPACK_WIX_PROPERTY_ARPURLINFOABOUT "https://github.com/fhamonic/gecot")
set(CPACK_WIX_PRODUCT_ICON "${PROJECT_SOURCE_DIR}/misc/gecot.ico")
set(CPACK_WIX_UI_BANNER "${PROJECT_SOURCE_DIR}/misc/wix_gecot_banner.png")
set(CPACK_WIX_UI_DIALOG "${PROJECT_SOURCE_DIR}/misc/wix_gecot_dialog.png")
set(CPACK_WIX_PATCH_FILE "${PROJECT_SOURCE_DIR}/misc/PathPatch.xml")
set(CPACK_WIX_EXTENSIONS "WixUtilExtension")

endif()
set(CPACK_PACKAGE_ICON "${PROJECT_SOURCE_DIR}/misc/gecot.ico")
set(CPACK_PACKAGE_VENDOR "F. HAMONIC")
if(WIN32)
  set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/misc/windows_licenses.txt")
  install(FILES "${PROJECT_SOURCE_DIR}/misc/windows_licenses.txt" RENAME "LICENCE.txt" DESTINATION .)
else()
  set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/misc/linux_licenses.txt")
  install(FILES "${PROJECT_SOURCE_DIR}/misc/linux_licenses.txt" RENAME "LICENCE" DESTINATION .)
endif()

include(CPack)
