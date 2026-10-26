# ################### Packages ###################
find_package(GTest)
include(GoogleTest)

# ################# TEST targets #################
add_executable(gecot_test ../src/instance.cpp ../src/parse_instance.cpp main_test.cpp preprocessing_test.cpp parse_instance_test.cpp solvers_test.cpp)
target_link_libraries(gecot_test gecot)
target_link_libraries(gecot_test vincentlaucsb-csv-parser::vincentlaucsb-csv-parser)
target_link_libraries(gecot_test nlohmann_json::nlohmann_json)
target_link_libraries(gecot_test nlohmann_json_schema_validator)
target_link_libraries(gecot_test phmap)
target_link_libraries(gecot_test GTest::gtest)

target_compile_definitions(gecot_test PUBLIC PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# each gecot header must compile alone
file(GLOB_RECURSE GECOT_HEADERS CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/gecot/*.hpp)
list(FILTER GECOT_HEADERS EXCLUDE REGEX unprep_target_benders)
foreach(HEADER ${GECOT_HEADERS})
  file(RELATIVE_PATH HEADER ${PROJECT_SOURCE_DIR}/include ${HEADER})
  string(MAKE_C_IDENTIFIER ${HEADER} TU)
  file(CONFIGURE OUTPUT header_check/${TU}.cpp CONTENT "#include \"${HEADER}\"\n")
  list(APPEND HEADER_CHECK_TUS ${CMAKE_CURRENT_BINARY_DIR}/header_check/${TU}.cpp)
endforeach()
add_library(gecot_header_check OBJECT ${HEADER_CHECK_TUS})
target_link_libraries(gecot_header_check gecot)

# execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libasan.so
#                 OUTPUT_VARIABLE LIBASAN_PATH
#                 OUTPUT_STRIP_TRAILING_WHITESPACE)

# message("cxx : ${CMAKE_CXX_COMPILER}")
# message("libasan : ${LIBASAN_PATH}")

# if(NOT "${LIBASAN_PATH}" STREQUAL "libasan.so")
#   target_compile_options(gecot_test PRIVATE -fsanitize=address)
#   target_link_options(gecot_test PRIVATE -fsanitize=address -lpthread)
#   target_link_libraries(gecot_test "${LIBASAN_PATH}")
# endif()

gtest_discover_tests(gecot_test)