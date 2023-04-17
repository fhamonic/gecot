# ################### Packages ###################
find_package(GTest)
include(GoogleTest)

# ################# TEST targets #################
add_executable(landscape_opt_test main_test.cpp preprocessing_test.cpp)
target_link_libraries(landscape_opt_test landscape_opt)
target_link_libraries(landscape_opt_test fast-cpp-csv-parser::fast-cpp-csv-parser)
target_link_libraries(landscape_opt_test nlohmann_json::nlohmann_json)
target_link_libraries(landscape_opt_test phmap::phmap)
target_link_libraries(landscape_opt_test GTest::gtest)

target_compile_definitions(landscape_opt_test PUBLIC PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

gtest_discover_tests(landscape_opt_test)