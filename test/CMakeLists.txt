# ################### Packages ###################
find_package(GTest)
include(GoogleTest)

# ################# TEST targets #################
add_executable(gecot_test ../src/instance.cpp ../src/parse_instance.cpp main_test.cpp preprocessing_test.cpp parse_instance_test.cpp)
target_link_libraries(gecot_test gecot)
target_link_libraries(gecot_test fast-cpp-csv-parser::fast-cpp-csv-parser)
target_link_libraries(gecot_test nlohmann_json::nlohmann_json)
target_link_libraries(gecot_test nlohmann_json_schema_validator)
target_link_libraries(gecot_test phmap)
target_link_libraries(gecot_test GTest::gtest)

target_compile_definitions(gecot_test PUBLIC PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

gtest_discover_tests(gecot_test)