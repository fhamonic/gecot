#include <gtest/gtest.h>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "melon/mapping.hpp"
#include "melon/utility/graphviz_printer.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/pc_num.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

// GTEST_TEST(parse_instance, json_test) {
//     static const nlohmann::json instance_schema = R"(
// {
    
// }
// )"_json;
//     std::string instance_path = PROJECT_SOURCE_DIR;
//     // instance_path.append("/test/instances/aude.json");
//     instance_path.append("/test/instances/quebec_438_RASY.json");
//     // instance_path.append("/test/instances/biorevaix_N1.json");
//     Instance instance = parse_instance(instance_path);
//     double budget = 0;
//     for(auto && o : instance.options()) budget += instance.option_cost(o);
// }


GTEST_TEST(parse_instance, test) {
    std::string instance_path = PROJECT_SOURCE_DIR;
    // instance_path.append("/test/instances/aude.json");
    instance_path.append("/test/instances/quebec_438_RASY.json");
    // instance_path.append("/test/instances/biorevaix_N1.json");
    Instance instance = parse_instance(instance_path);
    double budget = 0;
    for(auto && o : instance.options()) budget += instance.option_cost(o);
}
