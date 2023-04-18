#include <gtest/gtest.h>
#include <string>

#include "landscape_opt/preprocessing/compute_strong_and_useless_arcs.hpp"
#include "landscape_opt/preprocessing/compute_contracted_generalized_flow_graph.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

GTEST_TEST(preprocessing, fuzzy_test) {
  std::string instance_path = PROJECT_SOURCE_DIR;
  instance_path.append("/test/instances/aude.json");

  Instance instance = parse_instance(instance_path);

  const auto [strong_arcs_map, useless_arcs_map] =
      landscape_opt::compute_strong_and_useless_arcs(instance, true);

  for (const auto &original_t : melon::vertices(instance.landscape().graph())) {
    const auto [graph, quality_map, vertex_options_map, arc_no_map,
                probability_map, arc_option_map, t] =
        landscape_opt::compute_contracted_generalized_flow_graph(instance, strong_arcs_map,
                                                  useless_arcs_map, original_t);
  }

  
}
