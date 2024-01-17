#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include "gecot/indices/pc_num.hpp"
#include "gecot/indices/parallel_pc_num.hpp"
#include "gecot/utils/chronometer.hpp"

#include "gecot/helper.hpp"
#include "gecot/solvers/static_incremental.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

int main(int argc, const char * argv[]) {
    std::filesystem::path instances_description_json = argv[1];

    static_assert(gecot::instance_c<Instance>);
    static_assert(gecot::case_c<InstanceCase>);

    Instance instance = parse_instance(instances_description_json);
    InstanceCase instance_case = instance.cases().front();

    chronometer chrono;

    std::cout << "number of vertices " << melon::nb_vertices(instance_case.graph()) << std::endl;
    std::cout << "number of arcs " << melon::nb_arcs(instance_case.graph()) << std::endl;

    const double seq_pc_num = fhamonic::gecot::pc_num(
        instance_case.graph(), instance_case.vertex_quality_map(),
        instance_case.arc_probability_map());
    std::cout << "seq PC_NUM = " << seq_pc_num << " in "
              << static_cast<double>(chrono.lap_time_us()) / 1000.0 << " ms" << std::endl;

    const double par_pc_num = fhamonic::gecot::parallel_pc_num(
        instance_case.graph(), instance_case.vertex_quality_map(),
        instance_case.arc_probability_map());
    std::cout << "par PC_NUM = " << par_pc_num << " in "
              << static_cast<double>(chrono.lap_time_us()) / 1000.0 << " ms" << std::endl;

    std::cout << "number of options " << instance.options().size() << std::endl;

    gecot::solvers::StaticIncremental static_incr;
    auto solution = static_incr.solve(instance, 10000);

    std::cout << "found "
              << gecot::compute_solution_score(instance, solution)
              << " in " << chrono.lap_time_ms() << " ms" << std::endl;

    return EXIT_SUCCESS;
}
