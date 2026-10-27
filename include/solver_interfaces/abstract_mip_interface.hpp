#pragma once

#include <filesystem>
#include <fstream>
#include <vector>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include "gecot/solvers/mip_solver_base.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class AbstractMipInterface : public AbstractSolverInterface {
public:
    using AbstractSolverInterface::AbstractSolverInterface;

    static std::vector<std::string> parse_mip_start(
        const std::filesystem::path & solution_path) {
        if(!std::filesystem::exists(solution_path))
            throw std::invalid_argument("File '" + solution_path.string() +
                                        "' does not exists");
        if(std::filesystem::is_directory(solution_path))
            throw std::invalid_argument("'" + solution_path.string() +
                                        "' is a directory");

        std::ifstream instance_stream(solution_path);
        nlohmann::json solution_json;
        instance_stream >> solution_json;

        std::vector<std::string> taken_option_names;
        for(auto && [option_name, value] : solution_json["solution"].items()) {
            if(!value.template get<int>()) continue;
            taken_option_names.emplace_back(option_name);
        }
        return taken_option_names;
    }

protected:
    void add_mip_options(gecot::solvers::mip_solver_base & solver) {
        // Gurobi's default: below it, Gurobi discards the completed
        // --mip-start of prep_mip on large instances (residuals ~1e-6)
        add_feasibility_tol_option(solver.feasibility_tol, 1e-6);
        app.add_option_function<std::filesystem::path>(
            "--mip-start",
            [&solver](const std::filesystem::path & solution_path) {
                solver.mip_start = parse_mip_start(solution_path);
            },
            "Solution file (JSON) to propose as a MIP start to the solver.");
        app.add_option("--time-limit", solver.time_limit,
                       "Time limit in seconds, after which the best solution "
                       "found is returned")
            ->check(CLI::NonNegativeNumber);
    }

    void add_preprocessing_options(gecot::solvers::mip_solver_base & solver) {
        app.add_option("--resolution", solver.probability_resolution,
                       "Sets the resolution of the probabilities used by the "
                       "preprocessing algorithm")
            ->default_val(1e-8);
        app.add_option("--nb-mus", solver.num_mus,
                       "Sets the number of lagrangian multipliers to test for "
                       "the constrained preprocessing algorithm (experimental)")
            ->default_val(0);
    }
};
