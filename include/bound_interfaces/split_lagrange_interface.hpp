#pragma once

#include <optional>
#include <string>

#include "gecot/bounds/split_lagrange.hpp"

#include "bound_interfaces/abstract_bound_interface.hpp"

class SplitLagrangeInterface : public AbstractBoundInterface {
private:
    gecot::split_lagrange method;
    std::optional<double> target;  // raw score

public:
    SplitLagrangeInterface() : AbstractBoundInterface("split_lagrange") {
        app.add_option("--iterations", method.iterations,
                       "Subgradient steps on the prices, each a pricing of "
                       "every pair of habitats")
            ->default_val(method.iterations);
        app.add_option("--time-limit", method.time_limit,
                       "Time limit in seconds, setup included, after which "
                       "the best bound of the complete steps is returned; for "
                       "each case a non-linear criterion reads")
            ->check(CLI::NonNegativeNumber);
        add_target_option(target,
                          "Score the steps aim at, the best known solution's "
                          "for instance (default: the score of the first "
                          "knapsack solution's whole options)");
        using theta_start = gecot::qkp_planes::theta_start;
        add_choice_option<theta_start>(
            app, "--theta", method.vertex_side.start,
            {{"best", theta_start::best},
             {"half", theta_start::half},
             {"target", theta_start::target},
             {"source", theta_start::source}},
            [this](const theta_start start) {
                method.vertex_side.start = start;
            },
            "Split of the quadratic terms of the quality gains, as for "
            "qkp_planes");
    }

    gecot::bound_report compute(const Instance & instance,
                                const gecot::budget_t & B,
                                const bound_context & context) const override {
        gecot::split_lagrange m = method;
        if(target) m.target = target.value() / context.score_scale;
        return m.bound(instance, B);
    }

    std::string description() const override {
        return "Lagrangian bound pricing each pair of habitats' paths, on top "
               "of qkp_planes' bounds of the quality gains; needs every "
               "option's improved arcs to share a vertex, and each vertex to "
               "be entered by improved arcs of one option.";
    }
};
