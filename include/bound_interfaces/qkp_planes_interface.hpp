#pragma once

#include <optional>
#include <string>

#include "gecot/bounds/qkp_planes.hpp"

#include "bound_interfaces/abstract_bound_interface.hpp"

class QkpPlanesInterface : public AbstractBoundInterface {
private:
    gecot::qkp_planes method;
    std::optional<double> target;  // raw score

public:
    QkpPlanesInterface() : AbstractBoundInterface("qkp_planes") {
        app.add_option_function<std::string>(
               "--theta",
               [this](const std::string & s) {
                   using enum gecot::qkp_planes::theta_start;
                   method.start = s == "half"     ? half
                                  : s == "target" ? target
                                  : s == "source" ? source
                                                  : best;
               },
               "Split of the quadratic terms between the two options of a "
               "pair: half each, all to the one with the target gain, all to "
               "the one with the source gain, or the best of the three")
            ->check(CLI::IsMember({"best", "half", "target", "source"}))
            ->default_val("best");
        app.add_option("--iterations", method.iterations,
                       "Subgradient steps on the split after it")
            ->default_val(0);
        add_target_option(target,
                          "Score the subgradient steps aim at, a known "
                          "solution's for instance (default: 0.9 times the "
                          "bound)");
    }

    gecot::bound_report compute(const Instance & instance,
                                const gecot::budget_t & B,
                                const bound_context & context) const override {
        gecot::qkp_planes m = method;
        if(target) m.target = target.value() / context.score_scale;
        return m.bound(instance, B.front());
    }

    std::string description() const override {
        return "Upper planes of the quadratic knapsack bounding the score "
               "with every probability at its value with all the options "
               "bought; as all_options without quality gains.";
    }
};
