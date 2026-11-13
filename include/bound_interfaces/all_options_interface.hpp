#pragma once

#include "gecot/bounds/all_options.hpp"

#include "bound_interfaces/abstract_bound_interface.hpp"

class AllOptionsInterface : public AbstractBoundInterface {
public:
    AllOptionsInterface() : AbstractBoundInterface("all_options") {}

    gecot::bound_report compute(const Instance & instance,
                                const gecot::budget_t &,
                                const bound_context &) const override {
        return gecot::all_options_bound(instance);
    }

    std::string description() const override {
        return "The score with every option bought; needs a criterion "
               "non-decreasing in the case scores.";
    }
};
