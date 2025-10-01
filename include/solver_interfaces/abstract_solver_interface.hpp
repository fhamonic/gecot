#pragma once

#include <string>
#include <vector>

#include "instance.hpp"

namespace fhamonic {

class AbstractSolverInterface {
public:
    virtual ~AbstractSolverInterface() {}

    virtual void parse(const std::vector<std::string> & args) = 0;
    virtual gecot::instance_solution_t<Instance> solve(
        const Instance & instance, const double B) const = 0;

    virtual std::string name() const = 0;
    virtual std::string description() const = 0;
    virtual std::string options_description() const = 0;
    virtual std::string string() const = 0;
};

}  // namespace fhamonic
