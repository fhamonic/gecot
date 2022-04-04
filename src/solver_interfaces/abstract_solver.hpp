#ifndef ABSTRACT_SOLVER_HPP
#define ABSTRACT_SOLVER_HPP

#include <string>
#include <vector>

#include "instance.hpp"

namespace fhamonic {

class AbstractSolver {
public:
    virtual void parse(const std::vector<std::string> & args) = 0;
    virtual typename Instance::Solution solve(const Instance & instance,
                                              const double B) const = 0;
    virtual std::string name() const = 0;
    virtual std::string description() const = 0;
    virtual std::string params_lists() const = 0;
    virtual std::string string() const = 0;
};

}  // namespace fhamonic

#endif  // ABSTRACT_SOLVER_HPP