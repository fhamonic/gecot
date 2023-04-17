#ifndef ABSTRACT_RANKER_HPP
#define ABSTRACT_RANKER_HPP

#include <string>
#include <vector>

#include "instance.hpp"

namespace fhamonic {

class AbstractRanker {
public:
    virtual ~AbstractRanker() {}

    virtual void parse(const std::vector<std::string> & args) = 0;
    virtual typename Instance::OptionPotentialMap rank_options(
        const Instance & instance) const = 0;

    virtual std::string name() const = 0;
    virtual std::string description() const = 0;
    virtual std::string options_description() const = 0;
    virtual std::string string() const = 0;
};

}  // namespace fhamonic

#endif  // ABSTRACT_RANKER_HPP