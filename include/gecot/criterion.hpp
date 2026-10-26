#ifndef GECOT_CRITERION_HPP
#define GECOT_CRITERION_HPP

#include <algorithm>
#include <functional>
#include <limits>
#include <variant>
#include <vector>

#include "gecot/concepts/instance.hpp"

namespace gecot {

using criterion_constant = double;
using criterion_var = case_id_t;
struct criterion_sum;
struct criterion_product;
struct criterion_min;

using criterion_formula =
    std::variant<criterion_constant, criterion_var, criterion_sum,
                 criterion_product, criterion_min>;

struct criterion_sum {
    std::vector<criterion_formula> values;
};
struct criterion_product {
    std::vector<criterion_formula> values;
};
struct criterion_min {
    std::vector<criterion_formula> values;
};

template <typename M>
struct formula_eval_visitor {
    std::reference_wrapper<const M> value_map;

    formula_eval_visitor(const M & t_value_map) : value_map{t_value_map} {}

    double operator()(const criterion_constant & c) { return c; }
    double operator()(const criterion_var & v) { return value_map.get()[v]; }
    double operator()(const criterion_sum & f) {
        double sum = 0.0;
        for(auto && e : f.values) sum += std::visit(*this, e);
        return sum;
    }
    double operator()(const criterion_product & f) {
        double product = 1.0;
        for(auto && e : f.values) product *= std::visit(*this, e);
        return product;
    }
    double operator()(const criterion_min & f) {
        double min = std::numeric_limits<double>::max();
        for(auto && e : f.values) min = std::min(min, std::visit(*this, e));
        return min;
    }
};

}  // namespace gecot

#endif  // GECOT_CRITERION_HPP
