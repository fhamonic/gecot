#pragma once

#include <cstddef>

#include <spdlog/spdlog.h>

#include "mippp/model_concepts.hpp"
#include "mippp/utility/column_manager.hpp"

namespace gecot {
namespace solvers {

// What path_lp and tree_lp share of their column generation
// (docs/formulations/relaxations.md).

// The pricing test, at the master's dual tolerance: a column enters if its
// reduced cost is above this margin, sigma being the dual of the uniqueness
// row of its pair or target.
inline double pricing_margin(const double sigma) { return 1e-7 * sigma + 1e-6; }

// The columns in the master for more than evict_age rounds and unattractive,
// so nonbasic. Warns that --evict-age is ignored on a solver that cannot
// remove columns.
template <typename LP>
auto unattractive_columns(const std::size_t evict_age) {
    namespace colgen = mippp::colgen;
    if constexpr(!mippp::has_remove_variable<LP>)
        if(evict_age)
            spdlog::warn(
                "This solver cannot remove columns: --evict-age ignored");
    return colgen::conjunction<colgen::above<colgen::age>,
                               colgen::negative<colgen::reduced_cost>>{
        colgen::above<colgen::age>{evict_age},
        colgen::negative<colgen::reduced_cost>{1e-6}};
}

}  // namespace solvers
}  // namespace gecot
