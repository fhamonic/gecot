# Contributing to GECOT

This page is for whoever changes the code. It says where the mathematics is stated, how a solver file is written, what a comment is for, and how to check that a change that should preserve the results does preserve them.

## The documentation states the mathematics, the code builds it

- The formulations are in `docs/formulations/`. A program, a cut or a bound is derived there and nowhere else: a source file has no formulation block in its comments.
- The notation is the one of `docs/reference/notation.md`, which follows the thesis and the article. The solvers, the preprocessing and the bounds use it: an option is `i`, a case is `k`, a cost dimension is `r`, and a variable of a model is named after its symbol (`x(i)`, `z(k)`, `f_t`, `phi(a)`). `i` is kept for the options, and for their items in the knapsacks of the bounds. The command layer, `Instance` and the pricing model of `tree_lp` have not been renamed yet.
- A file, a struct and an interface are named after the algorithm on the command line: `flow_mip` is `include/gecot/solvers/flow_mip.hpp`, `gecot::solvers::flow_mip` and `FlowMipInterface`. `Static` and `Greedy` keep a capital, as `static` is a keyword.
- When the code and a docs page disagree, one of them is wrong. Fix it in the same change.

## How a solver file is written

`include/gecot/solvers/flow_mip.hpp` is the reference.

- The calls that build a model (`add_variables`, `add_constraints`, `add_column`, `add_lazy_constraint`) stay inline in `solve()`, in the order the solver receives them. No function builds rows, except the two helpers of the base that every MILP shares: `formula_variable_visitor`, for the rows of the linearized criterion, and `_add_budget_rows`, for the budget and conflict rows.
- They form blocks. A block is preceded by one comment that names the section of the docs page it builds and says what it builds.
- What is not the model is kept out of the way, in the base (`mip_solver_base.hpp`) or in a helper above `solve()`: the preprocessing table, the statistics, the MIP start, the readback of the solution.
- A constant has a name next to the reason for its value. It is shared only where two sites must agree.

## Comments

A comment names a mistake that a reader could make and that the code alone would not prevent. Three kinds qualify:

- a **contract**: a precondition, or what a caller must have done, that no type checks;
- a **trap**: a choice that looks wrong and that a maintainer would "fix", with the failure that follows;
- a **non-obvious mechanic**, in a sentence or two.

A comment that restates the line below it, reassures that the code is right, or tells how the code used to be, is removed. The history goes in the commit message. A property that a `static_assert` or a test pins needs no comment.

## Checking a change that must preserve the results

The rule: with one cost dimension and no conflict set, a refactoring leaves the results bitwise identical, and every model receives the same variables, rows and columns in the same order with the same coefficients. A rename is free. Moving a model call past another one is not, and rewriting an expression can change its last bits or the order of the terms of a row.

Capture the binary before and after the change, on the same machine, with the same compiler, solver and dependency revisions and the default release flags, and compare:

```console
$ python3 test/parity/capture.py build/gecot test/parity/captures/before -j 4
  ... the change, the build ...
$ python3 test/parity/capture.py build/gecot test/parity/captures/after -j 4
$ python3 test/parity/compare.py test/parity/captures/before test/parity/captures/after
... cases, 0 differ
```

A capture runs every algorithm and bound on small instances and keeps what they print, their logs, their result files, and for the runs that end in Gurobi a hash of the model. The hash does not see the order of the terms inside a row, nor the cuts added during a search: check these two by reading the diff. The docstrings of the two scripts say what else a capture does not see. `compare.py --ignore REGEX` drops the lines of a new parameter. The capture needs Gurobi. The captures stay on the machine: `test/parity/captures` is ignored by git.

Then:

```console
$ ./build/test/gecot_test
$ python3 test/brute_force/check.py build/gecot
```

The second one compares the algorithms and the bounds with a brute-force optimum on small instances. A change that is meant to alter results states which runs change and why, in its commit message.

## Adding an algorithm or a parameter

| Change | Where |
|---|---|
| An algorithm | its header in `include/gecot/solvers`; an interface header in `include/solver_interfaces`, which overrides `bound_description()` if a run proves an upper bound: `gecot bound` then lists the algorithm as a method; for a MILP-based one, an interface `.cpp` in `src/solver_interfaces` and its line in `CMakeLists.txt`; `src/commands/registry.cpp`; its docs page, `docs/reference/parameters.md` and `docs/usage/choosing.md`; `test/brute_force/check.py` and the matrix of `test/parity/capture.py` |
| A parameter | the member with its default, which is the only place that states it; the option in the interface, with `->default_val(member)`: CLI11 writes the default of an option into its member, so a literal there would replace the one of the struct for every run of the command and leave it to a caller of the library; a choice among names goes through `add_choice_option`, which reads its default from the member too; the docs tables |
| A bound | its header in `include/gecot/bounds`, an interface header in `include/bound_interfaces`, `registry.cpp`; `docs/formulations/bounds.md`, `docs/usage/bound.md` and `docs/reference/parameters.md`; `test/bounds_test.cpp`, `check.py` and the lists `CHEAP_BOUNDS`, `BOUNDS` and `METHOD_NAMES` of `test/parity/capture.py` |

## Building and formatting

- Each MILP interface is its own translation unit, because it instantiates its solver for every backend: compiling one takes 2 to 2.5 GB of memory. Limit the parallel jobs accordingly: `-c tools.build:jobs=2` on the `conan build` command, or `make -j2` inside `build/`.
- Format the lines you change, not the files: `git clang-format` on the staged changes. `clang-format -i`, which `make format` runs on every file, reflows untouched lines.
