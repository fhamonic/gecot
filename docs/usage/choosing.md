# Choosing an algorithm

The algorithms of GECOT differ in what they promise. Start from what you need:

- **the best selection, with a proof**: an exact algorithm;
- **a good selection, with a measure of how good**: an exact algorithm under a time limit, or `path_lp`;
- **a selection, quickly, on any landscape**: a heuristic, to be [certified afterwards](bound.md#certifying-a-solution) if needed.

## At a glance

| Algorithm | Returns | Use it |
|---|---|---|
| `multiflow_benders` | an optimal solution, with its proof | by default, if you have Gurobi, CPLEX or COPT |
| `multicut_benders` | an optimal solution, with its proof | as an alternative when `multiflow_benders` struggles |
| `flow_benders` | an optimal solution, with its proof | mostly as a point of comparison |
| `flow_mip` | an optimal solution, with its proof | with any other solver, or as the reference model |
| `path_lp` | a solution and the upper bound of a relaxation | when the exact algorithms do not end, or when a bound is enough |
| `greedy` | a solution, without guarantee | for a quick solution, a MIP start, a very large landscape |
| `static` | a solution, without guarantee | as the ranking method to compare with |
| `tree_lp` | a relaxation, possibly over the budget | to study the tree relaxation |
| `greedy --worst` | a deliberately bad solution | as a worst-case reference |

With [conflict sets](instance.md#conflict-sets) or [several costs](instance.md#several-costs), only `multicut_benders`, `flow_benders` and `flow_mip` run for now; the others stop with an error.

"Exact" always means up to the [tolerances](../formulations/flow.md#what-flow_mip-solves) of the solver and the [resolution](../formulations/preprocessing.md#resolution) of the preprocessing. Under `--time-limit` or after Ctrl-C, the exact algorithms return their best solution with the bound proved so far.

## An example

One instance does not make a benchmark, but it gives orders of magnitude. On the 40 × 40 raster landscape of the repository (`test/instances/Aix/instance_40x40`: 4,800 vertices, 9,440 arcs, 27 options) at budget 8, with Gurobi on an 8-core desktop computer, one run each:

| Algorithm | Score | Guaranteed share of the best gain | Time |
|---|---:|---:|---:|
| `static --decremental` | 418,296.9 | – | 42 ms |
| `static` | 432,401.8 | – | 28 ms |
| `greedy` | 437,866.4 | – | 0.2 s |
| `greedy --decremental` | 437,866.4 | – | 0.5 s |
| `path_lp` | 437,866.4 | 98.5 % | 4.0 s |
| `flow_benders` | 438,893.1 | optimal | 9.0 s |
| `flow_mip` | 438,893.1 | optimal | 6.6 s |
| `multicut_benders` | 438,893.1 | optimal | 1.4 s |
| `multiflow_benders` | 438,893.1 | optimal | 1.2 s |

`greedy` is within 1.3 % of the optimal gain here, in both directions, which only the exact algorithms can tell.

## Guidance

**You have Gurobi, CPLEX or COPT.** Run `multiflow_benders`, and `multicut_benders` if it struggles. Their master program has one variable per option and a few per target, whatever the size of the graphs, where `flow_mip` carries every flow in its model: they usually need less memory and less time.

**You only have an open-source solver.** `flow_mip --solver highs` is the exact algorithm, and `path_lp --solver highs` gives a solution with a bound. The Benders decompositions need lazy constraints, which GECOT only has for the three solvers above.

**The exact algorithm does not end.** Give it a time limit and a starting solution:

```console
$ gecot optimize multiflow_benders instance.json 100 \
      --mip-start-algorithm greedy --time-limit 3600
```

The result is at least as good as the heuristic's and comes with a guaranteed share of the best possible gain. If the bound stays loose, try `path_lp`, whose bound is proved from its first round, and [`--reformulation path_lp`](optimize.md#removing-options-first), which lets the relaxation remove options before the exact algorithm runs.

**The landscape is too large for any model.** Use `greedy`. Its cost grows with the number of options squared times the cost of evaluating the score, and it needs no solver and little memory. A [bound](bound.md) such as `qkp_planes` then costs nothing, although it is loose when the budget is small.

**The budget is very small.** Few options fit together, which the budget-aware preprocessing exploits: try [`--nb-mus 20`](../formulations/preprocessing.md#taking-the-budget-into-account) with `flow_mip` or a Benders decomposition, and compare.

**Several species with a minimum criterion.** Prefer an exact algorithm. A heuristic ranks the options by the gain of the criterion, and with a minimum only the options that help the worst-off case have any: its choices among the others are arbitrary.

**You compare with a ranking method.** `static` is the ranking of the options by their individual gain, `greedy` its iterated version, and the gap to the optimum measures what the interactions between the options are worth on your landscape.

## What the solvers support

Every solver can run `flow_mip` and `path_lp`, but not with the same features. GECOT warns when a parameter asks for a feature that the solver lacks.

| Solver | Algorithms | `--mip-start` | Gap of $10^{-10}$ | Bound in the result | `--evict-age` |
|---|---|:-:|:-:|:-:|:-:|
| Gurobi | all | ✓ | ✓ | ✓ | ✓ |
| CPLEX | all | ✓ | ✓ | ✓ | ✓ |
| COPT | all | ✓ | ✓ | ✓ | – |
| Xpress | `flow_mip`, `path_lp`, `tree_lp` | ✓ | ✓ | ✓ | – |
| MOSEK | `flow_mip`, `path_lp`, `tree_lp` | ✓ | – | ✓ | – |
| HiGHS | `flow_mip`, `path_lp`, `tree_lp` | ✓ | ✓ | ✓ | ✓ |
| SCIP | `flow_mip`, `path_lp`, `tree_lp` | – | ✓ | ✓ | ✓ |
| CBC | `flow_mip`, `path_lp`, `tree_lp` | ✓ | ✓ | ✓ | ✓ |
| GLPK | `flow_mip`, `path_lp` | – | – | – | – |

Every solver honours `--time-limit`. Without the optimality gap setting, a solver stops at its own default gap, and `optimal` then means optimal within this gap. Ctrl-C reaches a solver when it next reports its progress: within seconds with Gurobi, later with HiGHS, never with CBC.
