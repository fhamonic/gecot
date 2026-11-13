# Parameters

Every parameter of GECOT 2.0, as `gecot optimize --help`, `gecot bound --help` and `--list-params` print them. A value in brackets is the default.

## Commands

```text
gecot --help
gecot --version
gecot optimize --help
gecot optimize --list-algorithms
gecot optimize <algorithm> --list-params
gecot optimize <algorithm> <instance> <budget> [<parameters> ...]
gecot bound --help
gecot bound --list-methods
gecot bound <method> --list-params
gecot bound <method> <instance> <budget> [<parameters> ...]
```

### `gecot optimize`

| Option | Meaning |
|---|---|
| `-a`, `--algorithm <name>` | the algorithm, also the first positional argument |
| `-i`, `--instance <file>` | the instance JSON file, also the second positional argument |
| `-B`, `--budget <value>` | the budget, also the third positional argument |
| `-A`, `--list-algorithms` | lists the algorithms |
| `-P`, `--list-params` | lists the parameters of the chosen algorithm |
| `-o`, `--output-json <file>` | writes the result as JSON |
| `--output-csv <file>` | writes the selection as CSV |
| `--reformulation trivial`, `path_lp` [`trivial`] | the instance given to the algorithm. `trivial`: without the options of cost 0 outside the conflict sets, which are bought, and those over the budget. `path_lp`: also without the options that the path relaxation proves to be in no better solution than its own |
| `--reformulation-params "<name>=<value> ..."` | parameters of the `path_lp` of `--reformulation path_lp`, as `"time-limit=60 pricing=enumerate"`; its MIP start is the solution of `greedy` by default |
| `--no-merging` | for comparison: keeps apart the vertices that arcs of probability 1 link |
| `-v`, `--verbose` | logs the details of the algorithm; `-vv` everything, with the solver's own log on the standard output |
| `-q`, `--quiet` | logs only the errors |
| `--max-threads <n>` | limits the number of threads of GECOT's own computations |
| `-h`, `--help` | prints the help |

### `gecot bound`

| Option | Meaning |
|---|---|
| `--method <name>` | the method, also the first positional argument |
| `-i`, `--instance <file>` | the instance JSON file, also the second positional argument |
| `-B`, `--budget <value>` | the budget, also the third positional argument |
| `-M`, `--list-methods` | lists the methods |
| `-P`, `--list-params` | lists the parameters of the chosen method |
| `-o`, `--output-json <file>` | writes the result as JSON |
| `-v`, `--verbose`, `-q`, `--quiet`, `--max-threads <n>`, `-h`, `--help` | as for `gecot optimize` |

## Which algorithm takes what

| Algorithm | Common | Preprocessing | Own parameters |
|---|:-:|:-:|---|
| `flow_mip` | ✓ | ✓ | `--big-m`, `--lp-relaxation` |
| `multicut_benders` | ✓ | ✓ | `--groups`, `--grouping`, `--grouping-seed`, `--big-m` |
| `flow_benders` | ✓ | ✓ | `--big-m` |
| `multiflow_benders` | ✓ | ✓ | `--no-root-cuts`, `--no-in-out` |
| `tree_lp` | ✓ | ✓ | `--evict-age` |
| `path_lp` | ✓ | – | `--evict-age`, `--pricing`, `--max-activations`, `--relative-gap` |
| `static` | – | – | `-t`, `--decremental`, `--only-dec` |
| `greedy` | – | – | `-t`, `--decremental`, `--only-dec`, `--worst` |

## Common to the MILP-based algorithms

| Parameter | Default | Meaning |
|---|---|---|
| `--solver <name>` | `gurobi` | the solver, used through mippp. `flow_mip` and `path_lp`: `gurobi`, `cplex`, `copt`, `xpress`, `mosek`, `highs`, `scip`, `cbc`, `glpk`. `tree_lp`: the same without `glpk`. Benders decompositions: `gurobi`, `cplex`, `copt` |
| `-t`, `--feasibility-tolerance <value>` | `1e-07` | the tolerance for rounding errors: the solver's feasibility tolerance, and the relative error accepted on a candidate of a Benders decomposition |
| `--mip-start <file>` | | a solution file (JSON, as written by `-o`) to start the solver from. Excludes `--mip-start-algorithm` |
| `--mip-start-algorithm <name>` | `none` | the heuristic whose solution starts the solver, run in its incremental form: `none`, `static`, `greedy`. Its time counts in `--time-limit`. Excludes `--mip-start` |
| `--time-limit <seconds>` | | after this time, the best solution found is returned |

See [Optimizing](../usage/optimize.md#parameters-of-the-algorithms).

## Preprocessing

For `flow_mip`, `multicut_benders`, `flow_benders`, `multiflow_benders` and `tree_lp`. See [Preprocessing](../formulations/preprocessing.md).

| Parameter | Default | Meaning |
|---|---|---|
| `--resolution <value>` | `1e-08` | the [resolution](../formulations/preprocessing.md#resolution) of the probabilities in the labelling of the strong and useless arcs. Strictly between 0 and 1, and not so small that the integer lengths of a path overflow: other values are refused |
| `--nb-mus <n>` | `0` | the number of geometric steps between the Lagrange multipliers of the [budget-aware labelling](../formulations/preprocessing.md#taking-the-budget-into-account), which tests $n + 1$ positive multipliers after 0. `0`: no budget-aware labelling. Experimental: 20, or 9 on large instances |
| `--mus-sweep decades`, `full` | `decades` | the range of the positive multipliers, up to the largest useful one. `decades`: at most the 3 decades below it, and 0 again at the end. `full`: from the inverse of the largest option cost per arc. Needs `--nb-mus` above 0. Experimental |
| `--preprocessing all`, `strong`, `useless`, `none` | `all` | for comparison, what the labels are used for. `strong`: only the contraction of the strong arcs. `useless`: only the removal of the useless arcs. `none`: neither; `--resolution`, `--nb-mus` and `--mus-sweep` then have no effect |
| `--all-targets` | off | for comparison: the vertices that can never be targets get a flow like the others |
| `--no-pruning` | off | for comparison: keeps, in the graph of a target, the arcs that leave it and the vertices that no quality reaches |

## Parameters of each algorithm

### `flow_mip`

| Parameter | Default | Meaning |
|---|---|---|
| `--big-m plain`, `knapsack_bnb`, `knapsack_dantzig`, `tight` | `knapsack_bnb` | the [big-M constants](../formulations/preprocessing.md#big-m-constants). `plain`: with every quality gain, for comparison. `knapsack_bnb`: with the gains of the best options within the budget, by branch and bound. `knapsack_dantzig`: with the linear relaxation of that knapsack, looser but faster to compute. `tight`: experimental, lowered per arc over the sources that can use it |
| `--lp-relaxation` | off | experimental: solves the [continuous relaxation](../formulations/flow.md#what-flow_mip-solves) |

### `multicut_benders`

| Parameter | Default | Meaning |
|---|---|---|
| `--groups <n>` | `2` | the groups of targets per case, each with its own cuts. `1`: a single cut per case. `0`: a group per target |
| `--grouping voronoi`, `consecutive`, `random` | `voronoi` | how the targets are grouped. `voronoi`: the cells of a network Voronoi diagram around targets of value far from each other. `consecutive`: runs of consecutive targets. `random`: at random, as a control |
| `--grouping-seed <n>` | `0` | the seed of `--grouping random` |
| `--big-m plain`, `knapsack_bnb`, `knapsack_dantzig`, `tight` | `knapsack_bnb` | as for `flow_mip` |

### `flow_benders`

| Parameter | Default | Meaning |
|---|---|---|
| `--big-m plain`, `knapsack_bnb`, `knapsack_dantzig`, `tight` | `knapsack_bnb` | as for `flow_mip` |

### `multiflow_benders`

| Parameter | Default | Meaning |
|---|---|---|
| `--no-root-cuts` | off | for comparison: no cut on the linear relaxation before the search |
| `--no-in-out` | off | for comparison: separates each optimum of the relaxation, without the center that moves halfway to it |

### `path_lp`

| Parameter | Default | Meaning |
|---|---|---|
| `--pricing search`, `enumerate` | `search` | `search`: a search per pair and round, without the budget. `enumerate`: the columns of each pair once, within the budget, for few options per path |
| `--relative-gap <g>` | `0` | stops once the Lagrangian bound is within this gap of the value of the relaxation, relative to it. `0`: at convergence only |
| `--max-activations <n>` | `0` | the columns that enter the master per round, the best ones. `0`: every improving column |
| `--evict-age <n>` | `0` | the rounds a column stays in the master before it can go back to the pool, if unattractive. `0`: never |

### `tree_lp`

| Parameter | Default | Meaning |
|---|---|---|
| `--evict-age <n>` | `0` | as for `path_lp` |

### Heuristics

| Parameter | Default | Meaning |
|---|---|---|
| `-t`, `--feasibility-tolerance <value>` | `1e-07` | the slack allowed when a cost is compared with the budget |
| `--decremental` | off | starts from every option bought and removes options until the cost fits the budget, then buys back those that fit in the budget left |
| `--only-dec` | off | with `--decremental`: skips these final purchases |
| `--worst` | off | `greedy`, for comparison: the opposite choice at each step, which spends the budget while gaining as little as possible |

## Parameters of the bound methods

`all_options` has no parameter. `flow_mip`, `multicut_benders`, `flow_benders`, `multiflow_benders` and `path_lp` take the parameters of the algorithm of the same name.

### `qkp_planes`

| Parameter | Default | Meaning |
|---|---|---|
| `--theta best`, `half`, `target`, `source` | `best` | the split of the quadratic terms between the two options of a pair: half each, all to the one with the target gain, all to the one with the source gain, or the best of the three |
| `--iterations <n>` | `0` | subgradient steps on the split after it |
| `--target <score>` | 0.9 times the bound | the score that the subgradient steps aim at, that of a known solution for instance |

### `split_lagrange`

| Parameter | Default | Meaning |
|---|---|---|
| `--iterations <n>` | `30` | subgradient steps on the prices, each a pricing of every pair of habitats |
| `--time-limit <seconds>` | | setup included; the best bound of the complete steps is returned |
| `--target <score>` | the score of a first knapsack solution | the score that the steps aim at, that of the best known solution for instance |
| `--theta best`, `half`, `target`, `source` | `best` | the split of the quadratic terms of the quality gains, as for `qkp_planes` |

## Environment variables

| Variable | Meaning |
|---|---|
| `MIPPP_<SOLVER>_LIBRARY` | the exact file of a solver library, see [how a solver library is found](../getting-started/installation.md#how-a-solver-library-is-found) |
| `MIPPP_NO_VERSION_WARNING` | when set, silences the warning about a solver version outside the validated range |
