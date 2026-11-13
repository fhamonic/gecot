# Optimizing

`gecot optimize` selects the options to buy within a budget.

```console
$ gecot optimize <algorithm> <instance> <budget> [<parameters> ...]
```

| Argument | Meaning |
|---|---|
| `<algorithm>` | one of the algorithms listed by `gecot optimize --list-algorithms` |
| `<instance>` | the [instance file](instance.md), a `.json` file |
| `<budget>` | a non-negative number, in the unit of the costs of the options; for options with [several costs](instance.md#several-costs), `name=value,...` with each dimension once, as `money=5,area=3` (also accepted with one dimension, as `cost=5`) |
| `<parameters>` | the options of the command and the parameters of the algorithm, in any order |

The three arguments can also be given by name, with `-a`, `-i` and `-B`. `gecot optimize --help` prints the options of the command, and `gecot optimize <algorithm> --list-params` the parameters of an algorithm.

## Algorithms

```console
$ gecot optimize --list-algorithms
```

| Algorithm | What it is | Guarantee |
|---|---|---|
| `flow_mip` | the [flow formulation](../formulations/flow.md), solved by a MILP solver | optimal |
| `multicut_benders` | [Benders decomposition](../formulations/benders.md#multicut_benders-cuts-on-groups-of-targets) with a cut per group of targets | optimal |
| `flow_benders` | [Benders decomposition](../formulations/benders.md#flow_benders-cuts-on-the-flows) with a cut per target flow | optimal |
| `multiflow_benders` | [Benders decomposition](../formulations/benders.md#multiflow_benders-cuts-per-source) with cuts per source and target | optimal |
| `path_lp` | the [path relaxation](../formulations/relaxations.md), by column generation | a solution and an upper bound |
| `tree_lp` | the [tree relaxation](../formulations/relaxations.md#the-tree-formulation), by column generation | none: can exceed the budget |
| `static`, `greedy` | the [heuristics](../formulations/heuristics.md), incremental or with `--decremental` | none |

[Choosing an algorithm](choosing.md) compares them.

## Options of the command

| Option | Meaning |
|---|---|
| `-o`, `--output-json <file>` | writes the result as [JSON](results.md#json) instead of printing it |
| `--output-csv <file>` | writes the selection as [CSV](results.md#csv) instead of printing the result |
| `--reformulation trivial` or `path_lp` | the instance given to the algorithm, see [below](#removing-options-first) (`trivial` by default) |
| `--reformulation-params "<name>=<value> ..."` | the parameters of the `path_lp` of `--reformulation path_lp` |
| `--no-merging` | for comparison: does not [merge](../formulations/preprocessing.md#the-instance-given-to-the-algorithms) the vertices linked by arcs of probability 1 |
| `-v`, `--verbose` | logs the parameters and the details of the algorithm; `-vv` logs everything, with the solver's own log |
| `-q`, `--quiet` | logs only the errors |
| `--max-threads <n>` | limits the number of threads of GECOT's own computations |

`--max-threads` bounds the parallel loops of GECOT: the evaluations of the heuristics, the preprocessing, the pricing and the separation of cuts. The MILP solver keeps its own setting for the search itself.

## Parameters of the algorithms

The parameters of each algorithm are listed on the [parameters page](../reference/parameters.md). The ones below are common to all the algorithms that use a solver: `flow_mip`, the three Benders decompositions, `path_lp` and `tree_lp`.

### Solver

`--solver <name>` selects the solver, `gurobi` by default: see [which solver each algorithm accepts](../getting-started/installation.md#solvers) and how its library is found.

```console
$ gecot optimize flow_mip instance.json 100 --solver highs
```

Not every solver offers every feature. When one is missing, GECOT logs a warning and goes on without it: a solver without a time limit ignores `--time-limit`, one without MIP start ignores `--mip-start`, and one that reports no dual bound gives a result without upper bound, as GLPK does.

### Time limit

`--time-limit <seconds>` stops the algorithm and returns the best solution found so far, with the status `time limit reached` and the upper bound proved so far.

```console
$ gecot optimize multicut_benders instance.json 100 --time-limit 600
```

- The time counts from the start of the algorithm: the preprocessing, the construction of the model and the computation of a MIP start by `--mip-start-algorithm` are included. Reading the instance is not.
- The column generations of `path_lp` and `tree_lp` stop at the end of their current round, so they can run a little over.
- If the solver has found no solution when the time runs out, GECOT returns the MIP start if there is one, with a warning, and fails otherwise. With a short limit on a large instance, give a MIP start.
- The heuristics have no time limit.

Ctrl-C has the same effect as the time limit running out, at any moment: see [Results and logs](results.md#ctrl-c).

### MIP start

A solver searches faster from a good solution. Two parameters, which exclude each other, provide one:

- `--mip-start-algorithm <heuristic>` runs `static` or `greedy` first, in its incremental form, and starts from its solution;
- `--mip-start <file>` starts from a solution file written by `-o`, for instance by a previous run with another algorithm or a smaller time limit. Its options that the instance does not have are ignored.

```console
$ gecot optimize flow_mip instance.json 100 --mip-start-algorithm greedy
$ gecot optimize greedy instance.json 100 --decremental -o greedy.json
$ gecot optimize flow_mip instance.json 100 --mip-start greedy.json
```

The start also seeds the cuts of the Benders decompositions and the columns of the relaxations, and it is what `path_lp` returns if it finds nothing better.

### Tolerance

`-t`, `--feasibility-tolerance <value>` ($10^{-7}$ by default) is the solver's feasibility tolerance. In the Benders decompositions it is also the relative error accepted between the master and the true score of a candidate, and in the heuristics the slack allowed when comparing a cost with the budget.

### Preprocessing

`flow_mip`, the Benders decompositions and `tree_lp` share the parameters of the [preprocessing](../formulations/preprocessing.md): `--resolution`, `--nb-mus`, `--mus-sweep`, and the switches that disable its steps. The defaults are the right choice in practice; the switches are there to measure what each step brings.

## Removing options first

`--reformulation path_lp` runs the path relaxation before the algorithm and removes the options that it proves to be in no solution better than the one it found. The algorithm then works on fewer options, from that solution.

```console
$ gecot optimize flow_mip instance.json 8 --reformulation path_lp
[   0.0s] Reformulating with path_lp
[   0.0s] Computing the MIP start with greedy
[   2.8s] The path relaxation leaves 15 of 27 options
[   2.8s] Running algorithm 'flow_mip' on the options left
```

It pays when the relaxation is tight and the algorithm slow on the whole instance; otherwise it only adds the time of the relaxation. When the relaxation proves its own solution optimal, the algorithm is not run at all. `--reformulation-params "time-limit=60 pricing=enumerate"` passes parameters to this `path_lp`, without their dashes; its MIP start is the solution of `greedy` unless `mip-start-algorithm` says otherwise. The report of the run prefixes the fields of the relaxation with `reformulation_` and adds `remaining_options`.

## Several budgets

GECOT solves one budget per run, and the optimal selections of two budgets are not nested in general. To draw the score as a function of the budget, run it once per budget and collect the JSON results:

```bash
mkdir -p results
for budget in 2 4 6 8 10; do
    gecot optimize multicut_benders instance.json $budget \
        --time-limit 600 -q -o results/budget_$budget.json
done
```

The directory of an output file must exist: GECOT checks it before it starts, and does not create it.

The solution of a budget is a valid MIP start for any larger budget: pass it with `--mip-start`.

## Exit code

`gecot` exits with 0 when it returns a result, including after a time limit or an interruption that left a solution, and with 1 after an error: an argument or an instance that cannot be read, a solver library that cannot be loaded, a solver that stopped without any solution.
