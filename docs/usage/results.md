# Results and logs

`gecot` writes its result on the standard output and its log on the standard error stream, so that the two can be separated: `gecot optimize ... > result.txt` keeps the result and still shows the log.

## The result

By default the result is printed as text. Here is the two-species example [`two-species.json`](../examples/two-species.json), whose criterion is the smaller of the squirrel's score and 2.5 times the hedgehog's, with a budget of 5:

```console
$ gecot optimize multicut_benders two-species.json 5 -q
Status          optimal
Initial score   18.4
Solution score  30.9 (+67.93 %)
Upper bound     30.9 (at least 99.9 % of the best possible gain)
Cost            5 of 5
Time            3 ms
Per species
    hedgehog: 12.36 (+64.36 %)
    squirrel: 31.04 (+68.70 %)
Options bought  2 of 3:
    hedgerow
    underpass
```

| Line | Meaning |
|---|---|
| `Status` | how the algorithm ended, see [below](#status) |
| `Initial score` | the value of the criterion for the landscape as it is, without any option |
| `Solution score` | the value of the criterion with the options bought, and its gain over the initial score |
| `Upper bound` | a score that no solution within the budget exceeds, when the algorithm proves one, and the share of the best possible gain that the solution is thereby guaranteed to reach |
| `Cost` | the cost of the options bought, and the budget |
| `Time` | the time of the algorithm, without the reading of the instance |
| `Per species` | with several cases, the score of each case in the solution and its gain |
| `Options bought` | the options of the solution, among all the options of the instance |

The scores are recomputed from the options bought, not taken from the solver: a score printed is the score of the selection printed.

The **guaranteed share** compares gains, not scores: with $s$ the score of the solution, $b$ the upper bound and $s_0$ the score with only the options bought statically (see below), it is $(s - s_0) / (b - s_0)$, rounded down to one decimal. An optimal solution can therefore read 99.9 %, its bound being equal to its score only up to the solver's tolerance.

The options of cost 0 are always bought and listed, and they do not count in the initial score. The exception is an option of cost 0 in a [conflict set](instance.md#conflict-sets) with another affordable option: it stays a choice, listed only if the solution buys it, and $s_0$ does not count it. [Modeling with costs and conflicts](modeling.md#actions-that-harm-the-landscape) uses such an option, and shows how to read the scores then.

With [several costs](instance.md#several-costs), the `Cost` line is one line per dimension, as `Cost money      5 of 9`.

### Status

| Status | Meaning |
|---|---|
| `optimal` | the solution is proved optimal (`flow_mip` and the Benders decompositions), up to the [tolerances](../formulations/flow.md#what-flow_mip-solves) |
| `converged` | the column generation of `path_lp` or `tree_lp` ended: the relaxation is solved |
| `relative gap reached` | `path_lp` stopped at its `--relative-gap` |
| `completed` | a heuristic or a bound method ran to its end |
| `time limit reached` | `--time-limit` ran out: the result is the best solution and the best bound so far |
| `interrupted` | Ctrl-C stopped the algorithm: same as above |
| `no option left` | no option has a positive cost within the budget: there was nothing to decide |

A solver can also end with `limit reached`, `solver failure` or `unknown status`, which leave the best solution found if there is one, and a model that the solver declares `infeasible` or `unbounded` points to a numerical problem worth reporting.

!!! warning "Solutions over the budget"

    `tree_lp` returns the options that its relaxation uses, even fractionally, which can cost more than the budget. GECOT then logs `warning: The solution costs 5, over the budget 3`, the `Cost` line shows it, and the JSON has `"feasible": false`. Such a solution proves nothing: the upper bound is printed without a guaranteed share. Every other algorithm returns a solution within the budget.

### JSON

`-o <file>` writes the result as JSON instead of printing it:

```console
$ gecot optimize flow_mip three-woods.json 7 -o solution.json
```

```json
{
    "gecot_version":  "2.0",
    "format_version": 1,
    "instance_path":  "/home/user/three-woods.json",
    "algorithm":  "flow_mip",
    "algorithm_parameters": {
        "solver":                "gurobi",
        "feasibility-tolerance": 1e-07,
        "mip-start":             null,
        "mip-start-algorithm":   "none",
        "time-limit":            null,
        "numeric-focus":         false,
        "resolution":            1e-08,
        "nb-mus":                0,
        "mus-sweep":             "decades",
        "preprocessing":         "all",
        "all-targets":           false,
        "no-pruning":            false,
        "big-m":                 "knapsack_bnb",
        "lp-relaxation":         false,
        "indicator-constraints": false
    },
    "command_parameters": {
        "reformulation":        "trivial",
        "reformulation-params": null,
        "no-merging":           false
    },
    "computation_time_ms":  2,
    "initial_score":  18.4,
    "base_score":     18.4,
    "initial_pc_num": {
        "squirrel": 18.4
    },
    "budget":         7,
    "solution_score": 42.44,
    "solution_pc_num": {
        "squirrel": 42.44
    },
    "solution_cost":  7,
    "feasible":       true,
    "guaranteed_share": 100,
    "solution": {
        "hedgerow":   1,
        "underpass":  1,
        "plantation": 1
    },
    "algorithm_report": {
        "status": "optimal",
        "variables": 23,
        "constraints": 22,
        "nonzeros": 60,
        "model_build_time_ms": 2,
        "nodes": 0,
        "best_bound": 42.44,
        "gap": 0
    }
}
```

| Field | Meaning |
|---|---|
| `gecot_version` | the version of GECOT that wrote the file |
| `format_version` | the version of this format, an integer: it is raised when a field changes its meaning or goes, and a new field leaves it as it is |
| `instance_path` | the absolute path of the instance file |
| `algorithm` | the algorithm |
| `algorithm_parameters` | every [parameter](../reference/parameters.md) of the algorithm under its long name, with the value used, the defaults included: `true` or `false` for a switch, a number, a text, or `null` for a parameter that has no value |
| `command_parameters` | in the same way, the parameters of `gecot optimize` that change the instance given to the algorithm: `--reformulation`, `--reformulation-params` and `--no-merging` |
| `computation_time_ms` | the time of the algorithm, in milliseconds |
| `initial_score`, `solution_score` | the criterion without options and with the solution |
| `base_score` | the criterion with only the options bought statically, the $s_0$ of the [guaranteed share](#the-result) |
| `initial_pc_num`, `solution_pc_num` | the score of each case, without options and with the solution, including the cases that the criterion does not read |
| `budget`, `solution_cost` | the budget and the cost of the solution; with several costs, an object with a number per dimension, as `{"area": 4, "money": 9}` |
| `feasible` | `true` if the solution fits every budget and buys at most one option of each conflict set, as checked by the warnings below. A cost counts as within its budget up to a relative $10^{-6}$ of the budget, or of 1 under a smaller budget; this margin is the one of the command and does not follow `-t`, so that a run with a larger `-t` can be reported as not feasible |
| `guaranteed_share` | the guaranteed share, in %, as the text result prints it after the upper bound: it is computed from the larger of `best_bound` and `solution_score`. Absent when the algorithm proves no bound, when the solution is not feasible, when that bound is not above `base_score`, and when the algorithm's bound is under `solution_score` by more than $10^{-6}$ of it, or by more than $10^{-6}$ when it is under 1, which the [warnings](#warnings-worth-reading) tell |
| `solution` | for each option of the instance, 1 if it is bought and 0 otherwise |
| `algorithm_report` | the status and what the algorithm reports about its run; a value that is not a finite number is left out |

The names of the options, of the cases and of the cost dimensions are written as they are in the instance, with their quotes, backslashes and control characters escaped. The file is UTF-8 when they are: a name read from a CSV file in another encoding keeps its bytes, and the file is then to be read in that encoding.

The `algorithm_report` depends on the algorithm:

| Field | Reported by | Meaning |
|---|---|---|
| `status` | all | see [above](#status) |
| `best_bound` | the algorithms that prove a bound | the upper bound, as the solver or the relaxation gives it |
| `gap` | same | (bound − score) / score |
| `variables`, `constraints`, `nonzeros` | `flow_mip`, the Benders decompositions | the size of the model given to the solver; `path_lp` and `tree_lp` give the first two for their master. With `--indicator-constraints`, the indicator constraints are not counted |
| `model_build_time_ms` | `flow_mip`, the Benders decompositions | the time before the solver starts, preprocessing included |
| `nodes` | same, when the solver reports it | the nodes of the branch and bound |
| `lazy_constraints` | the Benders decompositions | the cuts handed to the solver during the search. One that the candidate violates by at most the solver's tolerance is counted too, although CPLEX and COPT are then not given it |
| `sources`, `root_rounds`, `root_cuts`, `root_relaxation_value` | `multiflow_benders` | the source and target pairs, and the [cuts on the relaxation](../formulations/benders.md#cuts-on-the-relaxation): the relaxations solved before the search, the cuts they added, and the value of the last one |
| `iterations`, `relaxation_value`, `pool_columns`, `evicted_columns` | `path_lp`, `tree_lp` | the rounds of the column generation and the value of the relaxation at the end |
| `pairs`, `searches` or `columns` | `path_lp` | the pairs priced, and the searches or the columns of its pricing |
| `reformulation_...`, `remaining_options` | with `--reformulation path_lp` | the report of the path relaxation, and the options it left |

A heuristic reports only its status. The file is also a valid [`--mip-start`](optimize.md#mip-start).

### CSV

`--output-csv <file>` writes only the selection, one line per option of the instance, its name between quotes and a quote of the name doubled:

```text
option_id,value
"hedgerow",1
"underpass",1
"plantation",1
```

`-o` and `--output-csv` can be given together. With either of them, the text result is not printed.

## The log

Each line of the log starts with the seconds elapsed since `gecot` started. On a terminal, the levels have their colours: warnings in yellow, errors in red.

```console
$ gecot optimize multicut_benders instance.json 8 --preprocessing none --time-limit 25
[   0.0s] GECOT 2.0, algorithm multicut_benders
[   0.0s] Instance: /home/user/instances/Aix/instance_40x40/instance.json
[   0.0s] Options count: 27
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s]   species   | vertices | habitats | improvable |  arcs   | improvable
[   0.0s]             |          |          |  vertices  |         |    arcs
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s]    squirrel |     4800 |      752 |         27 |    9440 |        100
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s] Budget 8: 28.17 % of the options' total cost (28.4)
[   0.1s] Solving the optimization model
[   2.2s] Ctrl-C stops the algorithm and keeps its best result so far
[   2.2s] -------------+--------------+-------------
[   2.2s]     gain     |   max gain   |  guaranteed
[   2.2s] -------------+--------------+-------------
[   2.2s]     +20.20 % |     +60.87 % |       33.1 %
[   7.4s]     +22.97 % |     +49.18 % |       46.7 %
[  12.5s]     +22.97 % |     +46.92 % |       48.9 %
[  17.6s]     +22.97 % |     +45.36 % |       50.6 %
[  22.7s]     +22.97 % |     +44.04 % |       52.1 %
[  25.0s] -------------+--------------+-------------
[  25.0s] warning: Solver stopped: time limit reached
```

The first lines say what was read: the options, the cases, and in a table a row per case with its vertices, those of positive quality (`habitats`), those that some option improves, its arcs and those that some option improves; then, with several costs or conflict sets, how many there are, as `Cost dimensions: 2, conflict sets: 2`, and the budget as a share of the total cost of the options, a line per dimension with several costs, as `Budget money=5: 71.43 % of the options' total (7)`.

### Progress

When the algorithm runs for more than two seconds, a table follows its progress, with a new row when a value changes: at most every five seconds, and at least every minute. Its header comes again every 20 rows, a value that is not known yet reads `-`, and a rule closes the table when the algorithm stops or goes on to another step.

For `flow_mip` and the Benders decompositions:

| Column | Meaning |
|---|---|
| `gain` | the gain of the best solution so far, over the initial score |
| `max gain` | the largest gain that the bound proved so far still allows |
| `guaranteed` | the share of this largest gain that the best solution reaches |

The run above was stopped by its time limit with a solution proved to reach 52 % of the best possible gain. It was deliberately run without preprocessing: with the defaults the same instance is solved to optimality in 1.4 s.

For `path_lp` and `tree_lp`, each row is a round of the column generation:

```text
[   2.1s] -------------+--------------+-------------
[   2.1s]    round     |  relaxation  |   max gain
[   2.1s] -------------+--------------+-------------
[   2.1s]            7 |     +22.06 % |     +24.67 %
```

| Column | Meaning |
|---|---|
| `round` | the number of the round |
| `relaxation` | the gain of the master's value, which grows toward the value of the relaxation |
| `max gain` | for `path_lp`, the gain of its Lagrangian bound |

### Verbosity

| Option | Log |
|---|---|
| `-q` | errors only |
| (default) | the steps of the run, the progress, the warnings |
| `-v` | also the parameters of the algorithm with their values, the size of the instance after its reformulation, the factor by which the qualities are divided, a table of the preprocessing of each species, the size of the model; the progress table has a row every second or every round, and a `nodes` column |
| `-vv` | also the details of each step, such as a table of the options that a heuristic buys or removes (a swap of `greedy` reads `a -> b`), and the solver's own log, which goes to the standard output |

With `-v`, the line `Qualities scaled by 2^...` gives the factor of the [rescaling](../formulations/preprocessing.md#the-instance-given-to-the-algorithms), and the line `The criterion multiplied by 2^...`, if any, the factor of the criterion on top. The solver's own log is in the rescaled unit, and so are the lines that GECOT logs about the run itself at `-v` and `-vv`, such as `Initial score: ... (was ...)` and those of the bound methods; the summary, the result files and the progress table are in the unit of the instance.

With `-v` again, the algorithms that [preprocess](../formulations/preprocessing.md) the graphs log a table once every species is preprocessed, here for `flow_benders`:

```text
[   0.0s] Preprocessing the 'squirrel' graph
[   0.1s] Computing contractions and big-Ms of the 'squirrel' graph
[   0.1s] -------------+-------------+--------------+------------+-----------
[   0.1s]   species    | strong arcs | useless arcs |  vertices  |    arcs
[   0.1s]              | per target  |  per target  | per target | per target
[   0.1s] -------------+-------------+--------------+------------+-----------
[   0.1s]     squirrel |     2740.35 |      2347.32 |     213.37 |     558.01
[   0.1s] -------------+-------------+--------------+------------+-----------
```

| Column | Meaning |
|---|---|
| `strong arcs per target`, `useless arcs per target` | the arcs that the graph of a target loses, contracted or removed, on average over the targets of the species; 0 for a kind that `--preprocessing` switches off |
| `vertices per target`, `arcs per target` | for `flow_benders`, the size of the graph of a target once preprocessed, on average |
| `targets per group` | for `multicut_benders`, the number of targets of each group of the species |

### Ctrl-C

Ctrl-C stops the algorithm and keeps its best result so far, with the status `interrupted`; the result is printed or written as usual. A second Ctrl-C quits at once, without result.

- A MILP solver notices the interruption when it next reports its progress to GECOT: within seconds with Gurobi, up to some fifteen seconds with HiGHS on a large model, never with CBC, which reports none.
- The column generations of `path_lp` and `tree_lp` stop at the end of their current round.
- The heuristics run to their end.
- If the algorithm has no solution yet and no MIP start, there is nothing to keep, and the run ends with an error.

### Warnings worth reading

| Warning | Meaning |
|---|---|
| `Budget doesn't allow purchasing any options` | every option costs more than the budget, or costs 0 |
| `The solution costs ..., over the budget ...` | `tree_lp` returned its relaxation, see [above](#status) |
| `The solution costs ... in <dimension>, over the budget ...`, `The solution buys ... options of the conflict set '...'` | should not happen with the algorithms that support several costs and conflict sets; report it |
| `Solver stopped: time limit reached` | the result is the best so far, not a proved optimum |
| `The criterion is ... times its terms where nothing is bought: an added constant dominates it, ...` | a constant added to the criterion, as in `{"operation": "sum", "values": ["trout", 1e8]}`, is more than $2^{20}$ times the terms that the options change. It changes no choice, but the relative tolerances of the solvers, and the margin of $10^{-6}$ of the check of a bound, apply to the whole value, so they can hide the gains. Taking the constant out of the criterion lifts that |
| `The bound ... of the algorithm is below the score ... of its solution: no bound is shown, nor any share guaranteed` | the solver's tolerances failed, typically on a criterion whose value is far smaller than the scores of its cases; the status, `optimal` included, is not proved either. The report keeps the algorithm's `best_bound` and `gap` |
| `Solver stopped without a solution: ..., returning the MIP start` | the solver found nothing in the time given |
| `This solver has no time limit: --time-limit ignored` and similar | the solver lacks a feature that a parameter asks for |
| `mippp: warning: the ... wrapper is validated for versions ...` | the solver library is outside the range of versions that mippp was tested with |
