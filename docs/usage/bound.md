# Bounding

`gecot bound` computes an **upper bound** on the score of every solution within a budget, without looking for a solution. Compared with the score of a solution obtained in any way, a heuristic, an earlier run, a plan drawn by hand, it tells how far from the optimum this solution can be.

```console
$ gecot bound <method> <instance> <budget> [<parameters> ...]
```

The arguments are those of [`gecot optimize`](optimize.md), with a method in place of the algorithm (`--method` to give it by name). `gecot bound --list-methods` lists the methods and `gecot bound <method> --list-params` their parameters. The options of the command are `-o`, `-v`, `-vv`, `-q` and `--max-threads`.

## Methods

| Method | Bound | Needs |
|---|---|---|
| `all_options` | the score with every option bought | nothing; instant |
| `qkp_planes` | as above, with the quality gains limited by the budget | nothing; instant |
| `split_lagrange` | also limits the improvements of the arcs by the budget | options of a [simple shape](../formulations/bounds.md#split_lagrange) |
| `path_lp` | the Lagrangian bound of the path relaxation | a linear solver |
| `flow_mip`, `multicut_benders`, `flow_benders`, `multiflow_benders` | the bound proved by the exact algorithm | a MILP solver; the optimum when the status is `optimal` |

The [bounds page](../formulations/bounds.md) explains what each one computes. Every method needs a criterion that is non-decreasing in the score of every case and refuses the others; see the [instance files](instance.md#criterion).

## Reading the result

```console
$ gecot bound path_lp instance.json 8 -q
Status             converged
Initial score      356238.4908
Upper bound        439093.3254 (at most +23.26 %)
All options bound  468383.4294
Budget             8
Time               3.4 s
```

| Line | Meaning |
|---|---|
| `Status` | how the method ended, with the same values as for [`gecot optimize`](results.md#status) |
| `Initial score` | the score of the landscape as it is |
| `Upper bound` | no solution within the budget scores more; in parentheses, the largest possible gain over the initial score |
| `All options bound` | the score with every option bought, as a reference |
| `Budget`, `Time` | the budget and the time of the method |

The solutions that the bound covers are those of `gecot optimize`: the options of cost 0 are always bought.

`-o <file>` writes the result as JSON instead:

```json
{
    "instance_path":  "/home/user/instances/Aix/instance_40x40/instance.json",
    "method":  "path_lp",
    "computation_time_ms":  3290,
    "initial_score":  356238.4908355161,
    "initial_pc_num": {
        "squirrel": 356238.4908355161
    },
    "budget":         8,
    "bound":          439093.3253832179,
    "all_options_bound": 468383.42941586697,
    "method_report": {
        "status": "converged",
        "iterations": 26,
        "variables": 17382,
        "constraints": 32972,
        "pairs": 5675,
        "pool_columns": 0,
        "evicted_columns": 0,
        "searches": 19441,
        "relaxation_value": 439093.29035766155,
        "solution_score": 437866.4004434928
    }
}
```

The `method_report` of a method that is an algorithm is the [report of this algorithm](results.md#json), plus `solution_score`, the score of the solution that the algorithm found on the way.

## Certifying a solution

On the 40 × 40 raster landscape of the repository (`test/instances/Aix/instance_40x40`, 4,800 vertices and 27 options) with a budget of 8, the greedy heuristic returns in 0.2 s a solution that scores 437,866.4:

```console
$ gecot optimize greedy_incr instance.json 8 -q
Status          completed
Initial score   356238.4908
Solution score  437866.4004 (+22.91 %)
Cost            7.8 of 8
Time            178 ms
```

How good is it? Each method answers with a bound $b$. The solution gains 81,627.9 over the initial score, and no solution can gain more than $b - 356{,}238.5$: the ratio of the two, rounded down, is the share of the best possible gain that the solution is proved to reach.

| Method | Upper bound | Time | Guaranteed share |
|---|---:|---:|---:|
| `all_options` | 468,383.4 | 0 ms | 72.7 % |
| `qkp_planes` | 468,383.4 | 1 ms | 72.7 % |
| `path_lp` | 439,093.3 | 3.4 s | 98.5 % |
| `multiflow_benders` | 438,893.1 | 0.7 s | 98.7 % |

`split_lagrange` does not apply to this instance, whose options improve arcs that share no vertex. The last line is the optimum: the greedy solution is at 98.7 % of the best possible gain. On an instance of this size the exact algorithm is the fastest way to know it. On a larger one, where it does not end, `path_lp` gives a bound from its first round on, and `--time-limit` or Ctrl-C returns the best bound proved so far, for every method that iterates.

`gecot optimize path_lp` does both steps at once: it returns a solution, the better of its own and of the MIP start, with the Lagrangian bound.

!!! tip

    To certify a given solution with an exact algorithm, pass it as a MIP start: `gecot optimize multiflow_benders instance.json 8 --mip-start solution.json --time-limit 600` returns it, or a better one, with the bound proved in the time given.

## When a bound is refused

`gecot bound` fails rather than print a bound it cannot stand behind:

- the criterion is not non-decreasing in the score of every case, or the options do not have the shape that `split_lagrange` needs;
- an exact algorithm stopped before finding any solution, and so before proving a bound: give it more time or a MIP start;
- the bound computed is below the score of buying nothing, which would be a contradiction.
