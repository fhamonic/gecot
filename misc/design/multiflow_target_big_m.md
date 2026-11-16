# multiflow_benders: tighter or exact constants for the target quality gains, a study

Status, 2026-11-16: `--big-m`, which `multiflow_benders` did not have, is in the code and the docs, with the kinds shared by every algorithm and `exact` (`exact_big_m.md`). The experimental `--target-gain-cuts` was removed: `target_gain_cuts.md` keeps what is needed to bring it back. Both came with the branch `multiflow-target-big-m` (tag `archive/multiflow-target-big-m`). The question was whether `multiflow_benders` should compute a tighter M^t_t, or an exact one. A tighter or exact constant moves the root little, and an exact one costs more to compute than it saves. Cuts per source on the same variables move it much more, but cost more time than they save at 120 s. At very small budgets too, `tight` lowers the constants by at most 2.4 % on the landscapes and moves the root by at most 0.29 points. Where a single option fits, the exact constant equals `knapsack_bnb`'s; where 5 to 24 fit, the exact constants, from a flow MIP per target, are 0 to 9 % below and move the root by at most 0.83 points, at the cost of one MIP per gain row.

## Where M^t_t is read

`multiflow_benders` reads one constant per target t, M^t_t, and in two places:

- **the upper bound of f_t.** Without it, the relaxation is unbounded until t receives a cut. It no longer matters after the first round.
- **the rows f^i_t ≤ M^t_t · x_i** of the McCormick linearization of the target quality gains, f^i_t = x_i · f_t, next to f^i_t ≤ f_t. These rows stay in the model.

Only the instances with target quality gains have these rows. In the benchmark, those are `dks`, `periurban-mixed`, `rural-planting`, `urban-greening(-large)`, `species-periurban` and the squirrel case of `bike-squirrel-*`: 23 of the 42 runs of the short suite. hexagon 1_1900_3800 has 1386 of them, one per option. Before the branch, M^t_t was always the `knapsack_bnb` one.

## What the branch adds

- **`--big-m plain|knapsack_bnb|knapsack_dantzig|tight`**, with `knapsack_bnb` the default and the same as main.
  - The first three give M^t_t as `flow_mip`'s kinds do, computed at t only.
  - `tight` keeps the `knapsack_bnb` M^t_t and lowers the row of each target quality gain i to the per-option target constant of `compute_tight_big_M_maps`: the Lagrangian reach from t within the budget less c_i, with the gains of i outside the knapsack. A new `with_arcs` argument skips the constants of the arcs, which `multiflow_benders` does not read. On `big-m-exact`, it is the template parameter `WithArcs`, and `multiflow_benders` takes M^t_t and these constants from `compute_contracted_flow_graph<false>`.
  - The constant of i covers only the scenarios that buy i, so it never bounds f_t.
- **`--target-gain-cuts`** (removed since, see `target_gain_cuts.md`): cuts on the f^i_t in the rounds on the relaxation, from the flows per source.
  - f^i_t = x_i · f_t is at most the sum over the sources s of the least of m_st · x_i, with m_st = w̄_s · Π̄(s, t), and of the largest flow of s when the constant part w_s of its supply is multiplied by x_i. Both terms are at least f_s at a scenario that buys i, and at least 0 at one that does not.
  - For t itself, the terms are w̄_t · x_i and w_t · x_i plus its other source gains.
  - `target_block::separate` computes them in its loop over the sources, with one more flow per source and gain on the same A_s. With `--no-stored-arcs`, the cuts of f_t and those of the f^i_t share the rebuild of A_s.
  - A cut is added when f^i_t at the optimum exceeds it, with the tolerance of the cuts of f_t. `root_gain_cuts` is reported when the flag is set.
  - At the candidates nothing changes: at a scenario, the two McCormick rows are exact.

## Measurements

Machine: 16 cores, Gurobi 12, single runs.

The root gap is (root − best) / (best − initial), in % of the best gain. Here root is `root_relaxation_value` and best is the best score that any run or `benchmark/best_known.csv` knows. `best_known.csv` has no rows for `species-periurban` and `bike-squirrel-*`, so there best comes from these runs alone. At 6000 it is the 120 s optimum, 1.2222, against main's 1.1997 at 30 s.

A root compares only where the rounds ended because no cut was violated. (c) marks the rounds that the 15 s clock of a 30 s run stopped first.

The scalar constants beyond `tight` were measured with a study build only (not committed), on both sides of the exact constant M^i_t = max f_t(y) over the feasible scenarios y that buy i:

- **LP constants, at least the exact ones, hence valid.** For each target with gains, the LP of the per-source flows of that target alone, with x_i = 1, the budget and the conflict sets, solved by plain Kelley rounds of the block's own cuts.
- **Rounding constants, at most the exact ones.** f_t at the scenario that buys i, then the options in decreasing order of their value in that LP, as long as they fit. Rows with these constants can cut the optimum off, so only their roots are evidence: they bound what any exact constant could do.

### The short suite runs with target quality gains, 30 s

| run | `knapsack_bnb` (main) | `tight` | `knapsack_dantzig` | `plain` | LP constants | rounding constants | `--target-gain-cuts` | cuts + rounding constants |
|---|---|---|---|---|---|---|---|---|
| `dks-30` at 9 | 0.41 | 0.41 | 0.41 | 3.45 | 0.41 | 0.41 | 0.41 | 0.41 |
| `dks-30` at 12 | 1.14 | 1.14 | 1.14 | 2.79 | 1.14 | 1.14 | 1.14 | 1.14 |
| `dks-30` at 15 | 1.73 | 1.73 | 1.73 | 2.14 | 1.73 | 1.73 | 1.73 | 1.73 |
| `dks-50` at 10 | 0.13 | 0.13 | 0.13 | 2.52 | 0.13 | 0.13 | 0.13 | 0.13 |
| `dks-50` at 15 | 0.44 | 0.44 | 0.44 | 1.99 | 0.44 | 0.44 | 0.44 | 0.44 |
| `periurban-mixed` at 2500 | 1.96 | 1.96 | 1.96 | 2.01 | 1.81 | 1.76 | 1.40 | 1.40 |
| `periurban-mixed` at 6000 | 1.56 | 1.56 | 1.56 | 1.58 | 1.36 | 1.33 | 0.99 | 0.98 |
| `periurban-mixed` at 12000 | 0.29 | 0.29 | 0.29 | 0.29 | 0.26 | 0.25 | 0.23 (c) | 0.22 |
| `rural-planting` at 90 | 0.51 | 0.51 | 0.52 | 3.92 | 0.33 | 0.25 | 0.20 | 0.17 |
| `rural-planting` at 260 | 0.81 | 0.81 | 0.82 | 2.38 | 0.69 | 0.65 | 0.47 | 0.46 |
| `rural-planting` at 510 | 0.36 | 0.36 | 0.36 | 0.92 | 0.29 | 0.26 | 0.12 | 0.11 |
| `urban-greening` at 1000 | 7.65 | 7.65 | 7.83 | 14.65 | 6.53 | 5.92 | 6.35 | 5.61 |
| `urban-greening` at 7500 | 2.67 | 2.67 | 2.68 | 4.29 | 1.93 | 1.84 | 0.65 | 0.65 |
| `urban-greening` at 15000 | 0.68 | 0.68 | 0.69 | 0.84 | 0.55 | 0.54 | 0.20 | 0.20 |
| `urban-greening-large` at 1600 | 2.99 | 2.99 | 3.00 | 4.26 | 2.63 | 2.57 | 2.43 | 2.42 |
| `urban-greening-large` at 4000 | 1.04 | 1.04 | 1.04 | 1.68 | 0.79 | 0.77 | 0.53 | 0.53 |
| `urban-greening-large` at 8000 | 0.50 | 0.50 | 0.51 | 0.74 (c) | 0.41 | 0.40 | 0.28 (c) | 0.28 |
| `species-periurban` at 2500 | 1.78 | 1.78 | 1.78 | 1.82 | 1.64 | 1.63 | 1.38 | 1.38 |
| `species-periurban` at 6000 | 4.26 | 4.26 | 4.26 | 4.26 | 4.23 | 4.22 | 4.19 (c) | 4.19 (c) |
| `bike-squirrel-town` at 200 | 35.54 | 35.54 | 35.54 | 35.54 | 35.54 | 35.54 | 35.54 | 35.54 |
| `bike-squirrel-town` at 300 | 6.63 | 6.63 | 6.63 | 6.63 | 6.63 | 6.63 | 6.63 | 6.63 |
| `bike-squirrel-suburb` at 200 | 24.36 | 24.36 | 24.36 | 24.36 | 24.36 | 24.36 | 24.36 | 24.36 |
| `bike-squirrel-suburb` at 500 | 28.39 (c) | 30.73 (c) | 28.39 (c) | 35.53 (c) | 28.39 (c) | 28.39 (c) | 28.39 (c) | 28.39 (c) |

The runs solved to optimality, and the seconds they took, less the time spent on the scalar constants:

| | solved | seconds |
|---|---|---|
| `knapsack_bnb` | 19 | 155.4 |
| `tight` | 19 | 150.5 |
| `knapsack_dantzig` | 20 | 192.7 |
| `plain` | 19 | 175.6 |
| LP constants | 19 | 156.4 |
| `--target-gain-cuts` | 19 | 153.6 |

The rounding rows are left out of this table, as they can cut the optimum off. All optimal scores agree across the configurations. Every 30 s run, main's column included, ran on a study build that also computed the four kinds of constants at every target and solved the relaxation once more after the rounds, so these seconds carry that overhead in every column. They cannot show what `tight` costs.

**The root is sensitive to M^t_t.** `plain` loses up to 7 points of the gain (`urban-greening` at 1000: 7.65 to 14.65 %). At the lowest budgets of `dks` and `rural-planting` it loses 2.4 to 3.4 points, and the gap grows several times over.

**`knapsack_bnb` is already close to the floor of a scalar constant:**

- `tight` gives the same roots to two decimals.
- The exact constants lie 0 to 13 % below the `knapsack_bnb` ones, between the two bounds below. The LP and rounding constants are 0 to 4 % apart.
- Even the rounding constants close only a fraction of the root gap (`urban-greening` at 7500: 2.67 to 1.84 %).

| run | rows | LP / knapsack | rounding / knapsack | LP constants (s) |
|---|---|---|---|---|
| `periurban-mixed` at 2500 | 118 | 0.953 | 0.940 | 1.4 |
| `periurban-mixed` at 6000 | 126 | 0.903 | 0.874 | 30.0 |
| `periurban-mixed` at 12000 | 126 | 0.954 | 0.947 | 37.8 |
| `rural-planting` at 90 | 122 | 0.983 | 0.980 | 4.4 |
| `rural-planting` at 510 | 122 | 0.995 | 0.993 | 4.9 |
| `urban-greening` at 1000 | 123 | 0.947 | 0.935 | 1.0 |
| `urban-greening` at 7500 | 123 | 0.975 | 0.971 | 10.5 |
| `urban-greening-large` at 4000 | 207 | 0.980 | 0.978 | 66.9 |
| `urban-greening-large` at 8000 | 207 | 0.986 | 0.985 | 72.0 |
| `species-periurban` at 6000 | 252 | 0.903 | 0.868 | 43.3 |
| `dks-*` | 30–50 | 1 | 1 | 0.0 |

The ratios are of the sums over the rows. The LP constants took 1 to 72 s with plain Kelley rounds, 19 to 243 rounds per row on average (2 on `dks`). In-out rounds would cut that, but the solve times net of it show nothing to recover.

**`dks` moves for nothing scalar.** Its options improve vertices only, on one cost dimension without conflict sets, so `tight`'s constant of i is the exact one. It equals `knapsack_bnb` on every row, because the option of t is in the best knapsack. The cuts per source do not move it either. Near the uniform point x = k/n, the aggregated row with the exact M is the tighter one, and the per-source terms min(x_s, x_t) are the usual weak relaxation of the densest subgraph.

**`bike-squirrel` moves for nothing at all.** Its root gap does not come from the target quality gains.

**The cuts per source close more than any scalar constant could**, on every landscape but `urban-greening` at 1000:

- `urban-greening` at 7500: 0.65 %, against 1.84 % with the rounding constants;
- `rural-planting` at 510: 0.12 %, against 0.26 %.

With the cuts in place, even the rounding constants add at most 0.03 points, except on `urban-greening` at 1000 (6.35 to 5.61 %).

### Larger budgets with target quality gains, 120 s

Main, and `--target-gain-cuts` as the study build computed them, before they were folded into `separate`: in a pass of their own over the sources, the same cuts. Time when solved; otherwise the bound at the limit, as a gap like the root's. The last column is the time until the search (`model_build_time_ms`): the preprocessing, the model and the rounds.

| run | main: time, nodes, root gap | `--target-gain-cuts`: time, nodes, root gap | rounds' time, main / cuts (s) |
|---|---|---|---|
| `periurban-mixed` at 12000 | 19.2 s, 11, 0.29 % | 25.0 s, 16, 0.23 % | 10.1 / 16.4 |
| `periurban-mixed` at 25000 | 56.8 s, 3597, 0.45 % | 65.7 s, 3150, 0.40 % | 16.5 / 23.2 |
| `periurban-mixed` at 63000 | 27.0 s, 786, 0.09 % | 38.8 s, 646, 0.07 % | 12.4 / 18.0 |
| `rural-planting` at 850 | 3.2 s, 1, 0.23 % | 3.8 s, 1, 0.16 % | 1.2 / 1.4 |
| `urban-greening` at 23000 | 5.6 s, 1, 0.10 % | 7.6 s, 1, 0.05 % | 3.7 / 5.5 |
| `urban-greening-large` at 8000 | 17.9 s, 31, 0.50 % | 21.0 s, 42, 0.28 % | 12.5 / 15.3 |
| `urban-greening-large` at 16000 | 48.2 s, 422, 0.61 % | 32.5 s, 109, 0.14 % | 22.5 / 22.2 |
| `urban-greening-large` at 40000 | 58.8 s, 1991, 0.07 % | 71.5 s, 1157, 0.04 % | 28.7 / 42.5 |
| `species-periurban` at 2500 | 36.7 s, 11, 1.78 % | 35.3 s, 9, 1.38 % | 8.0 / 10.2 |
| `species-periurban` at 6000 | 54.4 s, 1465, 4.26 % | 69.7 s, 1656, 4.19 % | 12.8 / 17.2 |
| `species-periurban` at 12000 | 24.6 s, 86, 0.11 % | 28.3 s, 96, 0.10 % | 13.4 / 16.1 |
| `species-periurban` at 25000 | 29.2 s, 194, 0.10 % | 44.7 s, 86, 0.04 % | 18.8 / 27.0 |
| `bike-squirrel-suburb` at 500 | limit, bound 6.24 %, 4969, 22.65 % | limit, bound 7.96 %, 5815, 22.65 % | 60.7 / 61.8 |
| `bike-squirrel-suburb` at 1000 | limit, bound 20.47 %, 4510, 31.70 % | limit, bound 20.44 %, 3314, 31.70 % | 62.7 / 61.4 |
| `bike-squirrel-suburb` at 2000 | limit, bound 16.90 %, 4488, 21.67 % | limit, bound 16.76 %, 3031, 21.52 % | 60.0 / 63.2 |
| `dks-50` at 20 | limit, bound 0.27 %, 9363568, 0.81 % | limit, bound 0.28 %, 9296527, 0.81 % | 0.0 / 0.0 |
| `dks-50` at 25 | limit, bound 0.25 %, 3784839, 1.12 % | limit, bound 0.26 %, 3344845, 1.12 % | 0.0 / 0.0 |

On `bike-squirrel-suburb`, the 60 s clock stopped the rounds in both columns. The 12 runs that both solve take 381.6 s with main and 443.9 s with the cuts. The cuts tighten every landscape root, and the tree often needs fewer nodes. But their rounds cost up to 14 s more, one more flow per source and gain: 2 to 14 s on 9 of the 12 runs, 0.2 and 1.8 s on `rural-planting` at 850 and `urban-greening` at 23000, and 0.3 s less on `urban-greening-large` at 16000. Their rows also slow the nodes. Only `urban-greening-large` at 16000 gains clearly, from 48.2 to 32.5 s; `species-periurban` at 2500 goes from 36.7 to 35.3 s.

### hexagon 1_1900_3800 at 151, 300 s, `--no-stored-arcs`

The final build, on both sides; 1386 rows of target quality gains. The clock stops the rounds of both runs after 5 rounds, at 171.2 s without the cuts and 175.6 s with them: one more flow per source and gain costs little there, as the cuts share the rebuild of A_s.

The 520 gain cuts leave the relaxation where it is, 422,635.85, at +103.68 % of the initial score against +76.98 % for the best solution. The bound at the limit is the same too, 422,564.82, after one node. The rounds are too far from converging for the rows of the gains to be what binds. The peak memory is 2.41 GB without the cuts and 2.43 GB with them.

### Very small budgets, 60 s (2026-11-16)

The question: at budgets that fit only a few options, a constant that accounts for the budget in the probabilities as well as in the qualities should lower the gain rows, if anything does.

**The runs.**

- 31 budgets on the 8 instances with target quality gains, with 1 to 28 options fitting together.
- Each budget ran with main, with `tight` and with `--target-gain-cuts`. The landscapes also ran with `plain`.
- Every run is optimal within 22 s, and no root loop reached its 30 s clock, so all the roots compare.
- On 5 of the 8 `bike-squirrel` budgets, no option within the budget has a target quality gain, so there is no gain row.

**What the study build added** (not committed). It logged, per gain row, M^t_t and four other constants:

- the `tight` one;
- the `tight` one with μ = 0 alone. That leaves only the knapsack within β − c_i, the gains of i outside it; the other multipliers add the budget-aware probabilities.
- f_t with i alone, Σ_s Π_i(s, t) (w_s + the gains of i at s), with the probabilities of i's improvements only;
- f_t at the empty scenario, with the base probabilities.

Any valid constant of row i lies between f_t with i alone and M^t_t. When a single option fits, the scenario {i} is the only one that buys i, so f_t with i alone is then the exact constant.

Two more study runs set the rows to the last two constants. Below the exact constant, the rows can cut the optimum off, as the rounding constants above can. Their roots are therefore at most those of any valid scalar constant. Where a single option fits, the i-alone roots are those of the exact constant.

**The constants**, as ratios to M^t_t of the sums over the rows. The rows count only the options within the budget, because the trivial reformulation removes the others. f_t with i alone was not logged on `dks`.

| run | options that fit together | rows | `tight` | μ = 0 only | i alone | empty scenario |
|---|---|---|---|---|---|---|
| `dks-30` at 3 | 3 | 30 | 1.000 | 1.000 | – | 0.000 |
| `dks-30` at 5 | 5 | 30 | 1.000 | 1.000 | – | 0.000 |
| `rural-planting` at 5 | 1 | 12 | 1.000 | 1.000 | 0.999 | 0.989 |
| `rural-planting` at 10 | 3 | 33 | 0.999 | 0.999 | 0.991 | 0.978 |
| `rural-planting` at 20 | 6 | 82 | 0.997 | 0.998 | 0.972 | 0.956 |
| `urban-greening` at 50 | 1 | 2 | 1.000 | 1.000 | 1.000 | 0.997 |
| `urban-greening` at 100 | 2 | 3 | 1.000 | 1.000 | 1.000 | 0.997 |
| `urban-greening` at 250 | 4 | 23 | 0.993 | 0.994 | 0.984 | 0.969 |
| `urban-greening` at 500 | 7 | 91 | 0.982 | 0.987 | 0.941 | 0.920 |
| `urban-greening-large` at 50 | 1 | 3 | 1.000 | 1.000 | 1.000 | 0.998 |
| `urban-greening-large` at 100 | 2 | 9 | 0.999 | 0.999 | 0.999 | 0.997 |
| `urban-greening-large` at 250 | 5 | 52 | 0.998 | 0.999 | 0.994 | 0.990 |
| `urban-greening-large` at 800 | 13 | 203 | 0.995 | 0.997 | 0.949 | 0.940 |
| `periurban-mixed` at 30 | 1 | 5 | 1.000 | 1.000 | 1.000 | 0.971 |
| `periurban-mixed` at 60 | 2 | 15 | 1.000 | 1.000 | 0.994 | 0.959 |
| `periurban-mixed` at 150 | 5 | 49 | 0.994 | 0.997 | 0.952 | 0.903 |
| `periurban-mixed` at 400 | 13 | 55 | 0.998 | 0.998 | 0.919 | 0.876 |
| `periurban-mixed` at 1000 | 24 | 80 | 0.976 | 0.993 | 0.835 | 0.809 |
| `species-periurban` at 30 | 1 | 10 | 1.000 | 1.000 | 1.000 | 0.979 |
| `species-periurban` at 60 | 2 | 30 | 1.000 | 1.000 | 0.995 | 0.972 |
| `species-periurban` at 150 | 5 | 98 | 0.989 | 0.997 | 0.942 | 0.911 |
| `species-periurban` at 400 | 13 | 110 | 0.997 | 0.997 | 0.914 | 0.886 |
| `species-periurban` at 1000 | 24 | 160 | 0.976 | 0.991 | 0.831 | 0.813 |
| `bike-squirrel-town` at 3 | 2 | 0 | | | | |
| `bike-squirrel-town` at 10 | 5 | 0 | | | | |
| `bike-squirrel-town` at 30 | 12 | 0 | | | | |
| `bike-squirrel-town` at 100 | 28 | 4 | 0.988 | 1.000 | 0.814 | 0.812 |
| `bike-squirrel-suburb` at 3 | 1 | 0 | | | | |
| `bike-squirrel-suburb` at 10 | 4 | 0 | | | | |
| `bike-squirrel-suburb` at 30 | 10 | 1 | 0.950 | 1.000 | 0.946 | 0.946 |
| `bike-squirrel-suburb` at 100 | 23 | 15 | 0.925 | 1.000 | 0.711 | 0.710 |

**The root gaps**, in % of the best gain, as above. The best gain is that of the four valid configurations.

| run | `knapsack_bnb` | `tight` | cuts | `plain` | i alone | empty scenario |
|---|---|---|---|---|---|---|
| `dks-30` at 3 | 0.00 | 0.00 | 0.00 | – | – | – |
| `dks-30` at 5 | 0.00 | 0.00 | -0.00 | – | – | – |
| `rural-planting` at 5 | 43.46 | 43.46 | 43.46 | 51.98 | 43.43 | 42.84 |
| `rural-planting` at 10 | 17.63 | 17.60 | 17.63 | 20.44 | 17.40 | 16.81 |
| `rural-planting` at 20 | 8.06 | 8.06 | 7.78 | 12.75 | 7.12 | 6.56 |
| `urban-greening` at 50 | 19.90 | 19.90 | 19.90 | 19.95 | 19.90 | 19.75 |
| `urban-greening` at 100 | 4.79 | 4.79 | 4.79 | 5.70 | 4.78 | 4.64 |
| `urban-greening` at 250 | 23.55 | 23.55 | 23.12 | 24.37 | 23.04 | 21.51 |
| `urban-greening` at 500 | 6.82 | 6.81 | 6.29 | 10.69 | 5.06 | 3.41 |
| `urban-greening-large` at 50 | 4.52 | 4.52 | 4.52 | 4.83 | 4.52 | 4.45 |
| `urban-greening-large` at 100 | 2.15 | 2.15 | 2.15 | 2.16 | 2.13 | 1.86 |
| `urban-greening-large` at 250 | 9.16 | 9.16 | 9.08 | 9.35 | 8.98 | 8.60 |
| `urban-greening-large` at 800 | 7.48 | 7.47 | 6.84 | 9.78 | 6.47 | 5.98 |
| `periurban-mixed` at 30 | 15.17 | 15.17 | 15.17 | 21.96 | 15.17 | 13.92 |
| `periurban-mixed` at 60 | 17.29 | 17.29 | 17.27 | 18.90 | 17.17 | 15.81 |
| `periurban-mixed` at 150 | 7.24 | 7.24 | 7.06 | 7.94 | 6.61 | 5.17 |
| `periurban-mixed` at 400 | 3.98 | 3.98 | 3.28 | 4.33 | 2.23 | 0.34 |
| `periurban-mixed` at 1000 | 2.58 | 2.56 | 1.94 | 2.69 | -0.88 | -2.65 |
| `species-periurban` at 30 | 17.40 | 17.40 | 17.40 | 21.73 | 17.40 | 14.24 |
| `species-periurban` at 60 | 10.48 | 10.47 | 10.46 | 11.39 | 10.35 | 7.07 |
| `species-periurban` at 150 | 14.30 | 14.01 | 13.52 | 15.74 | 13.03 | 11.71 |
| `species-periurban` at 400 | 26.59 | 26.59 | 25.84 | 27.50 | 24.41 | 22.91 |
| `species-periurban` at 1000 | 3.96 | 3.94 | 3.48 | 4.10 | 0.69 | -0.17 |
| `bike-squirrel-town` at 3 | 77.78 | 77.78 | 77.78 | – | – | – |
| `bike-squirrel-town` at 10 | 10.32 | 10.32 | 10.32 | – | – | – |
| `bike-squirrel-town` at 30 | 557.41 | 557.41 | 557.41 | – | – | – |
| `bike-squirrel-town` at 100 | 21.72 | 21.72 | 21.72 | – | 21.72 | 21.72 |
| `bike-squirrel-suburb` at 3 | -0.00 | -0.00 | -0.00 | – | – | – |
| `bike-squirrel-suburb` at 10 | 0.92 | 0.92 | 0.92 | – | – | – |
| `bike-squirrel-suburb` at 30 | 1023.30 | 1023.30 | 1023.30 | – | 1023.30 | 1023.30 |
| `bike-squirrel-suburb` at 100 | 45.83 | 45.83 | 45.83 | – | 45.83 | 45.83 |

On `bike-squirrel`, the gains at the smallest budgets are 0.00 to 0.24 % of the initial score, hence the gaps of several hundred percent.

**The rows bind.** `plain` loosens the root by up to 8.5 points: `rural-planting` at 5 goes from 43.46 to 51.98 %, and `periurban-mixed` at 30 from 15.17 to 21.96 %.

**Where a single option fits, no scalar constant can do better.**

- f_t with i alone, the exact constant there, is within 0.1 % of M^t_t, and its roots are main's to 0.03 points.
- The 4.5 to 43 % gaps of these runs sit outside the gain rows, in the flows. Their arc constants m_sa = w̄_s · Π̄(s, u) ignore the budget.
- A budget-aware m_s, from the Lagrangian reach of `tight` per source, was measured on 2026-11-12 in the Python prototype of the per-source cuts, outside the repository.
  - It cut the root gap of aix40 at 2 by 55 %, of aix70 at 2 by 42 %, and of aix70 at 4 by 6.5 %, but nothing from 8 on.
  - It won no time on the solved instances.
  - It is not rebuilt here.

**Where 5 to 24 options fit, there is room, but `tight` does not reach it.**

- The constants of f_t with i alone are 0.6 to 17 % below M^t_t on the landscapes, and their roots lie 0.2 to 3.5 points below main's. `periurban-mixed` at 1000 goes below the optimum, at −0.88 %, so those constants are invalid there.
- The exact constants lie between those and M^t_t; this study does not place them.
- `tight` lowers the constants by at most 2.4 % and the roots by at most 0.29 points (`species-periurban` at 150: 14.30 to 14.01 %). The cuts per source move the roots by at most 0.78 points.
- The budget-aware probabilities add little to the knapsack within β − c_i: 0.3 to 1.7 points of the ratio on `periurban-mixed` and `species-periurban` at 150 and 1000, and at most 0.5 elsewhere on the landscapes. On `bike-squirrel` they do all of `tight`'s work, 1.2 to 7.5 % on its 1 to 15 rows, but no constant moves its root, not even the empty-scenario one.
- μ = 0, with every improvement, gives most rows their `tight` constant. Why, as a reading not measured here: the reach of `tight` to t is the least, over μ, of e^{μ (β − c_i)} times the reach with the improvements of the other options tolled by e^{−μ c_j / k_j}.
  - k_j goes up to 7, as a greening or planting option improves up to 14 arcs.
  - The improvements of p_h are mild.
  - So at μ > 0 the factor outweighs the tolls.

The 31 runs take 67.4 s with main, 61.2 s with `tight` and 64.7 s with the cuts. These are single runs, and no difference stands out from noise.

### Exact constants from a flow MIP per target (2026-11-16)

The owner's suggestion: compute the constant with a MIP, `flow_mip`'s model for the single target t. A study build (not committed) does this for every target with gain rows.

- It builds `flow_mip`'s block of t alone on its graph with the `knapsack_bnb` arc constants, with the budget rows and the conflict sets. The objective is to maximize f_t.
- It solves once without fixing anything: the **scalar** constant, max f_t over the feasible scenarios.
- It then solves once per gain i with x_i ≥ 1: the **per-option** constant M^i_t, the one of row i.
- It takes Gurobi's dual bound, raised by 1e-9 relative, with OptimalityTol 1e-9 and a 10 s limit per solve. The bound stays valid if a solve stops early, but every solve below reached optimality, over 4608 target models.
- One model per target is solved again for each option.

The checks:

- The oracle passes 201 of 201 runs with the per-option constants, in `multiflow_benders`, `--no-in-out` and `--target-gain-cuts`. Its instances have target quality gains.
- At the very small budgets, every per-option constant is at least f_t with i alone. The optimal scores are unchanged wherever both runs solve, 52 runs at the very small budgets and 11 at the suite's.
- At the suite budgets, every exact ratio measured falls within the bracket of the LP and rounding constants measured above. So does every root that compares: `periurban-mixed` at 2500 1.80 within [1.76, 1.81], `urban-greening` at 1000 6.31 within [5.92, 6.53], `urban-greening` at 7500 1.90 within [1.84, 1.93].

**At the very small budgets, 60 s.** The MIPs time counts the MIPs and the second graph of each target. With `urban-greening-large` at 800, the 55 s of MIPs left no time for the rounds, as their clock is half of `--time-limit` from the start.

| run | rows | per option / knapsack | scalar / knapsack | MIPs (s) | root gap: main, `tight`, cuts, MIP scalar, MIP per option, i alone | time (s): main, MIP per option |
|---|---|---|---|---|---|---|
| `dks-30` at 3 | 30 | 1.000 | 1.000 | 0.0 | 0.00, 0.00, 0.00, 0.00, 0.00, – | 0.0, 0.0 |
| `dks-30` at 5 | 30 | 1.000 | 1.000 | 0.0 | 0.00, 0.00, -0.00, 0.00, 0.00, – | 0.0, 0.0 |
| `rural-planting` at 5 | 12 | 0.999 | 0.999 | 0.0 | 43.46, 43.46, 43.46, 43.43, 43.43, 43.43 | 0.2, 0.2 |
| `rural-planting` at 10 | 33 | 0.993 | 0.993 | 0.3 | 17.63, 17.60, 17.63, 17.48, 17.48, 17.40 | 0.3, 0.6 |
| `rural-planting` at 20 | 82 | 0.981 | 0.982 | 3.7 | 8.06, 8.06, 7.78, 7.73, 7.73, 7.12 | 1.0, 4.8 |
| `urban-greening` at 50 | 2 | 1.000 | 1.000 | 0.0 | 19.90, 19.90, 19.90, 19.90, 19.90, 19.90 | 0.1, 0.1 |
| `urban-greening` at 100 | 3 | 1.000 | 1.000 | 0.0 | 4.79, 4.79, 4.79, 4.79, 4.79, 4.78 | 0.1, 0.1 |
| `urban-greening` at 250 | 23 | 0.986 | 0.990 | 0.1 | 23.55, 23.55, 23.12, 23.12, 23.12, 23.04 | 0.2, 0.3 |
| `urban-greening` at 500 | 91 | 0.950 | 0.961 | 5.1 | 6.82, 6.81, 6.29, 6.10, 6.10, 5.06 | 0.4, 5.4 |
| `urban-greening-large` at 50 | 3 | 1.000 | 1.000 | 0.0 | 4.52, 4.52, 4.52, 4.52, 4.52, 4.52 | 0.6, 0.6 |
| `urban-greening-large` at 100 | 9 | 0.999 | 0.999 | 0.0 | 2.15, 2.15, 2.15, 2.14, 2.14, 2.13 | 0.7, 0.7 |
| `urban-greening-large` at 250 | 52 | 0.995 | 0.996 | 0.6 | 9.16, 9.16, 9.08, 9.04, 9.04, 8.98 | 0.7, 1.4 |
| `urban-greening-large` at 800 | 203 | 0.960 | 0.964 | 55.4 | 7.48, 7.47, 6.84, no rounds, no rounds, 6.47 | 6.1, 59.2 |
| `periurban-mixed` at 30 | 5 | 1.000 | 1.000 | 0.0 | 15.17, 15.17, 15.17, 15.17, 15.17, 15.17 | 0.1, 0.1 |
| `periurban-mixed` at 60 | 15 | 0.998 | 0.998 | 0.1 | 17.29, 17.29, 17.27, 17.23, 17.23, 17.17 | 0.2, 0.2 |
| `periurban-mixed` at 150 | 49 | 0.971 | 0.973 | 0.8 | 7.24, 7.24, 7.06, 7.08, 7.08, 6.61 | 0.2, 1.0 |
| `periurban-mixed` at 400 | 55 | 0.982 | 0.984 | 2.0 | 3.98, 3.98, 3.28, 3.76, 3.76, 2.23 | 1.8, 4.6 |
| `periurban-mixed` at 1000 | 80 | 0.912 | 0.934 | 5.2 | 2.58, 2.56, 1.94, 2.21, 2.21, -0.88 | 12.5, 16.2 |
| `species-periurban` at 30 | 10 | 1.000 | 1.000 | 0.0 | 17.40, 17.40, 17.40, 17.40, 17.40, 17.40 | 0.2, 0.2 |
| `species-periurban` at 60 | 30 | 0.998 | 0.998 | 0.1 | 10.48, 10.47, 10.46, 10.45, 10.45, 10.35 | 0.3, 0.4 |
| `species-periurban` at 150 | 98 | 0.962 | 0.965 | 2.5 | 14.30, 14.01, 13.52, 13.49, 13.47, 13.03 | 1.7, 4.9 |
| `species-periurban` at 400 | 110 | 0.978 | 0.981 | 6.0 | 26.59, 26.59, 25.84, 26.19, 26.19, 24.41 | 8.5, 13.9 |
| `species-periurban` at 1000 | 160 | 0.911 | 0.928 | 19.0 | 3.96, 3.94, 3.48, 3.63, 3.59, 0.69 | 21.9, 39.6 |
| `bike-squirrel-town` at 100 | 4 | 0.961 | 0.993 | 0.1 | 21.72, 21.72, 21.72, 21.72, 21.72, 21.72 | 3.1, 1.6 |
| `bike-squirrel-suburb` at 30 | 1 | 0.946 | 0.978 | 0.0 | 1023.30, 1023.30, 1023.30, 1023.30, 1023.30, 1023.30 | 1.2, 1.1 |
| `bike-squirrel-suburb` at 100 | 15 | 0.857 | 0.997 | 0.3 | 45.83, 45.83, 45.83, 45.83, 45.83, 45.83 | 4.4, 5.3 |

**At the budgets of the short suite, 120 s.** Main ran again beside the MIP version. A root compares only where the rounds ended before their 60 s clock: `periurban-mixed` at 2500 and 6000, the three `rural-planting` runs, and `urban-greening` at 1000 and 7500. At `urban-greening-large` at 8000, the MIPs took 147 s, and the master then stopped at the limit without a solution.

| run | rows | per option / knapsack | MIPs (s) | root gap: main, MIP per option | time: main, MIP per option | nodes: main, MIP per option |
|---|---|---|---|---|---|---|
| `periurban-mixed` at 2500 | 118 | 0.948 | 22.0 | 1.96, 1.80 | 9.1 s, 32.4 s | 52, 78 |
| `periurban-mixed` at 6000 | 126 | 0.896 | 43.7 | 1.56, 1.34 | 18.4 s, 63.3 s | 326, 142 |
| `periurban-mixed` at 12000 | 126 | 0.952 | 58.0 | 0.29, 2.59 | 18.7 s, 75.6 s | 11, 826 |
| `rural-planting` at 90 | 122 | 0.982 | 21.3 | 0.51, 0.31 | 1.3 s, 22.6 s | 1, 1 |
| `rural-planting` at 260 | 122 | 0.991 | 29.7 | 0.81, 0.68 | 12.8 s, 39.0 s | 156, 91 |
| `rural-planting` at 510 | 122 | 0.994 | 30.3 | 0.36, 0.28 | 5.7 s, 36.5 s | 1, 1 |
| `urban-greening` at 1000 | 123 | 0.941 | 16.4 | 7.65, 6.31 | 1.3 s, 17.7 s | 1, 1 |
| `urban-greening` at 7500 | 123 | 0.973 | 52.1 | 2.67, 1.90 | 8.4 s, 61.5 s | 338, 55 |
| `urban-greening` at 15000 | 123 | 0.986 | 62.4 | 0.68, no rounds | 5.1 s, 99.1 s | 1, 8219 |
| `urban-greening-large` at 1600 | 207 | 0.969 | 70.3 | 2.99, no rounds | 5.8 s, 76.9 s | 1, 151 |
| `urban-greening-large` at 4000 | 207 | 0.979 | 115.5 | 1.04, no rounds | 11.7 s, limit, bound 6.58 % | 5, 860 |
| `urban-greening-large` at 8000 | | | 147.4 | | no solution at the limit | |
| `species-periurban` at 2500 | 236 | 0.942 | 56.9 | 1.78, 1.67 | 36.5 s, 73.6 s | 11, 9 |
| `species-periurban` at 6000 | 252 | 0.898 | 99.6 | 4.26, no rounds | 53.4 s, limit, bound 16.12 % | 1465, 1679 |

**What the MIP gives.**

- The exact constants lie 0 to 9 % below `knapsack_bnb`'s at the very small budgets on the landscapes (4 to 14 % on the few rows of `bike-squirrel`), and 0.6 to 10 % below at the suite's. That is far from f_t with i alone: the room that the i-alone column suggests is mostly not there.
- Scalar and per-option constants give the same roots to 0.04 points, as most targets have a single gain.
- The roots move by at most 0.83 points at the very small budgets (`species-periurban` at 150: 14.30 to 13.47 %) and by 0.08 to 1.34 at the suite's (`urban-greening` at 1000: 7.65 to 6.31 %).
- Where at most 7 options fit, the exact constants and the cuts per source are within 0.2 points of each other. Beyond that, the cuts are tighter by 0.11 to 1.25 points (`urban-greening` at 7500: 0.65 % against 1.90 %). The one exception is `urban-greening` at 1000, at 6.31 % against 6.35 %. The cuts at the suite budgets come from the 30 s table, whose main roots are the same.

**What it costs.**

- One MIP per gain row and one per target, 0.1 to 0.4 s per solve on average, graphs included: 16 to 147 s per suite run, and up to 55 s at the very small budgets.
- Every suite run is slower, 2 to 19 times. Three of the 14 no longer finish in 120 s: `urban-greening-large` at 4000 and 8000, and `species-periurban` at 6000.
- Net of the MIPs, the seven runs whose roots compare take 1.3 to 19.6 s against main's 1.3 to 18.4 s. Only `rural-planting` at 260 is faster, 9.3 against 12.8 s. Even free exact constants would buy little time.

## Checks

- **Default mode.** The parity harness gives the same 851 cases as main b1ba10e, Gurobi models included, once the lines of the two new parameters are ignored. This includes `flow_mip --big-m tight`, whose header gained `with_arcs`. ctest passes 76 of 76, without warnings.
- **The oracle (`test/brute_force/check.py`).** 469 of 469 runs with `multiflow_benders` and `--target-gain-cuts`, `--target-gain-cuts --no-in-out`, `--target-gain-cuts --no-stored-arcs`, `--big-m tight`, `--big-m plain` and `--big-m knapsack_dantzig`. With `max_pushes` 0 and 1, 201 of 201 runs with the three `--target-gain-cuts` configurations, so through the fallback of a source that gives up.
- **A fuzz (scratchpad, not committed).** Random graphs of 7 to 12 vertices and 7 to 11 options, most improving the quality of one or two vertices for both source and target, at three budgets, against brute force:
  - 360 of 360 runs with `--target-gain-cuts`, with and without `--no-stored-arcs`, with 2430 gain cuts in each mode;
  - 180 of 180 runs at each of `max_pushes` 0 and 1, with 1625 and 1664 gain cuts.

## Traps met

- **The constants of a scenario that buys i only bound f^i_t, not f_t.** The bound of f_t needs every scenario.
- **An exact or LP constant must cover the budget plus the feasibility tolerance**, which the solver accepts. The study's LP constants used the budget alone: valid for the study, not for an implementation.
- **A mippp `variable_params` written with designated initializers has no lower bound.** `lower_bound` is then `std::nullopt`, which Gurobi reads as −∞; `default_variable_params` has 0. The study LP's x went to −55 before `.lower_bound = 0.0` was added.
  - The f_t column of main, `{.upper_bound = M_t}`, has no lower bound for the same reason.
  - It is harmless there: nothing in the model gains from a negative f_t.

## Leads, not tried

- **One flow for both.** The supply of a source for f^i_t is at most its supply for f_t, so its flow by highest-gain paths is a prefix of the one for f_t. The labels when that smaller supply is spent would give the cut of f^i_t without a second flow.
- **Fewer gain cuts.** Separate them only in the plain rounds after in-out, or only for the targets whose row f^i_t ≤ M^t_t · x_i is tight at the optimum.
- **The arc copies, partly measured.** The study of `flow_mip`'s tight constants (`misc/design/tight_big_m.tex`, 2026-11-08) measured one factor of the constant of a copy, its budget-aware probability of connection. Replacing it with its LP value or its exact value (bi-objective label setting) improved the constants by no more than a few percent over its six Lagrange multipliers. It calls the exact constant of a copy intractable and did not compute it.
