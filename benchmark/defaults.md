# The defaults and what backs them

One row per default of an algorithm: what it was compared with, on what, when, what came out, and where the record is. It was filled on 2026-11-17 from the design notes, the docs pages and the bodies of the commits of main; no run was made for it.

- **When a default is measured**, add or replace its row, with the command of the runs and their label: `benchmark/results` is not tracked, so a row that only names a label there can be reproduced, not read.
- **"Nothing"** means that no measurement was found in the repository, not that none was ever made.
- The suite is the short one of `README.md`, 42 runs, unless the row says otherwise; Gurobi, unless the row names other solvers.

## Measured on the benchmark suite

| Parameter | Default | Compared with | Runs | Date | Result | Record |
|---|---|---|---|---|---|---|
| `multiflow_benders --big-m` | `knapsack_bnb` | `tight`, `knapsack_dantzig`, `plain` | the 23 runs of the suite with target quality gains, 30 s, on a study build | 2026-11-15 | solved and seconds over them: `knapsack_bnb` 19, 155.4 s; `tight` 19, 150.5 s; `knapsack_dantzig` 20, 192.7 s; `plain` 19, 175.6 s. The root gaps of `tight` are those of `knapsack_bnb` to two decimals, but on one run whose rounds the clock stopped; those of `plain` are up to 7 points wider | `misc/design/multiflow_target_big_m.md` |
| `multiflow_benders --no-stored-arcs` | off: the sets A_s are kept | on | 30 s | 2026-11-15 | the 31 runs solved take 201.2 s kept, 230.1 s rebuilt (+14 %); 30 are identical in rounds, cuts and nodes, the other in cuts, nodes and score; peak memory on `species-natural` at 6000: 680 MB kept, 241 MB rebuilt | `misc/design/multiflow_benders_separation.md` |
| `--numeric-focus` | off | on | `flow_mip` and `multiflow_benders`, 30 s, one run each | 2026-11-14 | `flow_mip`: 12 runs proved optimal both ways, 51.6 s off against 67.8 s on. `multiflow_benders`: 30 off, 31 on. Mixed: the owner's choice | `misc/design/maintainability_review.md`, Implementation log |
| `multicut_benders --groups` | 2 | 0, a group per target | 30 s | 2026-11-15 | **against the default**: 17 runs solved with 2 groups, 20 with `--groups 0` | `misc/design/multicut_root_cuts.md` on the unmerged branch `multicut-root-cuts` |
| `multiflow_benders --root-cut-tolerance` | 1e-7 | 1e-3 | the bike instances, the rounds only | 2026-11-16 | 1e-3 ends the rounds in about half the time, for a bound higher by 0.02 to 0.09 points of gain | `docs/formulations/benders.md` |
| `multiflow_benders --root-cut-gap` | 0: never | 1e-3 | the same | 2026-11-17 | 1e-3 ends the rounds 55 to 69 % sooner, for a bound higher by about 0.04 points of gain | `docs/formulations/benders.md` |

## Measured on other instances

| Parameter | Default | Compared with | Runs | Date | Result | Record |
|---|---|---|---|---|---|---|
| `--big-m` of `flow_mip`, `flow_benders`, `multicut_benders` | `knapsack_bnb` | `exact` | `rural-planting` at 90, one run | 2026-11-16 | 67.7 s, 1.2 s and 2.1 s with `knapsack_bnb`; 227.1 s, 169.6 s and 170.3 s with `exact`, of which about 169 s for the constants. Not compared in time with `knapsack_dantzig` or `tight` | `misc/design/exact_big_m.md` |
| `--nb-mus`, `--mus-sweep` | 0, `decades` | `--nb-mus` 9 and 20, `full` | Aix, Quebec, bike crops | 2026-11-05 to 13 | the constrained labels pay only at small absolute budgets | `misc/design/lagrangian_preprocessing.md`, Summary |
| `--preprocessing`, `--all-targets`, `--no-pruning`, `--no-merging`, `--big-m plain` | every step on | each step off | `flow_mip` on Aix 30 × 30 at 8, one run each | – | an order of magnitude, in the table of the page | `docs/formulations/preprocessing.md`, "What each step brings" |
| `path_lp --pricing` | `search` | `enumerate` | Quebec, bike, green, N2 | 2026-10-29 to 11-12 | `enumerate` does not scale: 2.2 GB against 457 MB on Quebec at 10000, no end on bike | `misc/design/path_lp.md`, §5 |
| `path_lp --evict-age` | 0: never | 3 | Aude, Aix, Quebec; bike_small at 200 | 2026-11-03 | neutral or slower; on bike_small the evicted columns come back in a cycle | `misc/design/path_lp.md` |
| `path_lp --max-activations` | 0: every improving column | 5000 | Quebec at 1000 and 10000 | – | **against the default**: 5.44 s uncapped, 3.69 s capped; 28.82 s and 15.75 s. The cap has no rule | `misc/design/path_lp.md` |
| `static --decremental`, `greedy --decremental` | off | on | Aix 40 × 40 at 8, one run | – | `static`: 432,401.8 off, 418,296.9 on. `greedy`: the same score, 0.2 s off, 0.5 s on | `docs/usage/choosing.md` |

## Recorded in the body of a commit only

| Parameter | Default | Compared with | Runs | Date | Result | Record |
|---|---|---|---|---|---|---|
| `multicut_benders --grouping` | `voronoi` | `consecutive`, `random` with three seeds | 12 runs on Aix, Quebec and a bike crop, 60 s | 2026-11-10 | against every seed by 5 % or more, `voronoi` is better on 6 of the 12 and never worse; `consecutive` on 3 | c58f74b |
| `multicut_benders --groups` | 2 | 1, and a group per target | Aix 40 × 40 and 70 × 70, 60 s, with consecutive groups | 2026-11-10 | with 2 groups, one, and a group per target: 1.9 s, 1.1 s and 33.1 s on Aix 40 × 40 at 8; gaps of 3.71 %, 4.69 % and 12.62 % on Aix 70 × 70 at 10 | edff7c0 |
| `multiflow_benders --no-root-cuts`, `--in-out-ratio` | off: the rounds; `0.5` | the flag on; `1`, without the center, and no other step | Aix 40 × 40 and 70 × 70 at 8, 60 s | 2026-11-12 | Aix 70 × 70: 4.2 s; 15.2 s without the center; the limit without the rounds | fb1f619 |
| `--mip-start-algorithm` | `none` | `greedy` | Aix 70 × 70 at 10, 60 s, on `naive_benders`, removed since, and on `flow_mip` | 2026-11-10 | **against the default**: a gap of 2.38 % with the start against 4.75 %, and 0.95 % against 1.60 % | 57e13f3 |
| The barrier of the master of `tree_lp`, taken when nothing is evicted | on, where the solver has it | the simplex | Aude at 8, biobj-Aude at 5, Quebec at 20, on Xpress and MOSEK | 2026-11-11 | fewer rounds: 41 to 17 and 46 to 18 on Aude | c68c0b9 |

## Nothing

`-t 1e-7`; `--resolution 1e-8`, which every study ran at and none varied; `--indicator-constraints` off; `path_lp --relative-gap 0`; `tree_lp --evict-age 0`, which has the argument of its comment (with the barrier, the evicted columns come back in a cycle) and no run.

## Constants that no parameter reaches

A guard keeps a search valid or finite. A tunable is a choice that an experiment could move: it would first have to become a parameter, or a template parameter as `MaxPushes` is.

| Constant | Where | Kind | Record |
|---|---|---|---|
| The pricing margin, 1e-7 σ + 1e-6 | `pricing_margin`, `column_generation.hpp` | tunable | validity only (`misc/design/path_lp.md`); never varied. Absolute, in the unit of the solver |
| The eviction threshold, 1e-6 | `unattractive_columns`, `column_generation.hpp` | tunable | nothing |
| The tolerance 1e-8 and the plain big-Ms of the pricing models | `tree_lp.hpp`, `pricing_model` | tunable | nothing. The tolerance does not follow `-t`, which the master does |
| `root_cut_abs_tol`, 1e-7 | `multiflow_benders.hpp` | tunable | the reason in its comment; no run |
| `root_time_share`, 0.5 | `multiflow_benders.hpp` | tunable | nothing; two design notes record runs that ended their rounds on it |
| `path_tie`, 1e-9 | `multiflow_target_block.hpp` | tunable | the reason in its comment; no run |
| `mip_gap`, 1e-10 | `solve_result.hpp` | tunable | nothing |
| `arc_full` 1e-11, `negligible_flow` 1e-12 | `onaga_default_traits`, `melon/algorithm/onaga.hpp` (melon 1.1.0) | guard | nothing |
| `MaxPushes`, 100 000 | `multiflow_target_block.hpp` | guard | no search reached it in 8 million separations of a source (`misc/design/multiflow_benders_separation.md`); it is tested at 0 and 1 |
| Gurobi's `OptimalityTol`, 1e-9 | `_set_search`, `mip_solver_base.hpp` | guard | one observation at the default 1e-6, in its comment |
| `tie_window`, 1e-12 | `heuristics_common.hpp` | guard | 807 steps of the short suite, in the comment of `greedy.hpp` (3c8be7a) |
| The grain 64 of the parallel sums | `parallel_pc_num.hpp`, `split_lagrange.hpp` | guard | it fixes the order of a sum: another value changes the last bits of every score |
