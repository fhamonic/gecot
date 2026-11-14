# multiflow_benders: where the separation spends its time, and what could be tried

Status, 2026-11-14: a record of measurements and of leads that were not followed. Nothing here is implemented beyond what the first two sections say. The code is `include/gecot/solvers/multiflow_benders.hpp` at commit da6b556; the algorithm is stated in `docs/formulations/benders.md`, "`multiflow_benders`: cuts per source".

## What a separation does

For a target t and a point x̂, `target_block::separate` builds one cut. For each source s of the block, `_largest_flow` computes the largest flow of s to t on its arcs A_s, where an improved copy a of the option i carries at most m_sa · x̂_i:

1. a search of labels from t gives ρ_u, the gain of the best path from u to t in the residual graph (`label`, a queue-based label-correcting search: a residual arc taken backward has a gain above 1, which rules Dijkstra out);
2. the path of s takes what its arcs allow (a *push*);
3. the two steps repeat until the supply of s is sent or s no longer reaches t;
4. the labels of the last search are an optimal dual solution, and give the part of s in the cut.

The blocks are separated in parallel. The cut is separated on the linear relaxation before the search (the *rounds*), then at each candidate of the solver.

## What was measured

Machine: 16 cores, Gurobi 12. Instances of `benchmark/instances`, short suite, 30 s per run.

- **The searches of labels are the cost.** They take 85 to 97 % of the CPU time of the separation: 68 s of 79 s on `species-natural` at budget 6000, in 12.8 million searches; 55 s of 57 s on `bike-squirrel-town` at 300. Setting up the arcs of a source and computing its coefficients are the rest.
- **A source needs about three searches** on average at the points of the relaxation: one before its first push, and one after each push that fills an arc.
- **The sets A_s are large.** On `species-natural`: 714 targets, 113,969 sources and 46.8 million pairs of a source and an arc, about 410 arcs per source, which is most of the arcs of a block. The test of A_s removes little there.
- **The rest is minor.** On the same instance: the labelling of the strong and useless arcs 65 ms, the contracted graphs 45 ms, the big-M of the targets 5 ms, the sources (`compute_sources`) 0.2 s.
- **Where a run spends its time**, same instance, 12 s in all: 3.2 s in the separation of the rounds, 0.5 s in the linear programs of the rounds, 1.6 s in the separation at the candidates, the rest in the solver.
- **The fallback never fires.** In 8 million separations of a source on ten instances, no search gave up.

The pass of 2026-11-14 (commits 7f7d9f2 to da6b556) kept the cuts as they were, bit for bit, and brought the 31 runs of the suite that reach optimality from 255 s to 231 s, and the peak memory on `species-natural` from 1.45 GB to 1.00 GB.

## Tried and dropped

- **One Dijkstra per target, shared by the sources.** The labels of a search from t over every arc open at x̂ are a feasible dual solution for each source, and an optimal one for a source whose best path fills no arc. Those sources then need no search of their own. Valid, and the oracle passes, but: no faster on the suite (226 s against 220 s on the 31 runs), because at the points of the relaxation most sources fill an arc and search anyway; and the cuts are weaker, the shared labels being larger than a source's own away from its path (`species-natural` at 12000: 388 nodes against 14, 8858 lazy cuts against 770).
- **`bool` flags in place of `char` ones** in `flow_search`, against the aliasing of `char` writes: no measurable change.

## Leads

Each one changes the labels in their last digits at least, so none can be checked bit for bit: the check is the oracle (`test/brute_force/check.py` on the three configurations of `multiflow_benders`), the guards forced (below), and the suite, comparing the cuts on the relaxation, the lazy cuts and the nodes, not only the time.

1. **Dijkstra for the first search of a source.** Before any push, the residual graph has no backward arc, so the gains are at most 1 and a label-setting search applies. melon's `dijkstra` on the reverse view, with a length that is 0 outside A_s and on a copy of capacity 0, and one object reused from a source to the next, as `compute_sources` does. It replaces about a third of the searches by a search that scans each vertex once. Its labels are exact, where `label` keeps a label within `label_tie` of the best: the cuts move in their last digits.
2. **Updating the labels after a push.** A push changes the residual graph on its path only: the arcs it fills close, and its arcs can be taken backward. The second search starts again from t. Recomputing only the labels that depended on a filled arc would cut most of the two other thirds. It is the delicate one: the set of labels to lower is found by following `next` backward, and a mistake gives labels that are not a feasible dual solution, hence an invalid cut.
3. **Keeping the part of a source from a round to the next.** The part of s depends on x̂ through the options of its quality gains and of the improved copies of A_s only. When none of them moved since the last separation, the part is the same. It needs the part of each source kept sparse (its ρ_s and its coefficients), which is memory in the order of the pairs of a source and an improved copy. Worth it in the late rounds, where few options move; useless at the candidates, which differ from one another.
4. **melon's `bellman_ford_moore` in place of `label`.** It is the same queue search. It does not fit as it is: it has no tolerance on an improvement and no guard, and an arc taken forward then backward is a cycle of gain fl(π · 1/π), which rounding can put above 1, its "negative cycle". It would need the residual graph as a view (the arcs of A_s with capacity left, reversed, and the arcs with flow), a semiring whose comparison carries `label_tie`, and its detection of cycles as the guard. A gain in code, not in time.
5. **Smaller A_s.** The test keeps an arc when the best path of s through it, with every option, is as reliable as its best path without any. A test within the budget (the best path through the arc with the options that the budget allows) would remove more, and shorten every search. It is the tight big-M of `docs/formulations/preprocessing.md` applied to the membership of A_s; the cut stays valid as long as the test is.

## How to measure a change

```console
$ python3 benchmark/run.py --suite short --time-limit 30 --label before --gecot <binary before> multiflow_benders
$ python3 benchmark/run.py --suite short --time-limit 30 --label after  --gecot <binary after>  multiflow_benders
$ python3 benchmark/compare.py benchmark/results/before.csv benchmark/results/after.csv
```

Run alone on the machine: the suite varies by about 5 % from one run to the next. The reports of the runs (`benchmark/results/<label>/*.json`) hold `root_rounds`, `root_cuts`, `lazy_constraints` and `nodes`: a run that reaches optimality with the same cuts and nodes computed the same thing.

To exercise the fallback, build once with `max_label_steps = 0` in `target_block` (every search gives up after its first vertex), once with `max_pushes = 1` (every source that needs a second push gives up), and run the oracle on each: 111 of 111 runs must pass.
