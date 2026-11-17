# multiflow_benders: where the separation spends its time, and what could be tried

Status, 2026-11-15: a record of measurements and of leads. The search of labels is Dijkstra's algorithm since the branch `multiflow-dijkstra`, which did what the first two leads of 2026-11-14 asked for: "Dijkstra on reduced gains" says what it changed. Since the branch `multiflow-no-source-arcs`, a pruned Dijkstra computes the sets A_s, and `--no-stored-arcs` rebuilds them at each separation: "A_s rebuilt at each separation". "What was measured" is the label-correcting search of commit da6b556. Nothing else here is implemented. The code is `include/gecot/solvers/multiflow_benders.hpp`; the algorithm is stated in `docs/formulations/benders.md`, "`multiflow_benders`: cuts per source".

## What a separation does

For a target t and a point x̂, `target_block::separate` builds one cut (the block is `multiflow_target_block`, in `include/gecot/solvers/multiflow_target_block.hpp` since 2026-11-17). For each source s of the block, `_largest_flow` computes the largest flow of s to t on its arcs A_s, where an improved copy a of the option i carries at most m_sa · x̂_i:

1. a search of labels from t gives ρ_u, the gain of the best path from u to t in the residual graph (`label`, Dijkstra's algorithm, on gains reduced by the labels of the search before from the second search of a source on);
2. the path of s takes what its arcs allow (a *push*);
3. the two steps repeat until the supply of s is sent or s no longer reaches t;
4. the labels of the last search are an optimal dual solution, and give the part of s in the cut.

The blocks are separated in parallel. The cut is separated on the linear relaxation before the search (the *rounds*), then at each candidate of the solver.

## What was measured, 2026-11-14

Machine: 16 cores, Gurobi 12. Instances of `benchmark/instances`, short suite, 30 s per run.

- **The searches of labels are the cost.** They take 85 to 97 % of the CPU time of the separation: 68 s of 79 s on `species-natural` at budget 6000, in 12.8 million searches; 55 s of 57 s on `bike-squirrel-town` at 300. Setting up the arcs of a source and computing its coefficients are the rest.
- **A source needs about three searches** on average at the points of the relaxation: one before its first push, and one after each push that fills an arc.
- **The sets A_s are large.** On `species-natural`: 714 targets, 113,969 sources and 46.8 million pairs of a source and an arc, about 410 arcs per source, which is most of the arcs of a block. The test of A_s removes little there.
- **The rest is minor.** On the same instance: the labelling of the strong and useless arcs 65 ms, the contracted graphs 45 ms, the big-M of the targets 5 ms, the sources (`compute_sources`) 0.2 s.
- **Where a run spends its time**, same instance, 12 s in all: 3.2 s in the separation of the rounds, 0.5 s in the linear programs of the rounds, 1.6 s in the separation at the candidates, the rest in the solver.
- **The fallback never fires.** In 8 million separations of a source on ten instances, no search gave up.

The pass of 2026-11-14 (commits 7f7d9f2 to da6b556) kept the cuts as they were, bit for bit, and brought the 31 runs of the suite that reach optimality from 255 s to 231 s, and the peak memory on `species-natural` from 1.45 GB to 1.00 GB.

## Dijkstra on reduced gains, 2026-11-15

`label` was a label-correcting search with a queue: a residual arc taken backward has a gain 1/π above 1, which a label-setting search cannot take as it is. It is now Dijkstra's algorithm from t, the highest label first, on melon's updatable 4-ary heap:

- the first search of a source has no arc taken backward: its gains are at most 1, and the vertices come in the order of their labels;
- the next ones come in the order of ρ_u / p_u, p being the labels of the search before. A push changes the residual graph on its path only: the arcs it fills leave it, its arcs can be taken backward, and an arc it took backward can open forward again. The labels p are tight on the path, so they stay a feasible dual solution of the new residual graph: every reduced gain g · p_v / p_u is at most 1. A vertex that reaches t reaches the path by arcs that were there before, so the search before reached it too, and p is only read where it was set.

The labels are exact, where the queue search kept a label within `label_tie` = 1e-12 of the best. The guards `max_label_steps` and `max_path_arcs` are gone: a search ends, and a path follows `next` to vertices settled earlier, so it reaches t. The fallback, still taken past `max_pushes`, is `label` on the flow reset to 0.

Checked:

- with an instrumented build (counters, not committed), on `species-natural` at 6000, `periurban-mixed` at 6000, `bike-squirrel-town` at 300 and `bike-town` at 500: 23.0 million searches, 16.8 million of them after a push. No label violates the dual on a residual arc of A_s by more than 1e-12 (the largest violation is 4.4e-16), no reduced gain exceeds 1 + 1e-9 (the largest is 1 + 4.4e-16), no search reaches a vertex that the search before did not, and in every search the labels equal those of the queue search run on the same residual graph within 1.2e-15. No fallback;
- the oracle on the three configurations: 201 of 201 runs, and 201 of 201 again with `max_pushes = 0` and with `max_pushes = 1`.

The short suite at 30 s per run, one run of each, alone on the machine, before (main afc652e) and after:

- the same status in the 42 runs, 31 optimal, with the same scores; no bound and no root value of the 42, before or after, under the best score known (`benchmark/best_known.csv` and both runs);
- these 31 take 227.7 s before, 201.4 s after; building the model, which holds the rounds on the relaxation (`model_build_time_ms`), takes 187.4 s before, 155.9 s after over the 42;
- 36 runs have the same rounds and cuts on the relaxation, and the same root value to the last bit. The 6 others ended their rounds on the clock (half of the 30 s) before the change: `periurban-mixed` at 12000, `urban-greening-large` at 8000 and `species-periurban` at 6000 now converge, the other three do more rounds in the same time (`bike-squirrel-suburb` at 500: 10 rounds and 61.7 % of the best gain guaranteed before, 14 and 78.0 % after);
- lazy cuts and nodes move in a few runs (`species-natural` at 6000: 1233 lazy cuts and 37 nodes before, 726 and 33 after; `urban-greening-large` at 4000: 1 node before, 5 after), with no blow-up such as the shared Dijkstra's below.

| building the model, with the rounds | before | after |
|---|---|---|
| `bike-town` at 500 | 11.7 s | 5.7 s |
| `periurban-mixed` at 6000 | 12.7 s | 8.0 s |
| `bike-squirrel-town` at 300 | 6.5 s | 3.7 s |
| `urban-greening` at 15000 | 6.2 s | 4.0 s |
| `species-natural` at 6000 | 6.1 s | 6.3 s |
| `species-natural` at 12000 | 5.1 s | 5.4 s |

The 5 % on `species-natural` is within the noise of the suite: no gain there. Counted with the instrumented build, the queue search run on the same residual graphs scans 2.17 billion vertices on `species-natural` at 6000, where Dijkstra scans 1.12 billion, 1.9 times fewer; on `periurban-mixed` at 6000, 3.87 and 1.43 billion (2.7 times); on `bike-squirrel-town` at 300, 2.20 and 0.57 billion (3.9 times); on `bike-town` at 500, 3.03 and 0.69 billion (4.4 times). Where the queue search scans a vertex about twice, the heap costs what the scans save.

## A_s rebuilt at each separation, 2026-11-15

Since the branch `multiflow-no-source-arcs`: `--no-stored-arcs`, off by default (the owner's choice: the sets stay kept by default). The sets A_s with the m_sa of their improved copies are computed once and kept, 4 bytes per arc and 16 per improved copy. Their sizes add up to at most the arcs of G_t times its sources, summed over the targets. On `hexagon_instances/1_1900_3800` at budget 151: 1456 targets, 2.08 million sources, 8.92 billion pairs of a source and an arc, of which 2.14 billion improved copies, at least 70 GB before the slack of the vectors. G_t barely shrinks there (4077 of 4368 vertices), and A_s keeps 36 % of its arcs, the test counting every option where the budget buys 5 % of their cost.

With the flag, a block keeps its sources and the two maps of the test, Π(·, t) and Π̄(·, t), and each separation rebuilds A_s and the m_sa of a source before its flow, with the forward Dijkstra and the test of before. Without it, the same function computes them once, as before. That Dijkstra does not go on from a vertex u with Π̄(s, u) · Π̄(u, t) under the threshold Π(s, t) · (1 − 10⁻⁹), less a relative 10⁻¹² for the rounding of the test: no arc of A_s leaves u, nor any vertex reached through it, and the vertices it does go on from keep their exact Π̄(s, u). A_s and the m_sa are those of before, so are the cuts.

The measurements below were made before the flag, with builds that always rebuild.

- Hexagon at 151, 900 s, rebuilt without the pruning: 1.8 GB through the rounds, 4.7 GB at the peak. 9 rounds in 450 s, root value +103.57 % of gain; at the limit +81.5 % for a bound of +103.5 %, 78.7 % guaranteed. The peak is the master, 18.2 million nonzeros over 15,879 rows: the cuts are nearly dense in the 1386 options, about 2 million nonzeros more per round or separation at a candidate, so the master is the next wall.
- Hexagon at 151, 100 s, the two rebuilds: on 4 separations (8.3 million sources), the rebuild takes 1254 s of CPU without the pruning, 738 s with it, against 869 s for the flows, and the same 35.7 billion pairs. A round goes from 36 s to 28 s; the flows alone would take about 14 s.
- The short suite at 30 s, against main 7e2fd43: the 31 runs that reach optimality have the same rounds, cuts, lazy cuts, nodes, root values and scores, bit for bit; of the 11 others, 9 have the same rounds, cuts and root value, and 2 do fewer rounds before the clock. No bound or root value under the best known score. The 31 take 201.4 s before, 228.1 s rebuilt, 225.4 s with the pruning (+12 %); the rounds 155.9, 178.7, 176.4 s. The pruning hardly helps there, where A_s holds most of a small G_t.
- Peak memory: `species-natural` at 6000 680 MB before, 241 MB after; `urban-greening-large` at 1600 1021 and 258 MB; `bike-town` at 500 287 and 249 MB.
- The oracle: 201 of 201 runs, before and after the pruning.
- With the flag, the debug line of the sources no longer counts their arcs, which are not kept.
- With the flag's build, the short suite at 30 s against main 7e2fd43. Without the flag, the 31 optimal runs are identical in rounds, cuts, lazy cuts, nodes, root values and scores (201.2 s against 201.4 s). With it, 30 of them are identical, and `urban-greening-large` at 8000 has the same cuts, nodes and score but ends its rounds on the clock at 15.2 s, one round short of the one that adds nothing; the 31 take 230.1 s (+14 %). The oracle on five configurations, two of them with the flag: 335 of 335 runs; on the four of `check.py` with `max_pushes` at 0 and at 1: 268 of 268 each. Hexagon at 151 over 100 s: 1.2 GB with the flag, two rounds by 56.4 s; without it, past 6 GB within seconds of the setup. The parity captures of main and of the branch are the same in their 837 common cases, models included, and each case with the flag matches the same case without it.

## Tried and dropped

- **One Dijkstra per target, shared by the sources.** The labels of a search from t over every arc open at x̂ are a feasible dual solution for each source, and an optimal one for a source whose best path fills no arc. Those sources then need no search of their own. Valid, and the oracle passes, but: no faster on the suite (226 s against 220 s on the 31 runs), because at the points of the relaxation most sources fill an arc and search anyway; and the cuts are weaker, the shared labels being larger than a source's own away from its path (`species-natural` at 12000: 388 nodes against 14, 8858 lazy cuts against 770).
- **`bool` flags in place of `char` ones** in `flow_search`, against the aliasing of `char` writes: no measurable change.

## Leads

A change that moves the labels cannot be checked bit for bit: the check is the oracle (`test/brute_force/check.py` on the four configurations of `multiflow_benders`), the fallback forced (below), and the suite, comparing the cuts on the relaxation, the lazy cuts and the nodes, not only the time.

1. **Keeping the part of a source from a round to the next.** The part of s depends on x̂ through the options of its quality gains and of the improved copies of A_s only. When none of them moved since the last separation, the part is the same. It needs the part of each source kept sparse (its ρ_s and its coefficients), which is memory in the order of the pairs of a source and an improved copy. Worth it in the late rounds, where few options move; useless at the candidates, which differ from one another.
2. **Smaller A_s.** The test keeps an arc when the best path of s through it, with every option, is as reliable as its best path without any. A test within the budget (the best path through the arc with the options that the budget allows) would remove more, and shorten every search. It is the tight big-M of `docs/formulations/preprocessing.md` applied to the membership of A_s; the cut stays valid as long as the test is.
3. **A cheaper search where the heap does not pay**, as on `species-natural`: the queue search for the first search of a source, which takes no arc backward, or a heap of another arity. Untested.

## How to measure a change

```console
$ python3 benchmark/run.py --suite short --time-limit 30 --label before --gecot <binary before> multiflow_benders
$ python3 benchmark/run.py --suite short --time-limit 30 --label after  --gecot <binary after>  multiflow_benders
$ python3 benchmark/compare.py benchmark/results/before.csv benchmark/results/after.csv
```

Run alone on the machine: the suite varies by about 5 % from one run to the next. The reports of the runs (`benchmark/results/<label>/*.json`) hold `root_rounds`, `root_cuts`, `lazy_constraints` and `nodes`: a run that reaches optimality with the same cuts and nodes computed the same thing.

To exercise the fallback end to end, build once with the alias `target_block` of `multiflow_benders` set to `multiflow_target_block<0>` (`max_pushes` is the template parameter of the block: every source gives up after its first search), once to `multiflow_target_block<1>` (every source that needs a second push gives up), and run the oracle on each: 268 of 268 runs must pass. Without a build, the unit test `multiflow_cuts_are_valid_and_tight` checks the cuts of the block at 0 and at 1 on three small instances: valid in every scenario, tight at a scenario.
