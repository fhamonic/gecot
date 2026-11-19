# multiflow_benders: where the separation spends its time, and what could be tried

Status, 2026-11-18: a record of measurements and of leads. Since the branch `multiflow-flow-study`, the pushes of a source are found by searches from the source and its labels by one search from t: "The flow search, 2026-11-17" says what was asked, counted, tried and kept, and the sections before it describe the search it replaces. The search of labels is Dijkstra's algorithm since the branch `multiflow-dijkstra`, which did what the first two leads of 2026-11-14 asked for: "Dijkstra on reduced gains" says what it changed. Since the branch `multiflow-no-source-arcs`, a pruned Dijkstra computes the sets A_s, and `--no-stored-arcs` rebuilds them at each separation: "A_s rebuilt at each separation". Prefetches in the searches were tried and not kept: "Prefetching in the searches, 2026-11-18". The statuses of the flow search are epochs instead of being reset: "Status epochs, 2026-11-18". On the branch `multiflow-onaga`, the flow of a source is computed by `gecot::onaga`, a draft for melon of Onaga's algorithm that holds the search with its epochs: "The flow search by gecot::onaga, 2026-11-18". On the branch `multiflow-branchless`, three tests of the searches that go either way are written without a branch: "Branches, 2026-11-18". "What was measured" is the label-correcting search of commit da6b556. Nothing else here is implemented. The code is `include/gecot/solvers/multiflow_benders.hpp`; the algorithm is stated in `docs/formulations/benders.md`, "`multiflow_benders`: cuts per source".

## What a separation does

For a target t and a point x̂, `target_block::separate` builds one cut (the block is `multiflow_target_block`, in `include/gecot/solvers/multiflow_target_block.hpp` since 2026-11-17). For each source s of the block, `_largest_flow` computes the largest flow of s to t on its arcs A_s, where an improved copy a of the option i carries at most m_sa · x̂_i:

1. a search of labels from t gives ρ_u, the gain of the best path from u to t in the residual graph (`label`, Dijkstra's algorithm, on gains reduced by the labels of the search before from the second search of a source on);
2. the path of s takes what its arcs allow (a *push*);
3. the two steps repeat until the supply of s is sent or s no longer reaches t;
4. the labels of the last search are an optimal dual solution, and give the part of s in the cut.

Since 2026-11-17 step 1 is a search from s that stops at t, and the labels of step 4 come from one search from t at the end ("The flow search" below).

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

## The flow search, 2026-11-17

The owner asked for simpler and more efficient ways to compute `_largest_flow`, about half of the wall-clock time of a run, with two ideas: Dijkstra again with the filled arcs removed, nothing taken backward; and the flow of several sources at a time. Three sources of evidence: an instrumented build on four instances; prototypes behind a study switch, on the short suite at 30 s; and three agents that tried to refute each idea on small instances against an LP (Python, HiGHS and exact fractions; their scripts were not kept). The instrumented build and the prototypes are on the tag `archive/multiflow-flow-study-prototypes`, whose commit message says how to choose a variant.

### What the search of 2026-11-15 does, counted

Four runs at 30 s, at the fractional points of the rounds and at the scenarios of the candidates.

| | `species-natural` at 6000 | `periurban-mixed` at 6000 | `bike-squirrel-town` at 300 | `bike-town` at 500 |
|---|---|---|---|---|
| arcs of A_s, of which improved copies | 411, 48 | 549, 136 | 372, 128 | 465, 157 |
| searches per source, fractional points | 4.8 | 3.5 | 13.4 | 16.1 |
| searches per source, scenarios | 1.61 | 1.39 | 1.09 | 1.11 |
| vertices settled per search | 107 | 277 | 173 | 216 |
| of which before s | 73 % | 82 % | 81 % | 82 % |
| labels that change from a search to the next | 47 % | 36 % | 44 % | 42 % |
| pushes that take an arc backward | 2.5 % | 4.5 % | 14.0 % | 17.5 % |
| sources with such a push | 6.8 % | 11.4 % | 42.8 % | 54.2 % |
| labels of a first search that differ from one search over every open arc, fractional points | 0.5 % | 3.4 % | 0.8 % | 0.5 % |
| the same at scenarios | 11.8 % | 9.2 % | 3.8 % | 4.0 % |
| separation CPU, fractional points and scenarios | 93.7 s, 7.3 s | 118.6 s, 43.5 s | 42.0 s, 2.8 s | 67.0 s, 1.8 s |

- **"About three searches per source" understated the bike instances**: 13 to 16 there, 64 to 73 % of the sources at seven or more. Each push fills one copy, and their points hold many options at small values.
- **A scenario does not always take one search.** When the source has its whole quality and its path is its best one to a bought copy, the push equals m_sa, the copy counts as full, and a second search runs: 9 to 61 % of the sources. Its labels are not the probabilities of connection, which `benders.md` said. Both sets of labels are optimal; those of the second search put a coefficient on the option of the copy, and are the stronger cut in 52 to 62 % of the pairs an agent compared, the weaker in 1 %.
- **The time is at the fractional points**: 73 to 97 % of the separation CPU.

### 1. No arc taken backward: valid, neither exact nor tight

The idea: Dijkstra from t on the arcs that are not full, push, remove the copies the push filled, repeat; the labels of the last search give the cut.

- **Valid.** The labels of a search from t on any set of arcs that holds every arc without option are a feasible dual solution, with σ on the copies as `separate` computes it. So the cut holds in every scenario.
- **Not exact.** Smallest instance found: vertices s, a, b, t; arcs without option s→a 1/2, a→b 1, a→t 1/2, b→t 1/10, s→b 9/10; copies of one option on s→a and b→t, both of probability 1; a quality of 2 at s; x̂ = 1/2. The first push fills both copies: 5/4 reaches t, where the largest flow is 29/20. No order of pushes to saturation reaches it: the best one gives 13/9. On the four instances the flow is lower for 6 to 46 % of the sources, by 2 to 4 · 10⁻⁵ in total.
- **Not tight, and that is what costs.** Even where the flow is the largest, the labels of the arcs that are not full are not an optimal dual solution: their value at x̂ is above the flow for 68 to 87 % of the sources, by 2.5 to 10 % in total. In the search of today an arc that carries flow, taken backward, holds the label of its head; without it that label drops and σ grows on the full copies. The arcs taken backward do two jobs, pushing back and holding labels, and the second one is needed far more often than the first.
- **Not tight at a scenario either**, in the tie of the second bullet above (5 to 47 % of the sources): a cut that is not tight at a candidate may not cut its f_t off. The prototype therefore kept today's search at the scenarios.
- **As many searches**: 0.91 to 0.98 of today's.
- **Measured.** `species-natural` at 6000: a root bound 2.68 % above the optimum against 0.78 %, 544 nodes against 33, 12,210 lazy cuts against 726, 13.8 s against 12.5 s. `bike-town` at 500: a root bound 2.0 % above the optimum against 0.035 %, the 30 s limit with 7112 nodes against 477 nodes in 9.9 s.
- **Testing its own gap does not save it.** The flow sent F and the value D of its labels at x̂ are free to read, and D = F proves both optimal. But D > F for most sources whose flow is the largest, so "this search, then today's where D > F" runs more searches than today's alone (23.2 against 12.4 per source in an agent's bike-like regime).
- **What is kept of it**: the arcs are tried backward only at the vertices where a push of the source left or brought flow. On the search of 2026-11-15 that alone took 5 to 10 % off the separation CPU, with the same labels.

### 2. What is kept: the pushes by searches from the source

Dijkstra with arcs taken backward needs potentials p, with p_t = 1 and p_u ≥ g · p_v on every residual arc of gain g: the reduced gains g · p_v / p_u are then at most 1. The search of 2026-11-15 used as potentials the labels of its last search from t, which it ran in full after every push. Now:

1. **One search from t over every arc open at x̂**, once per target and separation, gives the first potentials of every source: the first residual graph of a source is among these arcs.
2. **Each push follows a search from s**, stopped when t is settled. With D_u the reduced gain of the best path from s to u, the potentials of the vertices settled, which have D_u ≥ D_t, become p_u · D_t / D_u. They stay feasible (the four cases of an arc between settled and other vertices), and are tight on the path, so the arcs that the push opens backward have a reduced gain of 1.
3. **One search from t at the end** gives ρ on the last residual graph.

- **The same cuts.** ρ is the gain of the best residual path to t once the flow is the largest. It is the least optimal dual solution: any optimal one is feasible on the residual graph of any largest flow, by complementary slackness, hence at least ρ everywhere. So ρ does not depend on which largest flow the pushes found, and is the ρ of the search before, up to rounding.
- **Why it is faster.** A search from t settled 73 to 82 % of its vertices before s. A search from s on exact potentials settles the vertices that a path at least as good as the new best one could cross if their potentials still held: 33, 76, 84 and 99 per search on the four instances, where a search from t settles 107, 277, 173 and 216. The gain is smaller on the bike instances: an improved copy there is worth much more than its arc, so the path that replaces a filled one is far worse, and many vertices pass the test.
- **The limit of pushes is simpler.** Past `max_pushes` the pushes stop and step 3 runs on the residual graph reached. Its ρ is feasible, the value returned is that of the cut at x̂, the flow sent plus ρ_s times the supply left, a bound on the flow; a scenario needs one push, so the flow of a candidate stays exact. The flows are no longer reset for a search without flow.
- **Checked.**
  - The unit tests: 77 of 77; `multiflow_cuts_are_valid_and_tight` now asks that the value of a cut be the cut at its point with the limit at 0 and at 1 too.
  - The oracle: 2715 of 2715 runs; 268 of 268 with `multiflow_target_block<0>` and 268 of 268 with `multiflow_target_block<1>`.
  - The parity capture against main: 7 of 1021 cases differ, all `multiflow_benders` on `aix30` at 8, by the hash of the model alone. In that model 33 of 8665 numbers differ, by 2.2 · 10⁻¹⁵ relative at most; scores, rounds, cuts, nodes and logs are the same in the 1021 cases.
  - The short suite at 30 s, main bc1690b then the branch, one run of each: the same 31 runs solved, with the same scores. They take 207.1 s on main, and 172.0 s then 174.6 s in two runs of the branch, before and after the remarks of its review, whose 31 solved runs have the same rounds, cuts and nodes; what precedes the search, which holds the rounds, takes 89.2 s on main, 55.3 s and 56.5 s on the branch.
  - 37 of the 42 runs have the same rounds, cuts and root value to the last bit. The 5 others stop their rounds on the clock and now do more of them: `bike-suburb` at 2000 goes from 10 rounds and a bound of 9,102,989 to 15 rounds and 9,063,286, `bike-squirrel-suburb` at 500 from 14 rounds to 19.
  - 26 of the 31 runs solved have the same lazy cuts and nodes. In the 5 others the master after the rounds is main's up to rounding, and the search then takes another path: on `species-natural` at 6000, 1669 of its 342,920 numbers differ, by 9.3 · 10⁻¹⁵ relative at most, for 21 nodes and 704 lazy cuts against 33 and 726; on `periurban-mixed` at 12000, 134 of 623,863 by 2.0 · 10⁻¹⁵, for 17 nodes and 2036 lazy cuts against 11 and 1698.
  - With the study build, which counts, on the same 31 runs: the separation CPU goes from 1060 s to 612 s at the fractional points and from 150 s to 128 s at the scenarios.
  - Two readers of the new function, neither building nor running gecot. The first proved the four cases of the potentials on the code, and ran a port of its own with every step asserted (potentials feasible, path tight, ρ equal to the labels of Bellman and Ford on the last residual graph and to those of the search before, the value at the limits 0, 1 and 2, the state left to the next source): 48,553 sources in exact arithmetic and 115,875 in doubles, no failure, and five mutations of the port all caught. The second, a port tested against HiGHS, was cut off by a usage limit after 60,697 sources in doubles: the flow of the search before within 2.2 · 10⁻¹⁶ of the supply, and its labels but for 68 sources with an entry of x̂ around 10⁻¹³ or a copy full within the tolerance ("Found on the way"). Their remarks are in: the vertices to restore are listed once, the search from s stops at t without scanning its arcs, two comments.
- **Not simpler.** `_largest_flow` is 173 lines where it was 125, with five more arrays in its scratch state. What is simpler is the limit of pushes.

### 3. Several sources at a time

Two readings.

**The same cuts, with shared work.**

- The search of step 1 above is shared by the sources of a target. Its labels serve as potentials, not as the dual solution: "Tried and dropped" records what the second use costs.
- *The last labels repaired from that search* (prototype): a vertex whose path in the shared search is in the last residual graph of the source keeps its shared label, the others are searched among themselves. Exact. At the scenarios it relabels 10 to 48 vertices per source where a search settles 91 to 216, and takes 36 % off their separation CPU on the suite; at the fractional points it relabels 53 to 85 % of them and gains nothing. On the suite: 169.4 s against 175.1 s for the 31 runs solved, for about 60 lines and five more arrays. Left out.
- *Labels repaired after each push*, in place of a search: exact with keys reduced by the labels before the push, wrong with the labels themselves. But 36 to 47 % of the labels change at each push, and each relabelled vertex has its arcs scanned twice: an agent counted 0.24 to 1.01 of the arcs of a full search. The searches from s reach the same vertices with less machinery.
- *Flows in proportion*: two sources with the same A_s and constants in a constant ratio do not have proportional flows (refuted on 295 of 1452 pairs). They do when a source has one arc of A_s leaving it, without option nor copy, to another source, and neither has a quality gain: its part of the cut is then a multiple of the other's. The benchmark has no such source: 0 on the eleven instances counted.

**One flow per group of sources.** This changes the relaxation; it is the aggregation the owner thought of for the memory of the sets A_s. With A_g the union of the A_s of a group and M_ga the sum of its m_sa on a copy:

- valid: the flows of the sources add up to a flow of the group; and exact at a scenario, where the group sends each quality on its own best path;
- the searches are divided by about the size k of the group, the stored arcs by up to k;
- the root gap grows. On an agent's grids, the root gap of groups of the k nearest sources over the gap per source, for k = 2, 4, 8, 16 and one group: 1.00 to 1.47, 1.00 to 2.57, 1.01 to 4.38, 1.02 to 6.60, 1.02 to 14.5. Almost nothing is lost where every arc has a copy and the budget is loose; groups drawn at random lose 1.5 to 3 times more.

Not prototyped in gecot: neither the nodes nor the time are measured, and the per-source relaxation is what makes `multiflow_benders` fast.

### Found on the way

- **The arc without option of a copy of A_s can be outside A_s**, since the test reads the probability of each arc: for 77 % of the copies of the sets on `species-natural` and `natural-crossing`, 1 to 7 % on the other landscapes, all of them on the cover instances. Putting it in does not change the largest flow at any point; it changes which optimal dual solution the labels are, towards cuts that charge a copy for its improvement. Prototype on the suite: the 31 runs in 169.2 s against 173.0 s, fewer cuts in the rounds (the separation CPU at fractional points goes from 612 s to 498 s), and nodes that move both ways (`species-natural` at 12000: 13 to 408; `rural-planting` at 260: 156 to 70). Not kept as it is.
- **Entries of x̂ under 10⁻¹²** make a push that fills its copy without opening it backward, since a flow under `no_flow` times the supply counts as none. On tie-prone instances with such entries an agent got cuts that depend on the order of the heap among ties, all valid. The center of in-out halves old values at each round, which could produce such entries; whether the real points hold some was not counted.
- **A candidate needs the labels of a block only if the master overvalues it**, and its flow is the quality of each source times its label in the shared search. The lazy cuts are 17 to 79 % of the separations at scenarios on the four instances: the flows of the others could be skipped. Not tried; the scenarios are 17 % of the separation CPU on the suite.

## Prefetching in the searches, 2026-11-18

The owner asked to try what melon's Dijkstra does: between `top()` and `pop()` of its heap, it prefetches the entries of the first arc that leaves the vertex it is about to settle (the target and the length), which gave him about 10 % elsewhere. He then asked for `status[u]`, the entry of the vertex popped, written right after `pop()`.

**Not kept: no variant is faster on this machine**, on the suite or on the largest instance at hand. Each row is one variant alone, its prefetches between `top()` and `pop()`; a figure is the time with the variant against the time without, and "that search" is the time of the search alone.

What was asked:

| Search | What is prefetched | Suite: separation CPU | Suite: that search | Hexagon: separation CPU |
|---|---|---|---|---|
| from s | every arc map that the loop reads, at the first out-arc: melon's pattern | +1.0 % | +2.8 % | – |
| last, from t | the ids of the in-arcs, and every arc map read, at the first: melon's pattern | +1.1 % | +2.6 % | – |
| both | the two rows above together | +2.0 % | +2.0 % and +2.4 % | +0.3 % and +0.6 %, on two series |
| of `_source_arcs` | the target and the probability of the first out-arc: melon's pattern | not in a separation | – | +0.3 %, the spread of the reference |
| the three | `status` of the vertex popped | +0.2 % | +0.2 % and −0.1 %, inside the spread | +0.1 %, inside the spread |

Variants around them, lighter or heavier, which were not asked:

| Search | What is prefetched | Suite: separation CPU | Suite: that search | Hexagon: separation CPU |
|---|---|---|---|---|
| from s | `in_A_s` of the first out-arc | +0.8 % | +1.0 % | – |
| last, from t | the ids of the in-arcs, and `in_A_s` of the first | +0.6 % | +1.3 % | – |
| last, from t | the ids, and `in_A_s` of each in-arc | +3.0 % | +7.9 % | – |
| both | every entry of the vertex popped: `status`, marks, potential, gain or label | +1.0 % | – | +0.6 % |
| both | `status` of each neighbour | +5.6 % | – | +1.4 % |
| both | `status` of each neighbour, its gain or label and its potential | +6.8 % | – | +2.1 % |
| of `_source_arcs` | `status` of each neighbour | not in a separation | – | +2.5 % |

How it was measured:

- **One study build for several variants**, each behind a bit of a mask read from the environment, so that the code is laid out the same for each of them, and the reference is the same binary without a mask. It counts the thread CPU time of `separate` and the time of each search. There were three such builds: the arc maps; the vertex entries and `_source_arcs`; `status` of the vertex popped alone, with melon's pattern on both searches again. A row is compared with the reference of its build. No build without the counters was made, since nothing won.
- **Suite:** the 31 runs of the short suite that main solves within 30 s, the variants interleaved, three passes on the first build and two on the others. Every pass did the same work: the same counts of sources (42.3 million), of searches and of settled vertices. A reference takes 717 to 722 s of separation CPU depending on the build, of which 394 s in the searches from s and 300 s in the last ones on the first; its passes are within 0.15 % of one another, and within 1 % for a search alone. The 31 runs take 171 to 174 s for the references and 171 to 176 s for the variants: a few percent of separation CPU hardly show there.
- **Hexagon:** `hexagon_instances/1_1900_3800` at 151 with `--no-stored-arcs`, 50 s, two runs per variant: one round and two candidates, 6.2 million sources. A reference takes 1113 to 1120 s of separation CPU, of which 598 s in `_source_arcs`, 451 s in the last searches and 8 s in the searches from s; its two runs are within 0.3 % of each other. The first series of the row "both" is on the first build, at 100 s, where the two runs of the reference are 0.6 % apart: +0.3 % there is no difference.

Why, as far as the sizes tell: there is little to fetch. The hardware counters of this machine cannot be read, so no miss was counted, and what follows is an inference.

- **A block is small.** Its searches read about 37 bytes per arc and 50 per vertex. The graphs G_t of an instance of the suite hold 200 to 4,600 arcs on average, 12,400 at most, and those of hexagon 11,800: 10 to 280 KB for a block of the suite, 700 KB at most, and 640 KB on hexagon. A core here has 1 MB of L2 (Ryzen 7 7800X3D: 32 KB of L1, 1 MB of L2 per core, 96 MB of L3), which two threads share in these runs. The blocks of the suite stay in it; those of hexagon do not, and its result rests on the points below.
- **A source loads its arcs before it searches:** it writes `in_A_s` and `flows` over A_s, two of the maps that its searches read there.
- **`status` is a byte per vertex,** 4 KB for the largest graphs.
- **The searches are short:** on average over a run of the suite, a search from s settles 3 to 98 vertices and the last search of a source 1 to 220; on hexagon, 1,500. The heap is small, and its `pop` leaves little time for a line to arrive.
- **The time per settled vertex does not grow with the graph,** which agrees with the above without proving it, the searches of these instances differing in length too: in the last search, at the points of the relaxation, 81 ns on `species-natural` (650 arcs), 50 ns on `urban-greening-large` (4,600) and 49 ns on hexagon (11,800), with 16 threads on 8 cores.

What it does not say:

- **Another machine.** With 256 or 512 KB of L2 per core, the larger blocks are read from L3, and a prefetch has something to do. The study build is on the tag `archive/multiflow-prefetch-study`, with the masks listed at the top of `multiflow_target_block.hpp`; it prints its times at exit on the lines `PREFETCH`. From the root of the repository, with that build, for the reference, melon's pattern on each search and on both, and `status` of the vertex popped:

  ```console
  $ misc/study/prefetch_passes.sh here build/gecot "1 2 3" "0 2 8 10 1024"
  $ python3 misc/study/prefetch_cmp.py here "1 2 3" "0 2 8 10 1024"
  ```

- **Graphs G_t far larger than these.**

If a variant is kept one day: melon's helper is `melon::detail::prefetch_keys_and_values`, in a `detail` namespace, and it does nothing, without a word, for a map that has no `data()`. The study build asserts `melon::contiguous_mapping` on each map it passes.

## Branches, 2026-11-18

The owner asked whether some branches can be replaced by branchless instructions, with an example: `visited` and `changed` as arrays of `num_vertices + 1` behind a `unique_ptr`, with an end pointer, a range being filtered by `*end = v; end += accept(v);`.

**Kept, on the branch `multiflow-branchless`: three of the five changes tried, the same results bit for bit, 3.3 % less separation CPU on the suite, 1.6 % less time, and about 20 % less memory at the peak with the sets kept.** A relaxation still ends in a branch that goes either way, and that one is the algorithm's.

| Change | Whose | In the build of the variants, on the suite: separation CPU | On hexagon | Kept |
|---|---|---|---|---|
| The lists of vertices (`reached`, `visited`, `changed`, and `visited` of `_source_arcs`) are arrays with an end pointer; `changed` is filled by a store and a conditional increment | asked | −1.15 %, of which −0.57 % for the arrays without the conditional increment | −1.7 % | no: about 0.3 % of the time of a run, for raw arrays whose bounds only the sanitizer checks |
| `_source_arcs` fills A_s and its improved copies the same way, in scratch arrays of `num_arcs + 1`, and the source gets them at their size | the owner's idiom, where a range is filtered | −1.1 % when added to the other changes, outside the searches; `compute_sources` −20 %; about 20 % less memory at the peak | −3.8 %, `_source_arcs` −6.1 % | yes |
| No `in_A_s`: the capacities and the flows are 0 outside A_s, and the capacity of an arc without option is the largest double, not infinity. An arc is open, forward or backward, by one comparison | mine | −2.8 %; the finite capacity alone −0.56 % | −0.6 % | yes: one map less, and no case for infinity |
| The potentials of the vertices a search went to are multiplied by 1 where the vertex is not settled, without a branch | mine | −0.74 % | – | no: a form that needs a comment to be read |
| `separate` adds a term of 0 for an improved copy whose σ is not positive, without a branch | mine | −0.63 % when added to the change without `in_A_s` | – | yes |

The owner ruled on that recommendation of mine ("Ok with your recommendation"). The figures of the table do not add: they sum to 6.4 %, and the first four changes together take 4.65 % less in the build of the variants. A row is its variant against the reference of that build, where a difference resolves to 0.1 %: two instantiations of the same code of the searches agree to 0.03 % in each search. Only the five together, then the three kept, were confirmed in builds of their own, below.

The three kept, as built against main, both with the timers of the study:

- **Suite**, the 31 runs that main solves within 30 s, three interleaved passes of each build, the same work in every pass: 710.1 s of separation CPU for main, 686.9 s for the change (−3.3 %), passes within 0.04 % of one another. The searches from s take 3.5 % less, `compute_sources` 20 % less (22.8 s to 18.2 s), and the last searches the same (+0.1 %). The 31 runs take 172.5 s and 169.7 s (−1.6 %). The five changes together had given −4.0 % of separation CPU and −1.3 % of time (−1.45 % without the timers).
- **Hexagon** at 151 with `--no-stored-arcs`, 50 s, two runs of each build: per source at the candidates, 174.7 µs of separation CPU for main and 169.0 µs for the change (−3.3 %); `_source_arcs` goes from 95.2 µs to 89.9 µs per source (−5.6 %). The totals do not compare: the first round of the change ends before half the limit, and a second one runs, so the points of the relaxation are not the same either.
- **Memory**, with the sets kept, one run of each binary: the peak goes from 705 MB to 559 MB on `species-natural` at 6000. With the five changes, from 1235 MB to 1009 MB on `urban-greening-large` at 8000 and from 906 MB to 747 MB on `periurban-mixed` at 12000; the memory is that of the sets, which the three kept hold. A set copied from the scratch arrays is a vector of its exact size, where one grown by `push_back` keeps its slack. With `--no-stored-arcs`, the same on hexagon (1230 MB).
- **Checks:** the parity capture is the same as main's in its 1021 cases, models included; the 77 unit tests pass, and pass again under the address sanitizer, as do six runs of `multiflow_benders`, three of them with `--no-stored-arcs`. A scratch array holds one more than its list can: an arc enters A_s once, and the store of an arc that is not kept needs the slot after the last. The forms without a branch were read in the code of the binary. The oracle and the builds that force the limit of pushes were not run again: the results are those of main bit for bit, and the unit test of the limit at 0 and 1 passes, under the sanitizer too.

### Where the branches go either way

A counting build recorded, for each test of the three searches, how often it runs, how often it holds, and how often a 2-bit counter of its own would have guessed it wrong: an estimate of what a predictor misses, on the high side, since a real one also reads the history. On the 31 runs, for 10.9 billion settled vertices:

| Test | Runs, billions | Holds | Misses of a 2-bit counter, billions |
|---|---|---|---|
| the vertex of a relaxation is settled | 28.0 | 36 % | 10.8 |
| the arc has no option (`capacities[a] == infinity`) | 29.1 | 74 % | 7.6 |
| the candidate is not better | 17.9 | 22 % | 4.5 |
| an arc scanned backward carries flow | 10.9 | 34 % | 4.2 |
| the arc is in A_s (`in_A_s[a]`), last search | 18.0 | 81 % | 3.0 |
| the vertex is in the heap: promote or push | 13.9 | 15 % | 2.3 |
| a copy has room | 7.5 | 37 % | 1.6 |
| the vertex has no arc to scan backward | 10.8 | 69 % | 1.3 |
| a vertex of `visited` is settled, in the update of the potentials | 6.3 | 83 % | 1.2 |
| the arc is in A_s, search from s | 14.7 | 98 % | 0.3 |
| the vertex is not yet in `changed` | 5.2 | 28 % | 0.26 |

That is 37 billion estimated misses, 3.4 per settled vertex. The five changes tried remove the tests of the second and fifth rows and of the last three, the three kept those of the second and fifth rows; the test of a copy's room then runs on every arc of A_s, where it holds for the arcs without option. In `_source_arcs` the test of an arc of A_s is predictable (4 % of misses on the suite, 2.6 % on hexagon, where it holds for 64 % of the arcs: the arcs kept come in runs), and the test of an option on a kept arc is not (24 to 28 %).

### Tried and not kept

- **One branch for "settled or not better"**, the two tests that miss the most: +0.2 % of separation CPU on the suite, −0.15 % on hexagon (the last search −2.6 %, `_source_arcs` +2.1 %). Merged, the test still goes either way (it skips 54 % of the relaxations from s), and it reads the value of a vertex that is settled. A relaxation ends in work on the heap or in nothing: this branch stays.
- **The owner's idiom on the arcs of a vertex**: the open arcs filtered into an array, then relaxed. On top of the change without `in_A_s`, +0.3 % of separation CPU: the loop over what was kept ends where the branch was.

### What the measurements say besides

- **A third of the separation is in the heap of melon.** A sampler under `gdb`, on three runs of the change (`periurban-mixed` at 12000, `bike-town` at 500, `urban-greening-large` at 8000): of the threads inside a separation, 59 to 63 % are in the code of `_largest_flow`, 33 to 36 % in the heap, `pop` alone 23 to 24 %. The branches of the heap were not counted.
- **The builds differ by more than most variants.** The reference of the build that holds the variants takes 732 s of separation CPU, the three study builds of the prefetches 717 to 722 s, and main with the timers alone 709 s. So a variant was timed against the reference of its own build, and the change against main in two builds made for it.
- **The compiler undoes some forms.** GCC 15.1 at `-O3` compiles `(a == 2) | (x <= y)` to the two jumps of `||`, and `x <= std::max(y, table[a])` to two jumps as well; the sum of the two tests gives one jump. `factors[test]` and `m * std::max(σ, 0.0)` come out without a jump. Each variant was read in the code generated (`objdump --disassemble=<symbol>`) before it was timed: the first form of "settled or not better" was the code of the reference.

How it was measured: the counting build once on the suite and on hexagon; a build that holds the variants as instantiations of the searches, chosen per source by a mask read from the environment (`GECOT_BRANCHLESS`), two interleaved passes per variant on the suite, where the passes of the reference are within 0.1 % of one another, and two runs on hexagon; then the two builds above. The headers and the scripts are on the tag `archive/multiflow-branchless-study`.

## The heap, 2026-11-18 (stopped)

Tried after the branches, as their first lead: other heaps for the three searches of a block, each an instantiation of the searches chosen per source in one study build. The owner stopped the study before a variant was settled on; nothing of it is on the branch.

| Heap | Suite: separation CPU, the 31 runs, two passes | Hexagon: per source at the candidates, two runs |
|---|---|---|
| melon's 4-ary heap (main) | 697 to 700 s | 170.5 to 170.8 µs |
| melon's 2-ary heap | −1.7 % | +1.3 % |
| melon's 8-ary heap | +0.9 % | – |
| an array scanned for its largest key, entries as pairs | −10.2 % | +34 % |
| the same, keys in an array of their own | −11.0 % (−12.1 % in a second build) | +34 % |
| the scan while it holds at most K entries, then melon's 4-ary heap until it is empty, `push` not inlined | K = 16, 32, 64: −0.7, −5.2, −6.1 % (one pass) | +6, +5.7, +10 % |
| the same, arrays of K, the small case inlined and the move out of line | not measured: its pass was stopped after 12 runs; on `bike-town` at 250 the same time as the scan | K = 16, 32, 64: +5.1, +4.1, +7.5 % |

- **The scan wins where the heaps are small.** At a pop of the last search, a heap of the suite holds 9 to 30 entries on average, and the scan takes 15 to 20 % less where it holds 12 to 18, 1 to 7 % on `urban-greening-large` (about 20). On hexagon it holds 70 on average, 310 at most, and the scan loses a third.
- **The hybrid first lost its gain to code, not to the switch**: GCC did not inline its `push`, which carried the move to the heap. With the small case inlined it matches the scan on `bike-town`; hexagon still pays 4 to 7.5 %, in the scan of the first entries of each search and in a call more for the large case.
- **The heaps change only the order of ties.** On the 31 runs, the scores and the root rounds, cuts and values are those of main bit for bit for every heap, the vertices settled within 0.2 %; two runs change their tree (`species-natural` at 6000: 704 lazy cuts and 21 nodes, 1233 and 35 with the scan; `urban-greening-large` at 4000: 9 nodes, 5). A heap other than melon's 4-ary would need the oracle, not the parity capture.

The study headers are on the tag `archive/multiflow-heap-study`, as for the branches.

## A min-cost flow by melon's network simplex, 2026-11-18

The owner asked whether the separation of a source can be solved by melon's network simplex, with cost −log π and the capacity of a copy x̂ times the quality of the source (read here as w̄_s · x̂_i, in units of the quality of s). **No: it solves another program, and on the benchmark it is slower than the flow search and gives weaker cuts.**

- **Another program.** The network simplex solves a min-cost flow: flow is conserved on every arc. The program of the docs has gains: what enters an arc arrives multiplied by π. With cost −log π, a cheapest path is a most reliable one, so a scenario, which sends everything along one path, is the same in both. At a fractional point the flow splits, and the min-cost flow minimizes Σ_P f_P · (−log g(P)), where the flow that reaches t is Σ_P f_P · g(P). The capacity w̄_s · x̂_i in units of s equals m_sa · x̂_i at u only for flow that reaches u along its best path.
- **Its potentials still give valid cuts.** With ρ_u = exp(π_u − π_t), from melon's reduced cost c(a) + π(u) − π(v) ≥ 0 on the plain arcs, which no capacity bounds, ρ_u ≥ π_a · ρ_v holds, so the cut of the docs is valid. On 900 random instances of 5 to 9 vertices and 2 to 6 copies, solved exactly by HiGHS (`check.py` of the archive tag):
  - the potentials were always feasible;
  - on the instances where a copy binds, the cut at x̂ was the optimal one on 96 to 99 %, and up to 8 % above it on the rest;
  - at a scenario the cut was not tight 99 times in 10,456 checks, and never in 7,696 with a slack of 10⁻⁶ on the capacities: my reading, not checked, is that a copy then carries exactly the supply.
- **The feasible set of the proposal is a tighter relaxation, but a path program.** Flow conserved in units of s, with copies of capacity w̄_s · x̂_i, is contained in the program of the docs and exact at scenarios: its optimum is up to 12 % lower at fractional points on those instances. Maximizing the flow that reaches t over it is a linear program over paths, not a network flow, and the min-cost flow does not compute it (its flow reaches t with up to 10 % less). Not pursued: a change of model.
- **melon's solver needs integral data**, as its header says. With doubles, `finalize()` declares the flow infeasible unless every artificial arc ends with exactly no flow: 98 % of the solves of `bike-town` ended that way, and runs on `species-natural` and `urban-greening-large` did not end within 120 s. The prototype therefore counts flows in units of 2⁻⁴⁰ of the supply and costs −log π in units of 2⁻⁴⁰, rounded down, which keeps the potentials feasible for the true costs.
- **Measured.** The prototype uses the network simplex at fractional points only and keeps the flow search at scenarios, so the cuts of the candidates are the same. It builds for each source a graph of A_s with a bypass arc s → t of cost 60. One run each, 60 s:

| Run | Search, per source at fractional points | Network simplex, per call | Separation CPU at fractional points | Root rounds | Root value | Nodes |
|---|---|---|---|---|---|---|
| `bike-town` at 250 | 25.7 µs | 37 µs | 2.3 s → 3.5 s | 15 → 16 | +1.2 · 10⁻⁷ relative | 98 → 360 |
| `species-natural` at 6000 | 22.7 µs | 55 µs | 49 s → 217 s | 20 → 36 | +0.05 % | 21 → 64 |
| `rural-planting` at 260 | 4.9 µs | 20.5 µs | 6.6 s → 21.8 s | 16 → 16 | +1.9 · 10⁻⁵ relative | 156 → 116 |
| `urban-greening-large` at 8000 | 29.6 µs | 209 µs | 78 s → 438 s | 18 → 16 | +3.0 · 10⁻⁵ relative | 28 → 61 |

  The scores are those of main; the reference is the same binary without the network simplex. A solve on a random graph of the size of A_s had taken 2.5 µs at 170 arcs (`bench.cpp`), where `bike-town` takes 37: the sets A_s need many more pivots, though the pivots were not counted. On the real instances the cuts are weaker than on the random ones; most likely because the potentials of a basis are not the least optimal dual that the flow search returns, and the capacities are rounded.

The prototype, the exact check and the benchmark are on the tag `archive/multiflow-network-simplex-study`.

## Status epochs, 2026-11-18

The owner asked to try a counter in place of the resets of the statuses: a vertex is before the heap when its status is under a threshold T, in the heap at T and settled at T + 1, and T moves by 2 between runs.

**Kept, on the branch `multiflow-status-counter`: the same results bit for bit, 2.8 to 2.9 % less separation CPU on the suite, 0.8 to 1.1 % less time, neutral on hexagon.** Since the branch `multiflow-onaga`, the epochs are those of `gecot::onaga`, and the code described here is gone from the block.

- **What changed.** `status` is an `unsigned char`, and each search of `_largest_flow` takes the next epoch, T + 2: the searches from s, several per source, and the last one from t. Once T would pass 254 every status is set to 0 again, once in 127 searches. `gain_from_s` and `rho` are no longer reset: a relaxation reads them only for a vertex in the heap, and the labels are read through `label(v)`, which gives ρ_v for a vertex the last search from t reached and 0 for the others. The list `reached` is gone, and `visited` became the list of the settled vertices, which a search needs to rescale their potentials. The restore of `potentials` and `marks` over `changed`, and the loop over A_s that sets and clears the capacities and flows, stay as they were: they restore values that are not constants, or set values that each source needs anyway.
- **Not applied to `_source_arcs`**, whose fill loop reads Π̄(s, u) at the tail of every arc of G_t, so it would read a status per arc as well. Untested.

As built against main, both with the timers of the study, the 31 runs that main solves within 30 s, the same work in every pass:

| Series | Separation CPU, main | With the epochs | Searches from s | Last searches | Time of the runs |
|---|---|---|---|---|---|
| three passes of each | 686.4 s | 666.9 s (−2.84 %) | −5.2 % | +1.35 % | −0.8 % |
| two passes of each, with the variant below | 686.0 s | 666.2 s (−2.88 %) | −5.7 % | +1.0 % | −1.1 % |

- **The last searches take longer, the whole separation less.** The timer of a search from s covers the reset of `visited` that it no longer does. That of the last search starts after the reset of `reached`, which `_largest_flow` did first: the saving shows in the separation, and the extra 1 % is most likely the relaxation with its tests in another order.
- **Hexagon** at 151 with `--no-stored-arcs`, 50 s, two runs of each: 168.7 µs per source at the candidates with the epochs, 169.0 µs for main (−0.15 %; the two runs of each agree to 0.04 %). The searches from s are 1 % of the separation there, and the last searches take 1.6 % longer.
- **A variant that keeps the order of main's tests**, `if(gain <= (queued ? value : 0.0)) return;` after the test of a settled vertex: −2.44 % of separation CPU, the last searches +1.8 %. Not kept.
- **Checks:** the parity capture is the same as main's in its 1021 cases, models included, and the 77 unit tests pass.

## The flow search by gecot::onaga, 2026-11-18

The separation of a source is a generalized maximum flow in a lossy network, and its search is Onaga's algorithm (`docs/formulations/benders.md`, "Separation"). The owner agreed it belongs in melon, under the name `onaga`, and asked for a draft in melon's style, then to use it in the block and compare the timings.

- **The draft**, `include/gecot/algorithm/onaga.hpp`: `gecot::onaga<Graph, GainMap, CapacityMap, Traits>`, the search of the block made generic, with the status epochs, a sparse `reset()` (the labels and marks of the vertices the searches settled, the flows of the arcs that carried flow), the labels to start from set once by `set_start_labels()`, `advance()` for one augmentation, and melon's lifecycle: the setters keep the flow, `reset()` and the setters come in either order, `run()` drains and is idempotent. Its tests (`test/onaga_test.cpp`) check optimality certificates on random lossy networks (a feasible flow and a feasible dual of equal value), that a reused object gives a fresh one's results bit for bit, with and without start labels, and that a run cut short bounds the maximum. Three mutations of the header each fail them.
- **In the block**, `flow_search` keeps the capacities, 0 outside A_s, and the algorithm that reads them; `_largest_flow` sets the capacities of A_s, runs at most `max_pushes` augmentations, computes the potentials and clears the capacities. The block goes from 511 lines to 341 (213 removed, 43 added); `onaga.hpp` has 526 and its tests 260. The tolerances `arc_full` and the flow taken for none are those of `onaga_default_traits`, the values of the block.
- **The same results bit for bit**: the parity capture is the same as main's in its 1021 cases, models included, and the 82 unit tests pass.

As built against main and against the epochs in the block, all with the timers of the study, two interleaved passes of each over the 31 runs that main solves within 30 s, the same work in every pass:

| Build | Separation CPU | Searches from s | Last searches | Time of the runs |
|---|---|---|---|---|
| main | 687.2 s | 386.2 s | 302.1 s | 173.0 s |
| the epochs in the block | 666.4 s (−3.0 %) | −5.2 % | +1.4 % | −0.6 % |
| `gecot::onaga` | 650.3 s (−5.4 %) | −9.6 % | −0.3 % | −1.3 % |

- **Hexagon** at 151 with `--no-stored-arcs`, 50 s, two runs of each: 166.3 µs of separation CPU per source at the candidates with `onaga`, 168.8 µs for main (−1.5 %), and 168.9 against 171.2 µs at the points of the relaxation (−1.3 %).
- **The vertices to restore in a flat array** of the number of vertices plus one, appended by `*end = u; end += !listed;` without a branch, as the owner asked: the same results, −0.09 % of separation CPU over three interleaved passes of each build (649.8 s to 649.2 s, the passes within 0.04 %), the run time +0.3 %, hexagon −0.1 % per source. The test it removes was predicted: 0.26 billion of the 37 billion estimated misses ("Branches, 2026-11-18"). Not kept; the commit is on the tag `archive/onaga-changed-array`.
- **The flows reset by a fill of the whole map** (`melon::detail::fill`) in place of the list of the arcs that carried flow, as the owner asked: the same results, +1.29 % of separation CPU over three interleaved passes of each build (650.0 s to 658.4 s, the passes within 0.1 %), the run time +0.16 %, hexagon +1.1 % per source at the candidates. The fill costs the arcs of G_t at each source, where the searches touch a few dozen: `rural-planting` +4 to +5.5 %, `urban-greening-large` +1.8 to +3.8 %, the bike and species instances, whose searches are long, ±0.5 %. Not kept; the commit is on the tag `archive/onaga-flow-fill`.
- **The flows cleared over the out-arcs of the vertices the searches settled**, in the loop that restores their labels, in place of the list of the arcs that carried flow, as the owner asked. It is enough: an arc gets flow only on an augmenting path, whose vertices a search settled. The same results, and the cost stays that of the vertices touched; but +0.66 % of separation CPU over three interleaved passes (650.7 s to 655.0 s, the passes within 0.06 %), 0.0 to 1.4 % on each run, the most where G_t has the most arcs per vertex, the run time −0.05 %, hexagon +0.2 % per source. Not kept, performance first; the commit is on the tag `archive/onaga-flow-outarcs`.
- **The lists emptied by `resize(0)` in place of `clear()`**, as the owner asked: with GCC 15 both are a load, a compare and a conditional jump, in the other direction, and the code differs only in its registers and layout. −0.13 % of separation CPU over three interleaved passes (648.9 s to 648.1 s, the passes within 0.06 %), the run time +0.01 %, hexagon −0.2 % per source at the candidates. The last search, which uses none of these lists, moved by −0.5 %: shifts of that size are layout. Not kept; the commit is on the tag `archive/onaga-resize-0`.
- **Why `onaga` is faster than the same epochs in the block** was not isolated. It also walks the path without reversing it, reads the flow taken for none once per search, clears the flows of the arcs that carried flow rather than those of A_s, and keeps its state in members where the block's lambdas captured references.

## Simplifications of gecot::onaga, 2026-11-19

The owner asked which parts of `onaga.hpp` could be simplified for speed while staying generic: useless allocations, cache misses, mispredicted branches, instruction count. A review by four lenses (allocations and layout, branches, generated code, arithmetic), with each finding then checked by a skeptic who compiled it, gave 29 candidates; 18 of them were timed, alone and stacked. **The owner then took my recommendation for an algorithm meant for melon: the tree reuse, the flow chains and the open test in one comparison, applied and measured at the end of this section.**

- **The simplifications proper gain nothing measurable:** one vertex map for the gains from s and the potentials (never live at once), the predecessor without its direction bool, the start labels kept in `_changed` in place of their map. Allocations are not a cost here: building the object once per `separate()` is 0.2 to 0.3 % of the separation, mostly in the block.
- **What gains adds state**, in the scans backward and in the heap. Bit for bit, the single arc that took flow kept per vertex and local copies of the views: 6.6 % less separation CPU and 2.1 % less time on the 31 runs. With the dead status and five changes that measure as nothing alone, 7.9 to 8.2 % and 2.1 to 2.6 %. **With the previous tree reused by the next search, which changes the order of ties, 17.5 % and 5.6 to 6.1 %.**
- **On hexagon** at 151 with `--no-stored-arcs`, where the searches from s are 1 % of the separation, 1.2 to 1.4 % less per source at the candidates for the stacks timed there.

How it was measured: each candidate is a bit of a template parameter of a study copy of the header; the block instantiates the variants and picks one per run by a mask read from the environment, and the reference is mask 0 of the same binary, whose parity capture is that of the branch in its 1021 cases. Four such builds, two interleaved passes of each mask over the 31 runs that main solves within 30 s. A figure is separation CPU against mask 0: in the first two builds, over the 27 runs whose separations, sources, scores and nodes agree for every mask of the build; in the last two, and for the stacks, over the 31. The per-vertex record is the verifier's header, in the fourth build, against its control, the branch's header with the same hoists. The two passes of a mask agree within 0.1 %, and a control mask, the code of mask 0 under another number, lands within 0.2 %; but the code generated for an instantiation moves a figure by up to about 1 %, as the hoists show (+0.9 % for fewer loads), so **a figure under 1 % is no evidence either way.**

| Candidate | Change | Results | Separation CPU |
|---|---|---|---|
| tree reuse | a search from s after an augmentation first settles, without the heap, the previous tree minus what the augmentation closed: after the rescale those vertices are at the relabeled gain 1, the largest there is | ties | −10.1 % on the 31 runs, −10.5 % on the 27 of the same trees (searches from s −20 %) |
| flow chains | per vertex, a chain of the arcs that took flow, in place of the scans of every in-arc (search from s) and out-arc (last search) with the flow test | ties | −4.9 % |
| single flowing arc | per vertex, the first arc that took flow into it and the first out of it, with a mark bit for a second one: the scan is one arc unless two took flow | bit for bit | −4.3 % |
| local views | the graph and map views copied into locals of the searches when they are small and trivially copyable (`graph_ref_view`, `mapping_ref_view`), else used in place: the out-of-line calls of the heap make the loops reload a member view's pointer at every iteration. A plain copy through `graph_all`/`mapping_all` would deep-copy an owning map view at each search | bit for bit | −2.3 % |
| raw gains | the gain of the path from s stored in place of the relabeled gain, key gain × label: no division per settled vertex, rescale label(u) = gain(t) / gain(u) | ties | −1.8 % (searches from s −6.7 %, last searches +3.8 %) |
| per-vertex record | one vertex map of a struct (label, gain or potential, predecessor, status, mark) in place of five maps, the verifier's prototype against its own control | bit for bit | −2.4 % against its control, which has the hoists; hexagon +0.2 % |
| no zero test, dead status, 16-bit statuses, lazy candidate, predecessor without its bool, open test in one comparison, one map for gains and potentials | see below | bit for bit | −1.1, −0.2, −0.7, −0.6, −0.3, −0.3, 0.0 % |
| start labels in `_changed`, backward scans compacted into an array, hoists | see below | bit for bit | +0.2, +0.2, +0.9 % |
| raw gains and inverse labels | the last search multiplies by 1/label | ties | −0.5 %, worse than raw gains alone |

The small ones: `gain <= 0` dropped in the last search, where only an underflow makes it true (alone, an underflow at a vertex of label 0 pushes the key 0/0); the vertices of label 0 given a status above every epoch, so that one compare decides settled, dead or `gain <= 0` in both searches, which also drops the test of the previous; statuses and marks of `uint16_t`, against the aliasing of `char` stores, though the review found the code of the loops unchanged by them; the gain of a relaxation computed past the test of a settled vertex; the direction of a predecessor found as `arc_target(a) == v`; `flow < (1 − arc_full) × capacity` as the open test, which makes an infinite capacity open its arc where the header says it closes it; the label at first listing logged in `_changed`; the arcs with flow filtered into an array before they are relaxed; the gain of t, the potential of v and the gain of u held in locals.

| Stack | Separation CPU | Time of the runs | Hexagon, per source |
|---|---|---|---|
| single flowing arc and local views | −6.6 % | −2.1 % | – |
| the same and the dead status | −7.4 % | −2.5 % | – |
| single flowing arc, local views and the tree reuse | −14.5 % | −5.3 % | – |
| the same and the dead status | −15.2 % | −5.6 % | −1.4 % |
| single flowing arc, local views, dead status, 16-bit statuses, lazy candidate, predecessor, one-compare open test, one map (the no zero test is inert next to the dead status) | −8.2 %, −7.9 % in the fourth build | −2.1 %, −2.6 % | −1.3 % |
| the same with flow chains for the single flowing arc | −8.5 % | −2.3 % | – |
| the eight bit for bit above and the tree reuse | −17.6 %, −17.5 % | −5.6 %, −6.1 % | −1.3 % |
| raw gains, flow chains, local views, no zero test, 16-bit statuses, predecessor, one map, lazy candidate, dead status, one-compare open test, tree reuse | −16.0 % | −4.9 % | −1.2 % |

- **The tree reuse** keeps Dijkstra's order: after the rescale every arc of the previous tree between settled vertices has relabeled gain 1, and the augmentation closes the bottleneck only, so the next search would pop the surviving tree first, at the largest key. It saves a push and a pop per reused vertex, and the heap is a third of the separation. On random networks with tied gains, converged runs give the flow value to 4·10⁻¹⁶ and the same potentials; runs cut short can take another of the best paths. On the suite, the same scores and root rounds, 3 fewer root cuts of 82,039 and 87 fewer lazy cuts of 36,584, and other trees on three runs (`periurban-mixed` at 2500: 52 nodes to 37; at 12000: 17 to 13; `species-natural` at 6000: 21 to 24). Raw gains lose on top of it: −8.2 % together on the 31 runs, −10.1 % alone.
- **Raw gains** change the trees of three runs of the suite (`periurban-mixed` at 12000: 17 nodes to 16; `urban-greening-large` at 4000: 9 to 5; `species-natural` at 6000: 21 to 17); the flow chains change none of the 31.

**A lead, not a drop-in: the labels as the dual, without the last search.** After the searches, the labels, with those of a failed search's reach set to 0, satisfy complementary slackness with the flow: an optimal dual solution (supply sent, or no path left) or a feasible one bounding the flow (run cut short), as the potentials are. The certificate tests of `onaga_test.cpp` pass on it; the one that pins the potentials independent of the start labels fails, as it should. It removes the last search, nearly half of the separation: on the 31 runs, 654 s of separation CPU with mask 0, 602 s with the bit-for-bit and raw-gain changes of the first build, 244 s with the labels on top of these. But it is another optimal dual, larger than the potentials away from the paths, and its cuts are weaker on the cover instances: `cover-40` at 12 and `faircover-40` at 12 go from 9.4 s and 3.7 s to the limit, and the 31 runs take 185 to 198 s against 165 s, 29 of them solved. On the landscapes it often halves the time (`urban-greening-large`, `natural-crossing`). Untested: the labels at the fractional points only, or the potentials only where the labels' cut is not violated enough.

Checked and left out by the skeptics: an early stop of the last search once the vertices the block reads are settled (needs a contract for the reach of a failed search; 2 to 5 % on random graphs, uncounted on the benchmark); searches from t (twice the settles); the bottleneck of `_augment` fused with the path walk (needs raw gains, about 0.2 %); the root of a search settled without the heap, `promote` given copies, a 32-bit heap index (the heap's), the construction per `separate()`, a pruning of pushes below the key of t: all estimated under 0.2 %. The gains from s read from the heap in place of their map: sound, within 0.3 %, not with raw gains; not timed.

Checks: each variant passes the five tests of `onaga_test.cpp`, the labels as the dual all but the one on start labels. On 12,000 runs of random lossy networks, with start labels, reused objects and runs cut short, the bit-for-bit candidates and their stack give the flows and potentials of mask 0 bit for bit, with continuous and with tied gains. The parity capture of the bit-for-bit stack is that of mask 0 in its 1021 cases, models included; the two stacks with the tree reuse differ in 7 cases, the models of `multiflow_benders` on `aix30` at 8, whose outputs and scores are the same. The oracle on the four configurations of `multiflow_benders`, which a mask unknown to the binary fails: 268 of 268 for the two stacks with the tree reuse and for the labels as the dual.

The study headers, the blocks that dispatch the variants and the scripts are on the tag `archive/onaga-simplify-study`.

**Applied.** For a header meant for melon I recommended what improves the algorithm on any graph: the tree reuse, and the flow chains rather than the single flowing arc, which is tuned to these statistics; and the open test in one comparison for its contract, infinity becoming the unbounded capacity as in melon's maximum flows. I advised against the local views, a workaround of the code GCC generates around the heap's calls, which would belong in a helper of melon if anywhere; against the per-vertex record, which goes against melon's map per role; and against the rest, under 1 %. The owner agreed ("Ok for your recommendations").

- **The header:** the three changes; the marks `entered` and `left` give way to the heads of the chains, and `listed` becomes a `bool` map. The block's unbounded capacity is infinity. The tests found a defect of the earlier header too, on new random instances: an arc taken backward as the bottleneck could keep −1 ulp of flow, which the certificate rejects; the flow is now clamped at 0 there. `misc/design/onaga_for_melon.md` is the note for the maintainer of melon.
- **Timed against the branch**, two builds with the timers of the study, three interleaved passes over the 31 runs: 648.0 s of separation CPU to 555.0 s (−14.3 %), the searches from s −27 % and the last searches −2 % on the 28 runs of the same trees, and 165.4 s to 157.1 s for the runs (−5.0 %). The same scores, and other trees on the three runs of the study (`periurban-mixed` at 2500 and 12000, `species-natural` at 6000). Hexagon at 151 with `--no-stored-arcs`, two runs of 50 s each: +0.3 % per source at the candidates, 167.1 µs against 166.6 µs: no gain where the searches from s are 1 % of the separation.
- **Moved into melon** 1.1.0, as `melon::onaga`, with the sparse reset and the flow lists as switches of its traits, on by default. The block uses it, and gecot's header and its tests are removed (the tests are melon's now). The parity capture is that of the binary built with gecot's header in its 1021 cases, models included, and the oracle passes 268 of 268.
- **Checks:** 86 unit tests, of which four new ones for the onaga header (tied gains, potentials between augmentations, infinity against the largest capacity, the state a run leaves for the next search). Each new mechanism broken on purpose fails them: the residual test of the kept tree (the run hangs), the chains or the tree left over by `reset()` (a crash; a failure), the tree left over by `set_source()`, the gain of a kept vertex, either link of a chain, the open test in two terms, the clamp. The tests of the header pass under the address and undefined-behaviour sanitizers. The parity capture differs from the branch's in the 7 models of `multiflow_benders` on `aix30` at 8, as in the study, with the same outputs and scores. The oracle on the four configurations of `multiflow_benders`: 268 of 268, and 268 of 268 with `max_pushes` forced to 0 and to 1.

## Tried and dropped

- **One Dijkstra per target, shared by the sources.** The labels of a search from t over every arc open at x̂ are a feasible dual solution for each source, and an optimal one for a source whose best path fills no arc. Those sources then need no search of their own. Valid, and the oracle passes, but: no faster on the suite (226 s against 220 s on the 31 runs), because at the points of the relaxation most sources fill an arc and search anyway; and the cuts are weaker, the shared labels being larger than a source's own away from its path (`species-natural` at 12000: 388 nodes against 14, 8858 lazy cuts against 770).
- **`bool` flags in place of `char` ones** in `flow_search`, against the aliasing of `char` writes: no measurable change.
- **One branch for "settled or not better" in a relaxation, and the open arcs of a vertex filtered before they are relaxed:** no gain. "Branches, 2026-11-18".
- **A min-cost flow by melon's network simplex, cost −log π:** another program; valid cuts, but 1.4 to 7 times the CPU per source and weaker cuts. "A min-cost flow by melon's network simplex, 2026-11-18".
- **Prefetches between `top()` and `pop()` of the heaps,** of the arc maps and of the vertex entries: no difference to 6.8 % more separation CPU, never less. "Prefetching in the searches, 2026-11-18".

## Leads

A change that moves the labels cannot be checked bit for bit: the check is the oracle (`test/brute_force/check.py` on the four configurations of `multiflow_benders`), the limit of pushes forced (below), and the suite, comparing the cuts on the relaxation, the lazy cuts and the nodes, not only the time.

1. **Keeping the part of a source from a round to the next.** The part of s depends on x̂ through the options of its quality gains and of the improved copies of A_s only. When none of them moved since the last separation, the part is the same. It needs the part of each source kept sparse (its ρ_s and its coefficients), which is memory in the order of the pairs of a source and an improved copy. Worth it in the late rounds, where few options move; useless at the candidates, which differ from one another.
2. **Smaller A_s.** The test keeps an arc when the best path of s through it, with every option, is as reliable as its best path without any. A test within the budget (the best path through the arc with the options that the budget allows) would remove more, and shorten every search. It is the tight big-M of `docs/formulations/preprocessing.md` applied to the membership of A_s; the cut stays valid as long as the test is.
3. **A cheaper heap.** It holds 33 to 36 % of the separation, `pop` alone 23 to 24 % ("Branches, 2026-11-18"). A scan of an array takes 11 % less separation CPU on the suite but a third more on hexagon; a hybrid that keeps the first and holds on the second was not reached ("The heap, 2026-11-18").
4. **The scans backward.** Half of the vertices settled from s at the points of the relaxation scan their in-arcs for one that carries flow, and a third of those arcs do; with the scans of the last search, 5.5 billion of the 37 billion estimated misses. Measured in "Simplifications of gecot::onaga, 2026-11-19": per-vertex chains of the arcs that took flow, −4.9 % of separation CPU; the single arc that took flow kept per vertex, −4.3 % bit for bit.
5. **The leads of "Found on the way"** above: the flows of a candidate only for the blocks that the master overvalues; the arcs without option of the copies in A_s; the entries of x̂ under the tolerances.

## How to measure a change

```console
$ python3 benchmark/run.py --suite short --time-limit 30 --label before --gecot <binary before> multiflow_benders
$ python3 benchmark/run.py --suite short --time-limit 30 --label after  --gecot <binary after>  multiflow_benders
$ python3 benchmark/compare.py benchmark/results/before.csv benchmark/results/after.csv
```

Run alone on the machine: the suite varies by about 5 % from one run to the next. The reports of the runs (`benchmark/results/<label>/*.json`) hold `root_rounds`, `root_cuts`, `lazy_constraints` and `nodes`: a run that reaches optimality with the same cuts and nodes computed the same thing.

To exercise the limit of pushes end to end, build once with the alias `target_block` of `multiflow_benders` set to `multiflow_target_block<0>` (`max_pushes` is the template parameter of the block: every source stops before its first push), once to `multiflow_target_block<1>` (every source that needs a second push stops after the first), and run the oracle on each: 268 of 268 runs must pass. Without a build, the unit test `multiflow_cuts_are_valid_and_tight` checks the cuts of the block at 0 and at 1 on three small instances: valid in every scenario, tight at a scenario.
