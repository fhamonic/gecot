# `--big-m exact`: what the knapsack constants bound, computed exactly

Status, 2026-11-16: on the branch `big-m-exact`, stacked on `multiflow-target-big-m`. It exists to measure how far the `knapsack_bnb` constants are from what they bound, for every algorithm that reads big-Ms. It is not meant to speed anything up.

## What it computes

For each vertex u of G_t that needs a constant, the value E_u is the largest flow that can reach u over the scenarios within the budgets and the conflict sets. `knapsack_bnb` is an upper bound of this value: it takes every improvement at once for the probabilities, and the best knapsack of the gains under those probabilities.

E_u is not the least valid constant of a copy. A source that reaches u can still have a better path to t that avoids u, so less than E_u can leave u. `misc/design/tight_big_m.tex` calls that least constant intractable.

`compute_exact_big_M_map` solves one MIP per vertex:
- the rows of flow.md with u as the sink, on the subgraph of the vertices that reach u;
- the improved copies bounded by the `knapsack_bnb` constants of their tails;
- the rows of the budgets and the conflict sets, from the solver's own `_options_model`.

It keeps the dual bound, raised by 1e-6 relative, and at most the `knapsack_bnb` value. The constants fill the layer of `tight`, so the copies read the constant of their tail and the target quality gains that of t.

`multiflow_benders` reads only M^t_t. It solves the MIP of t alone, for the targets that have options.

## Size and cost

| run | big-M vertices, all targets | G_t, mean vertices | reach of u, mean vertices | MIP per vertex, 1 thread | exact / knapsack per vertex, mean (min) | vertices already exact |
|---|---|---|---|---|---|---|
| `rural-planting` at 90 | 229,556 | 864 | 64 | 6.4 ms | 0.992 (0.821) | 62 % |
| `urban-greening` at 1000 | 156,695 | 1188 | 172 | 17.9 ms | 0.946 (0.685) | 20 % |
| `periurban-mixed` at 2500 | 146,678 | 609 | 89 | 14.2 ms | 0.980 (0.776) | 41 % |

These were sampled on every 20th target, from a study build.
- Per vertex, the solve dominates: 5 to 17 ms. Creating the model takes about 1 ms, and building it 0.1 ms.
- Sequentially, the MIPs would take 1,466 to 2,799 s per run.

**Reusing one model per graph does not pay.** The suggestion was one model of the contracted G_t per graph, reused across its vertices with the sink column switched on in turn. I measured it on the same sample:

| run | fresh model on the reach | one model of G_t | one model of G_t, arcs outside the reach fixed to 0 |
|---|---|---|---|
| `rural-planting` at 90 | 6.4 ms | 12.8 ms | 7.0 ms |
| `urban-greening` at 1000 | 17.9 ms | 35.4 ms | 18.8 ms |

The bounds agree to 5e-9 relative. The loop that shipped reuses what does pay:
- the TBB loop runs over the vertices of G_t;
- each block keeps one `melon::breadth_first_search` on the reversed graph, and one `melon::views::subgraph` filtered by its reached map;
- each vertex gets a fresh model on that view, with one solver thread.

**A full run, `rural-planting` at 90, 16 threads on 8 cores.** All runs reach the same optimum.

| algorithm | `knapsack_bnb` | `exact` | of which constants |
|---|---|---|---|
| `flow_mip` | 67.7 s | 227.1 s | 169.0 s |
| `flow_benders` | 1.2 s | 169.6 s | 168.8 s |
| `multicut_benders` | 2.1 s | 170.3 s | 168.4 s |

- The constants take 169 s, against a sequential estimate of 1,466 s: 8.7 times faster, on 8 physical cores.
- The `flow_benders` run uses 2,000 s of user time and 296 s of system time in 169.6 s, so 13.5 of the 16 hardware threads are busy on average. The system time is probably the creation of a Gurobi environment per model, which mippp gives no way to share.
- The time limit counts the constants. With `--time-limit 60`, `flow_mip` logs its statistics at 170 s and then stops without a solution.
- `flow_mip -v` reports:
  - on the copies, 0.9856 of the `knapsack_bnb` constants;
  - 140,031 of the 229,163 copies (61 %) not lowered, and none halved;
  - on the target quality gains, 0.9822.

**multiflow_benders** reproduces the roots of the per-target MIPs of `multiflow_target_big_m.md` exactly: 3.630 % on `species-periurban` at 1000, 6.099 % on `urban-greening` at 500, 7.084 % on `periurban-mixed` at 150. The constants take 16.8 s, 4.7 s and 0.7 s.

These are single runs on one instance, plus the sample above: a start for the justification, not the whole of it.

## Checks

- The refactor below it (f80066c) gives the same 851 parity cases, Gurobi models included. multiflow_benders with each `--big-m` kind on three benchmark runs writes the same model, report and solution as before.
- With `exact` (bd67614), parity differs on 22 cases only:
  - 8 parameter listings, which show `exact`;
  - 14 `tight` statistics lines, which gain the count of copies not lowered.
- The oracle passes 268 of 268 runs with `--big-m exact` in the four algorithms.
- ctest passes 76 of 76, without warnings, and both new headers compile alone.

## Traps met

- `std::movable` is false for mippp models, which have no move assignment. The factory needs `std::move_constructible` only.
- Without `ArcBigMs`, the knapsack map holds `DBL_MAX` off t. The exact MIP bounds its copies by the constants of their tails, so `exact` computes those of every tail. Before that fix, multiflow's roots dropped below the exact ones.
- multiflow builds a graph for every target, about 600 on `species-periurban`, but only the 160 with options read M^t_t. Solving them all took 55.7 s instead of 16.8 s.

## Not done

- **Parallelism across targets.** Making the graph the first dimension across graphs would split every solver's target loop in two: compute all G_t first, then add their rows. multiflow's MIPs of t run one after another, each on all of the solver's threads.
- **A stop during a MIP.** Ctrl-C is checked between the MIPs only, and the vertices left keep the `knapsack_bnb` constant.
- **A parity case with `exact`.** `capture.py` has none.
