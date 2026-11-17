# Solvers: what hides the flow of the algorithms, and what can be lifted out

Status, 2026-11-17: a review, on the branch `solvers-boilerplate-review`, of `include/gecot/solvers` and of what it depends on, at main d5b126f, written for the owner, who rules on each proposal. Sections 1 to 10 are the review as it was ruled on: their line numbers and sizes are those of d5b126f. The owner then asked for its proposals to be implemented, on the same branch: §11 is the record of what was done, checked and left.

- **What was asked.** Whether boilerplate can be lifted out of the algorithms and their code simplified so that their algorithmic flow reads clearly. Performance comes first, maintainability a very close second, modularity third, where modularity serves the experiments that justify the choices of parameters.
- **What stands from the review of 2026-11-14** (`maintainability_review.md`) and is not reopened: the model calls stay inline in `solve()`, in dense blocks under a comment that names their family of constraints; no helper for the opening of the masters; `flow_mip` and `flow_benders` are the reference style; the experimental switches and the commented contraction block stay; `tree_lp` keeps its single budget.
- **Sources.**
  - My reading, in full, of the twelve files of `include/gecot/solvers`, of `flow_rows.hpp`, `helper.hpp`, `compute_contracted_graph.hpp`, `compute_contracted_generalized_flow_graph.hpp`, `progress.hpp`, `progress_bar.hpp`, of the interfaces and their sources, and of the diff of the unmerged branch `multicut-root-cuts`.
  - Not read by me: the two labelling headers, `compute_big_M_map.hpp` past its first lines, `compute_tight_big_M_maps.hpp`, `compute_exact_big_M_map.hpp`, `pc_num.hpp`, `parallel_pc_num.hpp`, `criterion_analysis.hpp`, `log_table.hpp` and `concepts/instance.hpp`. A skeptic read the two big-M computations and the two indices for its claims; nobody looked in the others for what could be lifted, and the review of 2026-11-14 covered their comments, names and constants only.
  - Three measurements with `build/gecot` of 2026-11-17 05:27 (§3).
  - Six skeptics, each asked to refute one cluster of my draft claims against the code and against the sources of mippp and melon as built. None could build or run. Of the 41 claims, 8 held as written, 32 were restricted or corrected and 1 was refuted. Their corrections are folded in, and §9 lists what they removed.
- **Evidence marks.** **[read]**, the default: read by me in the code, and by a skeptic. **[once]**: the reading or the count of one of us only, a skeptic unless it says mine; where it cites a line, I read that line. **[run]**: measured here. Line numbers are those of d5b126f: each of the 173 citations was resolved against its file by a script, and its first line read.
- **Vocabulary.** *Parity*, as in `CONTRIBUTING.md`: the results are bitwise identical, and a model receives the same variables, rows and columns in the same order with the same coefficients.

## 1. Summary

- **Little boilerplate is left to lift in the three reference MIPs.** In `flow_mip`, `flow_benders` and `multicut_benders`, 58 to 71 % of `solve()` is the program, and the scaffold of the preprocessing is 8 to 12 lines. An object of the base that would hold it, my first draft, did not survive the verification: one call is common to the five sites (§9).
- **The flow hides in three long functions and in the base.**
  - `multiflow_benders`: 93 of the 239 lines of `solve()` are the rounds on the relaxation, and 400 of the 733 lines of the file are the subproblem (A1).
  - `path_lp::relax`: 488 lines, with a loop of 268 in which the search pricing is written (A2).
  - `mip_solver_base`: 776 lines in four concerns, of which `path_lp` reads two (A4).
  - `Greedy::solve` is long because its rule is: 152 of its 264 lines.
- **What is written several times is steps, not scaffolding**: the cut of a target in two files (A8); "buy" and "remove" an option in four places (A3); the graph of a target in three forms, renumbered in two places (A9); the check of a start in four places and the trace of a master in two (A6).
- **Nine proposals change no behaviour** (A1 to A9). Three change rows, bits or a result and wait for a ruling (B1 to B3); I recommend against B1 and B3 unless a measurement asks for them.
  - **A8 reverses a refusal of 2026-11-14**, the shared cut builder, on one reading of a header of GCC that was not compiled. It is the least settled of the nine: its first step is a `static_assert`.
  - A1 and A8 each move model calls out of `solve()`, which `CONTRIBUTING.md` keeps inline: they need the owner's word, not only a check.
- **Performance.**
  - Nothing proposed adds work to a hot path. The hot path of the fastest exact algorithm is `_largest_flow`, about half of its wall-clock time (§3); A1 moves it to another file without changing it, which is to be measured all the same.
  - Two places pay for what is not used: every algorithm copies the contracted graph and five maps once per target, and three algorithms build two maps that they never read (A9); the six MIP units each compile the two heuristics (A7).
  - A gain that I expected is not there: the separation of `flow_benders` runs its targets one after the other, and it is 7 % of its wall-clock time on the instance sampled.
- **The harness does not run** the search of a Benders decomposition without lazy constraints, which is the path of the open-source default, nor several parameters of the algorithms. It is widened first (S0).
- **Experiments.** A switch costs a handful of cheap sites. A variant of the structure of a step has been a copy five times, for 162 to 319 lines each, three of them folded into switches since; A1, A2 and A8 leave a seam where those copies were (§7.1). The measurements behind the defaults are mostly in commit bodies and in an untracked directory (§7.3).

## 2. What each `solve()` is made of

My tagging of each line of the four MIP entry points, in five classes. The counts are approximate: a line of locals goes with the block that reads it. [once]

| `solve()` | lines | the program: model calls and steps | preprocessing scaffold | solver session | statistics, logs | start, fallback, readback |
|---|---|---|---|---|---|---|
| `flow_mip` (`flow_mip.hpp:75-231`) | 157 | 91 | 12 | 11 | 17 | 13 |
| `flow_benders` (`flow_benders.hpp:38-202`) | 165 | 117, of which the cut 32 | 8 | 7 | 13 | 7 |
| `multicut_benders` (`multicut_benders.hpp:142-315`) | 174 | 123, of which the cut 42 | 8 | 4 | 17 | 6 |
| `multiflow_benders` (`multiflow_benders.hpp:491-729`) | 239 | 80, and the rounds on the relaxation 93 | 10 | 6 | 24 | 12 |

- *Preprocessing scaffold*: the labels, the contraction call and its ten arguments, the aliases of their results.
- *Solver session*: the refusals of a backend, the tolerances, the stopwatch, the time limits, the numeric focus.
- *Statistics, logs*: the rows of the preprocessing table, the sizes of the graphs, the progress bars, the debug lines, the stats of the report.

The two relaxations and the heuristics are not models with a scaffold around them: they are one long function each, and what hides the flow is the size of the steps written in place.

| Function | lines | its largest steps written in place |
|---|---|---|
| `path_lp::relax` (`path_lp.hpp:470-957`) | 488 | the search pricing 107 (`:711-817`) and its precomputation 34 (`:505-538`); the master 33; the columns 39; the bounds of a round 27; the rounding 27; the check of the start 17; the trace log 19 |
| `tree_lp::solve` (`tree_lp.hpp:134-422`) | 288 | the trees of the start 43 (`:240-282`); the master and its pricing models 56; the pricing 28; the trace log 16 |
| `Greedy::solve` (`greedy.hpp:36-299`) | 264 | the rule 152: the tie 26, the removals 33, the purchases 48, the swaps 45; the rest 112, of which the bookkeeping of a step taken 25 and the tables 15 |
| `Static::solve` (`static.hpp:29-160`) | 132 | the purchases 56; the removals 34 |

`mip_solver_base.hpp` is 776 lines for six derived structs, in four concerns: the solver session and its search (`_set_search`, `_solve`: about 230 lines), the criterion, the options and the start (about 190), the preprocessing switches, wrappers and table (about 110), the Benders search (`_benders_search`, `_hand_back`: about 190). `path_lp` derives from it and reads nothing of the last two; `flow_mip` and `tree_lp` read nothing of the last.

## 3. Measurements

Taken to know where a change can cost, before proposing any. `perf` is not allowed on this machine (`perf_event_paranoid` is 4), so the program was run under `gdb`, stopped every 0.1 s, and each stop classified by the frames of its threads (appendix). A stop is a sample of wall-clock time. One run each, 30 s limit, Gurobi, 16 threads. [run]

The binary is the one of the capture `branch-root-cut-gap` (sha256 0e8a4bc0…): built at 05:27 from be9e876 with the changes that dd95109 committed at 05:46, the two commits that d5b126f squashes. It has `--root-cut-gap`. Whether the last edits of dd95109 came after the build is not known; they are in the rounds of `multiflow_benders`, not in its separation, and `flow_benders.hpp` is the same in both.

| Run | Samples | Where the wall-clock time goes |
|---|---|---|
| `flow_benders`, periurban-mixed at 25000 | 301 | the solver alone 90.7 %; the callback 7.3 %, of which the flows and potentials 6.6 %; before the search 2.0 % |
| `multiflow_benders`, species-natural at 6000 | 150 | a separation running 60.7 %; the solver 36.7 % |
| `multiflow_benders`, bike-squirrel-suburb at 500 | 300 | a separation running 55.0 %; the solver 42.7 % |

Inside the separations of `multiflow_benders`, over 1391 and 2307 samples of a thread: `_largest_flow` 93.5 % and 93.2 %; the code of `separate` itself 4.2 % and 5.5 %; `_source_arcs` 1.0 % and 0.5 %; malloc, free, memset and mprotect 1.1 % and 0.2 %.

What they say for this review:

- **`_largest_flow` is the hot path of the fastest exact algorithm**: about half of its wall-clock time. A proposal that touches it has to be measured, whatever it looks like.
- **Reusing the structures of a separation across calls is not worth code.** `separate` builds, per block and call, a `flow_search` of seven maps, a vector and a heap, and a `source_search` that only `--no-stored-arcs` reads (`multiflow_benders.hpp:428-430`). The allocation is about 1 % of the separation. The fills of four of these maps are not in that figure if they are inlined: they are then within the 4.2 % and 5.5 % of `separate`, with its loop over the sources. A search kept per block would also be memory in the configuration that `--no-stored-arcs` exists for.
- **The sequential separation of `flow_benders` is not its bottleneck** on that instance. I expected it to be, since the run used 4.5 cores of 16 on average (122.8 s of user time for 30.2 s); the samples say the solver holds the time. Running its Dijkstras in parallel would gain at most the 7 % of the callback there.

## 4. First: what the harness does not run

`CONTRIBUTING.md` asks that a refactor be checked by `test/parity/capture.py`. Several paths that the proposals below touch are in none of its cases, nor in `test/brute_force/check.py`:

| Not run | Where it is | Touched by |
|---|---|---|
| The search of a Benders decomposition without lazy constraints: no case has `--no-lazy-constraints`, and the only case on another solver than Gurobi is `flow_mip --solver highs` (`capture.py:61-80`) | `mip_solver_base.hpp:476-606`, the path of the open-source default | A4 |
| `--root-cut-tolerance` and `--root-cut-gap` at another value than their default | `multiflow_benders.hpp:660-679` | A1 |
| `--evict-age`, `--max-activations`, `--relative-gap`, and any parameter of `tree_lp`; `tree_lp` with a MIP start | `path_lp.hpp:857-910`, `tree_lp.hpp:240-282`, `:398-401` | A2, A5 |
| A MIP start that is over the budget or buys two options of a conflict set: the four messages | `mip_solver_base.hpp:609-621`, `path_lp.hpp:481-497`, `tree_lp.hpp:276-282` | A6 |
| The lines logged at `-vv`: the trace of the masters, the removal table of `greedy`, both tables of `static` (the capture is at `-v`) | `path_lp.hpp:580-592`, `tree_lp.hpp:206-221`, `greedy.hpp:72`, `static.hpp:56`, `:105` | A2, A3, A6 |
| A stop by the time limit or by Ctrl-C | every loop | by reading only: the timing is not reproducible |

- **S0. The matrix is widened before the first refactor**, with a capture of main as the baseline: the three Benders with `--no-lazy-constraints` and `multiflow_benders --solver highs`; the two parameters of the rounds at `1e-3`; `path_lp` with each of its three parameters; `tree_lp --evict-age 2` and `tree_lp --mip-start-algorithm greedy`; an infeasible start given to `flow_mip` with `--time-limit 0` and to `path_lp`; one instance of each of `path_lp`, `tree_lp`, `greedy --decremental` and `static --decremental` at `-vv`. About 20 lines of `capture.py`. Each new case is captured twice on main before it is trusted: HiGHS, the loop that solves the master again and a time limit of 0 may not give the same output twice.
- The order of the terms inside a row and the cuts added during a search stay invisible to the hash, as `CONTRIBUTING.md` says: a diff that rewrites a cut expression is read by hand.

## 5. Proposals that change no behaviour

Each moves statements whole: no model call changes its place in the sequence that the solver receives, and no floating-point expression is rewritten. The build has `-O3` without `-march=native`, so the compiler has no multiply-add to form, and a product is not split from its sum by a function call.

### A1. `multiflow_benders`: the separation in its own header, the rounds as a step

`multiflow_benders.hpp` is 733 lines: 400 for `target_block` (`:61-460`), which is the subproblem, and 239 for `solve()`, of which 93 are the rounds of cuts on the relaxation (`:595-687`).

1. **`target_block` in its own header.** It names no model type and reads no member of `multiflow_benders`: the flag `no_stored_arcs` enters through `compute_sources` and `separate` (`:466`, `:485`). `multiflow_benders.hpp` keeps its parameters, `_compute_sources`, `_separate` and `solve()`: 333 lines.
   - The block is more than the separation: it also stores what the master and the floor of the rounds read (`flow_bound`, the constants of the target quality gains, `cut::settled`).
   - Four places in the design notes cite `target_block::separate` or `max_pushes` in `target_block`; they follow.
2. **`floor_at` out of `solve()`** (`:606-625`): 20 lines, a function of the instance, the blocks, the point separated and its cuts. It stands between the state of the loop and the loop. It becomes a member next to `_separate`.
3. **The rounds as a member of `multiflow_benders`**, called from `solve()` between the cuts of the start and the search. It receives the model, the stopwatch, `x`, `f`, the blocks and the lambda `cut_constraint` of `solve()`, and returns the value of the last relaxation, the number of rounds and of cuts. The rows are still built by the lambda written in `solve()`, as the cuts of `_benders_search` are. `solve()` then reads: the master, the blocks, the cuts of the start, the rounds, the search: about 150 lines.
   - What stays as it is: `set_continuous`, `set_binary`, the bare end of the progress and the debug line run even with `--no-root-cuts` (the flag is in the condition of the loop, `:626`), and that configuration is captured at `-v`.
   - **For the owner.** This moves `set_continuous`, the solves of the relaxation and the `add_constraint` of its cuts out of `solve()`, where `CONTRIBUTING.md:16` keeps the model calls. `_benders_search` does the same since d187663 for the cuts of the search; that is an argument, not a ruling.
   - What stays in `solve()`: the stats and the bound added after the search (`multiflow_benders.hpp:716-726`). The report does not exist before the search, and the order of the stats is part of the JSON.
   - Two traps for whoever moves it. The two tests of a cut, `multiflow_benders.hpp:660` and `:666-667`, look like one and are not: merged, they round differently and add other cuts. And `:499-500`, the numeric focus set at the top of `solve()`, is there for the rounds.
4. **A unit test of `separate`**, which none of the 77 unit tests covers: the exact algorithms are only run end to end, by the oracle and the captures. At a scenario the value of the cut is the flow that reaches t, and the cut is at least that flow at the other scenarios. With `max_pushes` a template parameter of the block, the fallback of a source, which no run takes (`multiflow_benders_separation.md:25`), is tested at 0 and 1 where the note's protocol asks for two hand-edited builds.

- **Not proposed: a generic function of the rounds that takes the algorithm's part as a lambda.** It would have one caller on main. Its second caller is the branch `multicut-root-cuts`, whose own note argues against merging. And its interface is recent: of the three values that the lambda would return, two (the violation and the floor) were added on 2026-11-17. If another algorithm gets rounds, the member of step 3 is the starting point: 49 of its 93 lines are loop control, and the five steps of `multiflow_benders` in it (reading f, separating, the floor, the violation, the test and the row) become the lambda.
- **Parity.** By construction. The cuts of the rounds are rows of the last model, so the hash pins their order and coefficients.
- **Performance.** `_largest_flow` changes file, not code: the same inline members in the same unit. The same machine code is expected, not shown: the protocol of `multiflow_benders_separation.md` (the short suite before and after, comparing `root_rounds`, `root_cuts`, `lazy_constraints` and `nodes`) settles it. A template parameter for `max_pushes` keeps it an immediate at `multiflow_benders.hpp:356`.
- **Cost.** The symbols of the block change, so object files cannot be compared.

### A2. `path_lp`: its two pricings as two structs, its steps named

This is the split of `path_lp` that the review of 2026-11-14 left to the owner (§5.5, §11): a proposal for that design.

`relax()` is 488 lines and its loop 268 (`path_lp.hpp:644-911`). The pricing is chosen at run time by `search_pricing` in six places: `_pairs` (`:369-410` against `:411-442`), the precomputation of the reaches (`:505-538`), the memo of the searches, declared for both (`:629-634`), the lambdas `enumerate` (`:698-709`) and `search` (`:720-817`), the dispatch (`:818-824`), the stats (`:942-954`).

1. **Each pricing a struct** that owns its precomputation and its memo, with three touch points: building the pairs, pricing a pair, its stat. `relax()` keeps the loop and chooses per round with a generic lambda.
   - Three conditions on the call that prices a pair, from what the two lambdas read today:
     - **Three outcomes**, not two: a bound with a column, a bound alone, and cut short. The search tests the stop only after its skip, its case without a positive dual and its reuse of the last column (`path_lp.hpp:727-746`), so that "a reused pair never cuts a round short" (`path_lp.md:240`). A stop test in front of the call would change that.
     - **The rows of the pair, not only their duals**: the search reads the keys of the purchase rows of p in their order (`path_lp.hpp:731-737`).
     - **The duals read lazily**: the skip path reads two duals and returns. A vector of the duals of the pair built by the caller would allocate once per pair and round on that path, with 59 000 to 269 000 pairs (`path_lp.md:11`).
   - The pairs stay with `relax()`: the master reads their case and their start columns, and `_pairs` decides the model, since the list of pairs and the C_k depend on the pricing (146 987 pairs for the search against 131 525 for enumerate on Quebec at 1000, `path_lp.md:129`).
   - The pricing must not become a template parameter of `relax()`: `path_lp_interface.cpp` instantiates it for eight LP types in one unit.
2. **The rounding as a function** (`path_lp.hpp:913-939`) of the instance, the budget, x, the start and the tolerance.
3. **The bounds of the options as a lambda defined before the loop** (`path_lp.hpp:833-851`), called with the duals and the bound of the round. As a member it would take eight parameters: it reads the rows of the master and updates a minimum over the rounds. `options_sets` (`:559`) moves next to it.
4. **The exits of the loop through one local lambda.** Four exits (`path_lp.hpp:650-659`, `:857-865`, `:895-899`, `:900-906`) repeat the end of the progress, a line "Column generation stopped: …" and the status, in an order that is a contract (`progress.hpp:23-27`). The four texts stay.

- **Size.** `relax()` goes from 488 to about 290 lines, about 255 with the small lifts of A6; the loop from 268 to about 130. I had estimated 200: what remains is the accessors of the duals, redefined at each round (21 lines), the activation (26), the master and the columns.
- **Parity.** The arithmetic is unchanged if the duals are passed as the accessors of today. Each body of the `parallel_for` pushes at most one column, so the order of the columns under `--max-threads 1` is kept.
- **Performance.** The callbacks of `_labels` stay closures of the function that calls it, so `_labels` is instantiated on concrete types as today; `weight(i)` runs in its inner loop and must stay a template argument, never a `std::function`. The call returns a pointer to the column that the pricing already holds, which keeps the one copy of today. To measure on a run bound by its searches, as bike_small at 400 single-threaded (`path_lp.md:218`).
- **Compile memory.** Possibly lower, not measured [once]: every LP backend returns the same type of duals, so a pricing that takes the duals and the rows, not closures of `relax<LP>`, is instantiated once where the search and its `_labels` are instantiated eight times.

### A3. Heuristics: the two changes of the landscape written once

`heuristic_landscape` writes "buy the option i" and "remove the option i" four times: in `buy` (`heuristics_common.hpp:134-146`) and `scores_with` (`:202-209`), in `remove` (`:149-168`) and `scores_without` (`:263-277`). The same test is spelled `!solution[j] || j == i` in one and `solution[j] == 0.0 || i == j` in the other. `scores_with` (`:177-233`) and `scores_without` (`:237-299`) are otherwise one loop: the bar, the blocks, the copies of the maps, the evaluation, the restore by assignment.

1. **Each change once**, on three maps given: `buy`, `set` and `scores_with` call the first, in the order they are given; `remove` and `scores_without` the second, which also takes the case and the solution.
2. **One loop** that takes the change as a lambda; `scores_with` and `scores_without` stay as its two callers, which a test names (`solvers_test.cpp:93-95`).
3. **`std::erase_if(free_options, unavailable)`** at `greedy.hpp:173-177` and `:292-294`: eight lines for "drop the options that no longer fit".
4. **The ranking of the options once.** The bindings of `free_option_tier` and `ratio_key` to the costs, the ratios and a score, with the comparator on them, are 11 lines in each file (`greedy.hpp:81-91`, `static.hpp:125-136`) that differ by the name of the score and the direction.
5. **The opening** (`greedy.hpp:46-58`, `static.hpp:32-44`): 13 identical lines. Shared, it must hold neither `landscape.set`, which only `greedy` calls there, nor the costs, which `static --decremental --only-dec` never computes and whose computation can throw.

- **Size.** About 90 lines less in `heuristics_common.hpp`, about 25 in the two solvers.
- **What is not there to lift.** Of the 264 lines of `Greedy::solve`, 152 are its rule, and its two loops cannot be read without the 25 lines that apply a step. I asked whether the rule could be read in 80 lines if the rest left: no.
- **Parity.** The changes are additions, subtractions, maxima and assignments in the same order. The tables that `static` and the removals of `greedy` log are at `-vv`: S0.
- **Performance.** A change runs once per case and option, before a `parallel_pc_num` that runs a Dijkstra from every source. Passed as a template argument it is a direct call.

### A4. `mip_solver_base` in three structs, the Benders search in two functions

1. **A chain of three plain structs**, each in its file:

   | Struct | Holds | Derived |
   |---|---|---|
   | `mip_solver_base` | the tolerance, the time limit, the numeric focus, the start; the criterion and the options (`formula_variable_visitor`, `_add_budget_rows`); `_set_search`, `_solve`, the fallback and the readback | `path_lp` |
   | `flow_preprocessing` | the seven switches of the preprocessing, `_is_target`, the labels, `_options_model`, the table of the cases and the sizes of the graphs | `flow_mip`, `tree_lp` |
   | `benders_base` | `no_lazy_constraints`, `_benders_search`, `_hand_back` | the three Benders |

   - Every read across the three goes up the chain; no CRTP; no call changes in a solver, only the base of five of them. No code builds these structs as aggregates.
   - `add_preprocessing_options` and `add_benders_options` take the narrower type: that `path_lp` has no preprocessing is then checked by the compiler.
   - An exact order would have four levels, with the search of one MIP apart, since `path_lp` and `tree_lp` do not call `_solve`. Functions that are not instantiated cost nothing: three is enough.
   - `big_Ms`, declared four times with the same default (`flow_mip.hpp:33`, `flow_benders.hpp:32`, `multicut_benders.hpp:41`, `multiflow_benders.hpp:49`), can move to `flow_preprocessing` with the four calls of `add_big_M_option` left where they are, which keeps the order of the listings. `tree_lp` then carries a member that it does not read.
2. **`_benders_search` in two functions** (`mip_solver_base.hpp:442-607`): the callback of the lazy constraints (26 lines) and the loop that solves the master again (131 lines) share no state. The first names the callback handle of M, so it stays under the `if constexpr` of today or takes the concept `lazy_constraint_model`.
   - Left as it is: about 35 lines of the loop repeat the session of `_solve` around it (`_set_search`, `_model_report`, the readback of the bound and the nodes, the end of the progress). Merging them would change the JSON: the stats come as `nodes`, `lazy_constraints` on one path and as `lazy_constraints`, `master_solves`, `nodes` on the other, and the report of the model is taken before the first solve on the loop.

- **Why.** The search of a Benders decomposition, steps 2 to 4 of `benders.md`, gets a file where it is the subject; the base loses the two concerns that half of its derived structs never read.
- **Parity.** Code moves between structs and functions. The loop is in no capture: S0.
- **Compile memory.** Two more function templates per Benders and backend, with the same bodies.

### A5. `tree_lp`: the master apart from the pricing models

The rows of the master are created in the same loop as the preprocessing of each case and the construction of the pricing models (`tree_lp.hpp:163-199`): the program of the master cannot be read without stepping over the labels, the table, `targets.emplace_back` and the numeric focus of each pricing model. The pricing models are separate solver objects. A first loop builds them; a second one holds the calls of the master in the order of today (z_k, its contribution row, the uniqueness rows of its targets, their purchase rows, case by case). [once]

- **Parity.** The same sequence for the master; the captures of `tree_lp` hash it.

### A6. Small lifts

| What | Where | Note |
|---|---|---|
| `_set_time_left(model, sw)` in place of its body | `path_lp.hpp:645-648` | token for token `mip_solver_base.hpp:291-294` |
| The same with a deadline for the rounds | `multiflow_benders.hpp:597-599`, `:626-630` | a third spelling, with half of the limit |
| `time_up` as a member of the base | `path_lp.hpp:477-479`, `tree_lp.hpp:142-144` | verbatim. The `>=` of `mip_solver_base.hpp:585` is not folded in |
| One helper that gives the defect of a start: over the budget, two options of a conflict set, or none | `mip_solver_base.hpp:504-511`, `:609-621`, `path_lp.hpp:481-497`, `tree_lp.hpp:276-282` | the four messages assemble byte for byte around the two fragments; `tree_lp` builds its start twice |
| The trace log of a master, with its label | `path_lp.hpp:580-592`, `tree_lp.hpp:206-221` | in `column_generation.hpp`; `tree_lp` keeps its guard in front of `get_solution()` |
| An alias for the column manager, a getter for the reduced cost of an entry | `path_lp.hpp:596-600`, `tree_lp.hpp:225-229`; five spellings of `entry.state.template get<colgen::reduced_cost>()` | the two managers are one type |
| `lowered_big_M_statistics` next to `contracted_flow_graph` | `flow_mip.hpp:39-72` | 34 lines of a statistic of `--big-m tight` and `exact` in the reference file |
| Positive members behind the negative flags: `pruning`, `root_cuts`, `in_out`, `stored_arcs`, `lazy_constraints` | `!no_pruning` at five sites, `multiflow_benders.hpp:466`, `:626` | the flags keep their names, set through `add_flag_callback`. A flag is listed from its count, not from its member, so the listing should not move: to check on the capture of `-P` |

Looked at and left: a named helper for "the master overvalues this block" (`flow_benders.hpp:184-186`, `multicut_benders.hpp:302-304`, `multiflow_benders.hpp:706-708`). The three are one expression, but it is a test with a side effect that `_hand_back` needs (it writes the value in the candidate), written `<=` then skip so that a NaN still receives its cut, and a helper saves no line.

### A7. The MIP start computed in one unit

`with_mip_start` (`abstract_mip_interface.hpp:149-184`) is a template of the solver type, though its body reads only `mip_start`, `feasibility_tol` and `time_limit`, three members of the base. Each of the six MIP units instantiates through it `Static::solve` and `Greedy::solve`, with the landscape, the TBB loops and the Dijkstra of `parallel_pc_num`. These are the units that take 2 to 4 GB. A function that takes the base by reference, defined in one source file, compiles the two heuristics once. [once]

- **To measure**: the peak memory and the time of one MIP unit before and after (`/usr/bin/time -v`). No effect on a run.

### A8. The cut of a target written once

`flow_benders.hpp:126-156` and `multicut_benders.hpp:218-249` build the same expression, the cut of a target of `benders.md`. The parts of the vertices are identical token for token; the parts of the arcs differ by the order of one product, `M * x(i) * sigma` against `M * sigma * x(i)`, which gives the same bits. `multicut_benders` then adds its term of the target quality gains (`multicut_benders.hpp:250-256`) and scales ρ beforehand.

- **Proposal.** One function holds the expression, next to `flow_and_potentials` whose ρ it reads. Both solvers call it, and `multicut_benders` adds its third term to the result.
- **Why it comes back.** The review of 2026-11-14 dropped "a shared cut builder" (§10): mippp joins the terms of A + B with `unordered_concat`, which under GCC 15.1 can swap them. Read again in the header of GCC 15.1 (`/opt/gcc-15/include/c++/15.1.0/ranges`), the swap is decided by a property of the two ranges, not by their types: `concat(r1, r2)` fails to be a range when r2 is a common range and the iterator of an operand has no `==`. A sum built by `xsum` is a join over ranges returned by value, which is never a common range, and every right operand of these cuts is such a sum. The two files already differ in the type of their arc term, and both get the written order. So a lift of the lambda as it is cannot reorder the terms. [once], derived from the header and not compiled: a `static_assert` that `std::views::concat` of the two ranges is a range pins it at build time.
- **The hazard stays real for a change of shape**: a variable, a single product or a materialized expression written on the right of a lazy sum moves to the front.
- **Conditions.** The lambdas inside the function capture its reference parameters only: x, y or ρ taken by value dangle once it returns. The product of the arc term takes one of the two orders.
- **Size.** About 30 lines less.
- **Parity.** By construction if the reading above is right, and the `static_assert` is what pins it: the hash is not a safety net here. mippp merges the terms of a variable in the order of the row (`mippp/solvers/model_base.hpp:354-360`), and a swap of two blocks of a row changes the bits of a merged coefficient only where three terms or more of one option are spread over both.
- **For the owner.** `CONTRIBUTING.md:16` allows two helpers that build rows. `add_flow_rows` has been a third since ea09ace, for the rows that three models share; this would be a fourth, for a row that two files state.

### A9. The contraction gives what its callers read, and the graph of a target has one static form

G_t exists in three forms: `contracted_flow_graph` (a mutable graph, an arc per improvement; read by `flow_mip`, `tree_lp` and `multiflow_benders`), `contracted_graph_data` (a static graph, a list of options per arc; read by `flow_benders` and `multicut_benders`), and the `target_block` of `multiflow_benders` (a static graph, an arc per improvement). The two static forms are the same graph, renumbered by the same comparator in two places (`compute_contracted_graph.hpp:307-341`, with its two branches, and `multiflow_benders.hpp:135-144`); only their arc maps differ.

1. **A copy per target, for every algorithm.** `compute_contracted_generalized_flow_graph` ends with `std::make_tuple` on its named locals (`compute_contracted_generalized_flow_graph.hpp:153-155`), which copies the contracted graph and its five maps. `std::move` on each keeps the type that the callers and four tests destructure.
2. **Two maps built for three algorithms that never read them.** `contracted_flow_graph` carries the improvements twice: as `probability_map` and `improvement_option_map`, and as `base_probability_map` and `arc_options_map`, a vector per arc with one allocation per improvement (`compute_contracted_graph.hpp:91-101`). The second pair is read by `compute_contracted_graph_data` and by the tight big-Ms only.
3. **A list per arc that holds one entry at most.** Each improvement is an arc of its own (`compute_contracted_generalized_flow_graph.hpp:72-80`), the tight and exact layers add none, so `improvement_big_M(a, n)` is only reached with n = 0 and the loops over `iota(0, arc_options_map[a].size())` of the cuts run once at most.
4. **One static form**: the graph, t, the source qualities and their options, a probability and an optional option per arc. `contracted_graph_data` and `target_block` hold it, and the renumbering is written once. `flow_and_potentials` keeps its bits: the same graph, the same lengths, the same order of settling.
   - The big-Ms stay beside it: `flow_benders` and `multicut_benders` need the constant of each tail, `multiflow_benders` only that of t and those of the gains.
   - `contracted_flow_graph` stays for `flow_mip` and `tree_lp`: renumbering would change the order of their rows.
   - `compute_tight_big_M_maps` then reads the optional option where it reads the list today, at four sites, and a base probability computed on the fly at three; the matrix has its cases.
   - The cuts still emit one term per improved arc, in the order of the arcs.
5. **`contracted_graph_data` as an aggregate**: its five defaulted members and its constructor of eleven parameters (`compute_contracted_graph.hpp:262-289`) go. Its two returns then name the template argument, which an aggregate cannot deduce through `reference_wrapper<const C>`.

- **A question of the keep-list.** The one client of several options per arc is the commented block that the owner keeps (`compute_contracted_graph.hpp:344-557`): it puts the improvements on the arc of the base probability. If the list goes, the form that this block returns goes with it. The block no longer compiles as written: it destructures three values where `make_static_digraph` returns one flat tuple, and passes nine arguments to the constructor of eleven.
- **Parity.** By construction for 1 and 2. For 3 to 5 the models of `flow_benders` and `multicut_benders` are hashed, and the cut expressions are read by hand.
- **Performance.** In the preprocessing, not in a hot path, and not measured: one copy of a graph and five maps less per target, and no allocation per improvement for three algorithms. In `multiflow_benders`, `_largest_flow` reads the graph and its two arc maps as members of the block; held in a plain struct they are the same loads.

## 6. Proposals that change rows, bits or logs

### B1. The cuts of `flow_benders` and `multicut_benders` as data

`multiflow_benders` holds a cut as a constant and a coefficient per option (`target_block::cut`) and turns it into a row in one place (`cut_constraint`, `multiflow_benders.hpp:574-584`). The two other Benders could compute theirs as the same data, in one pass over the vertices and the arcs of G_t, and use the same builder. A8 is the alternative that keeps the rows.

- **For.**
  - Each pass is made once. Today a sum whose terms carry a constant is walked once for the constant when it is built and again when the row is read (`mippp/linear_expression.hpp:369-376`): `flow_benders` runs its loop over the vertices twice per cut, `multicut_benders` three times, and calls its cut twice per target.
  - The exact zeros leave the row. A source quality gain of 0, a σ of 0 and a ρ of 0 are handed to Gurobi as entries today; only the COPT and CPLEX paths of mippp strip them from a lazy row.
  - A cut is a value: it can be tested without a solver, and it is the type of `multiflow_benders` and of the branch `multicut-root-cuts`.
  - The nested views of the cuts leave the two units: favourable for the memory of their compilation, not measured.
- **Against.**
  - **It is not parity, and the harness cannot arbitrate it.** The entries come by option where they came by vertex then by arc, and the zeros go. For `flow_benders`, with the sums made in the order of today, the coefficients keep their bits, the hash does not see the order inside a row, and Gurobi seems to drop the zeros itself (22 nonzeros reported on `test` at 10, which is a skeptic's count by hand of the non-zero ones [once]; `capture.py --keep-models` settles it): the captures would most likely be equal while every row handed to the solver changed. Only the nodes, the cuts and the times of the suite would show an effect.
  - **For `multicut_benders` the bits and the parallel loop exclude each other.** A group cut as a sum of the vectors of its targets associates the coefficients otherwise, and other bits show in the seed rows. Accumulating into one vector per group keeps the bits and is sequential.
  - **The lambda reads as the formula** of `benders.md`; the data is an accumulation loop, as `multiflow_benders.hpp:446-456`. The ruling of 2026-11-14 is that the program is stated by the code.
  - The same builder keeps a trap of lifetime: the row reads the cut as it reads ρ today, unless it is materialized.
- **What it can gain** is bounded by the callback: 7 % of the wall-clock time of `flow_benders` on the one instance sampled, nothing measured for `multicut_benders`.
- **A parallel loop needs no cuts as data.** The Dijkstras of the targets can run in a `tbb::parallel_for` today, filling a vector of flows and potentials that the sequential loop then reads unchanged, with the same bits: `multicut_benders` already computes them all before any cut (`multicut_benders.hpp:264-268`). It keeps every ρ alive at once (47 MB on hexagon 1_1900_3800), and its gain has the same bound.
- **My recommendation**: A8, and neither B1 nor the parallel loop unless a run of the suite shows the callback to matter on the instances that the two baselines are compared on.

### B2. `tree_lp` keeps the value and x of its last solve, as `path_lp` does

A master changed since its last solve gives no solution. `path_lp` keeps the value and x of each solve (`path_lp.hpp:574-579`, `:660-666`); `tree_lp` reads the model, so it solves again after a stop (`tree_lp.hpp:387-393`) and guards its trace log. This is the root of three differences between the two loops: the trace log, the stop block, the stat `relaxation_value`. With the last solve kept, the stop block of `tree_lp` is that of `path_lp` and the trace log one function. It changes the result of `tree_lp` on a stopped run, and the round counted by the re-solve. A ruling, not a refactor.

### B3. The heuristics store the gain, not the ratio times the cost

`greedy` recomputes the gain of an option as its ratio times its cost (`greedy.hpp:265-266`, `:280-282`, and `:131`, `:145`, `:152` for the removals), and `static` at `static.hpp:77`, `:141-142`. (g / c) · c is g only when c is a power of two, so storing the gain changes the last bit of `previous_score` on the other costs, and from the next step on a tie at the edge of the window can flip. The unit tests pin selections, on costs of 1 for most; neither they nor the harness would show the change short of a flipped tie.

- With the gain stored, `previous_score` after a purchase is the score that was evaluated for the option bought, and the product that `-march=native` could contract with its sum is gone.
- Not one helper for the seven sites: at `static.hpp:77` it would compute the costs, which can throw, on a path that returns at `:86` without them.
- I would leave it unless the owner wants the simpler arithmetic: it is a declared difference for about ten lines.

## 7. What an experiment costs

### 7.1 A switch is cheap; a variant of a step has been a copy

- **A switch of one algorithm** touches its member with its default, its option in the interface, `parameters.md` twice (the table of who takes what, and its own), for a Benders `benders.md` twice, and, if one thinks of it, `capture.py` and `check.py`. The listing, the JSON and the benchmark follow by themselves. `--no-stored-arcs` went into the two matrices; `--root-cut-tolerance` and `--root-cut-gap` did not, so their other values run in no check (§4).
- **Parameters declared more than once**: `big_Ms` four times (A4), `evict_age` in `path_lp` and `tree_lp`, `decremental` and `only_dec` in `greedy` and `static`, `indicator_constraints` in `flow_mip` and `flow_benders`, each with its option written again in the interface.
- **An experiment that adds to a step** has been made in place behind a parameter: the box of `path_lp` (tag `archive/path-rr-boxstep`, 151 lines), the gain cuts of `multiflow_benders` (140f91b, 149 lines, removed by 8597aab), the smoothing (tag `archive/path-rr-smoothing`, 86 lines in the solver and a helper of 116). [once]
- **An experiment that varies the structure of a step** has been made as a second algorithm or a second copy of the step: [once]

  | Experiment | Its cost | Of which copied |
  |---|---|---|
  | `multicut-root-cuts` (unmerged): the rounds and a fractional cut for `multicut_benders` | 306 lines | 116 to 136: the label search and the two walks of a push of `_largest_flow` (67), the skeleton of the rounds (41 to 46), the stats |
  | `tree_rr_cm` (30a19eb): another column management for the tree relaxation | 290 lines, an interface, a unit | 94 of 211 code lines; the next experiment was then written in both |
  | `unprep_target_benders`, `naive_benders`, `mip` (before db8899d and f2d69a0) | 237, 319 and 162 lines, each with an interface and a unit | 170 of 172, 207 of 243, 59 of 103 code lines |

  The owner then folded the last three into switches (`--preprocessing none`, `--groups`). The copy of `multicut-root-cuts` has already drifted: the rounds of main gained the floor, the violation and the gap since.
- **Where a seam would have paid**: the rounds on the relaxation (A1 keeps them one step away from a function that takes a lambda), the pricing of `path_lp` (A2), and the cut of a target (A8). The flow search does not make the list: sharing it between `multiflow_benders` and the branch would have saved 67 lines by a change in the hot path (§9).

### 7.2 The constants that no parameter reaches

Guards keep a search valid or finite; tunables are choices that an experiment could move. *Recorded* says where a measurement of the value can be found. [once]

| Constant | Where | Kind | Recorded |
|---|---|---|---|
| The pricing margin 1e-7 σ + 1e-6 | `column_generation.hpp:19` | tunable | no measurement; validity only (`path_lp.md:171`). Absolute in the unit of the solver |
| The eviction threshold 1e-6 | `column_generation.hpp:34` | tunable | nowhere |
| The tolerance 1e-8 and the plain big-Ms of the pricing of `tree_lp` | `tree_lp.hpp:70`, `:75` | tunable | nowhere; it does not follow `-t`, which the master does |
| The barrier for the master of `tree_lp` when nothing is evicted | `tree_lp.hpp:290-292` | tunable | the body of c68c0b9 (aude at 8: 41 to 17 rounds). An experiment on `--evict-age` compares two things at once: eviction with the simplex against no eviction with the barrier |
| `root_cut_abs_tol` 1e-7, `path_tie` 1e-9 | `multiflow_benders.hpp:56`, `:75` | tunable | a rationale in their comments; no measurement |
| Half of the time limit for the rounds | `multiflow_benders.hpp:598` | tunable | no measurement; two notes record runs that ended their rounds on it |
| The in-out step 1/2 | `multiflow_benders.hpp:647` | tunable | on against off in the body of fb1f619 (Aix 70 × 70 at 8: 4.2 s against 15.2 s); the step never varied |
| `mip_gap` 1e-10 | `solve_result.hpp:14` | tunable | nowhere |
| `num_groups` 2, a parameter | `multicut_benders.hpp:33` | default | the body of edff7c0, before the benchmark. On the suite, the note of the unmerged branch has 17 runs solved of 42 with 2 groups and 20 with `--groups 0` |
| `arc_full` 1e-11, `no_flow` 1e-12, the factor 1 − 1e-12 | `multiflow_benders.hpp:78`, `:224` | guard | nowhere for the first two |
| `max_pushes` 100 000 | `multiflow_benders.hpp:81` | guard | `multiflow_benders_separation.md:25`: no search gave up in 8 million separations of a source |
| Gurobi's `OptimalityTol` 1e-9 | `mip_solver_base.hpp:312` | guard | its comment: one observation at the default |
| `tie_window` 1e-12 | `heuristics_common.hpp:55` | guard | `greedy.hpp:77-78`, over 807 steps of the short suite |
| The grain 64 of the sums | `parallel_pc_num.hpp`, `split_lagrange.hpp:449` | guard | it fixes the order of a sum: another value changes the last bits of every score |

- If one of the constants of `multiflow_benders` becomes a parameter: `arc_full` is read per arc scanned in the label search, between stores of doubles, so it stays a compile-time value or is copied to a local, as `negligible` is at `multiflow_benders.hpp:303`. `root_cut_abs_tol`, `path_tie` and `max_pushes` are read outside that loop. A default of the in-out step must keep the bits of `(center[i] + point[i]) / 2`.
- Two entries are stale where they stand: the `1e-8` given to the contraction at `tree_lp.hpp:75` is never read (the tolerance serves the tight big-Ms only), and the comment at `mip_solver_base.hpp:660` says that `OptimalityTol` is set in `_solve`, where `_set_search` sets it.

### 7.3 Where the defaults are justified

`benchmark/README.md` holds no result and `benchmark/results` is not tracked. What backs a default today is in four kinds of places: [once]

| Recorded in | Defaults |
|---|---|
| A design note, on the benchmark suite | `multiflow_benders`: `--big-m knapsack_bnb` and the stored arcs; `--numeric-focus` off |
| A design note, on the older instances | `--nb-mus 0` and `--mus-sweep decades`; `path_lp`: the search pricing, no eviction |
| A docs page, one instance | `--root-cut-tolerance` and `--root-cut-gap`; the preprocessing switches; `--decremental` |
| The body of a commit of main only | `multicut_benders`: 2 groups and `voronoi`; `multiflow_benders`: the rounds and in-out; the barrier of `tree_lp`; `--mip-start-algorithm none`, against its own numbers (57e13f3) |
| Nowhere | `-t 1e-7`; `--resolution 1e-8`; `--big-m knapsack_bnb` in time for `flow_mip`, `flow_benders` and `multicut_benders`; `--indicator-constraints` off; `--relative-gap 0`; the `--evict-age 0` of `tree_lp` |

One recorded number goes against its default: `path_lp --max-activations 5000` took 3.69 s against 5.44 s and 15.75 s against 28.82 s (`path_lp.md:252`), with the note that the cap has no rule.

- **E1. One tracked table of the defaults**, in `benchmark/` or in the docs: the parameter, its default, the values tried, the suite, the date, the result and where its runs are. The measurements exist; they are in commit bodies, in an unmerged branch and in an untracked directory. This is outside the code, and it is what the experiments are for.

## 8. Suggested order

1. S0, the matrix, and a capture of main.
2. A6, the small lifts, and A3, the heuristics: the smallest diffs, checked by the widened matrix.
3. A1, `multiflow_benders`, with its unit test and the suite before and after.
4. A4, the base, then A5 and A7.
5. A8 and A9, the cut and the graph of a target, in one build: they change the same struct.
6. A2, `path_lp`, once the owner has ruled on its design.
7. B1 to B3 as ruled.

## 9. Removed by the verification, and not proposed

- **An object of the base for the preprocessing of a case** (the labels, the debug line, the bar, the contraction, the sizes, the row). I had drafted it as the first proposal. Only the labels call is common to the five sites: the debug line has three texts and none, the bar counts vertices, targets or does not exist, the contraction is three functions with or without an options model, `flow_mip` walks every vertex, `multicut_benders` takes its own order and its own row, and `tree_lp` contracts inside the constructor of its pricing model. It would save about 8 lines in three solvers for an object of several dozen, and a smaller form was already tried and dropped in 543ad9f. The scaffold of these files is 8 to 12 lines: there is little left to lift.
- **One flow search for `multiflow_benders` and the branch `multicut-root-cuts`.** The two differ in the sources pushed and their order, the supply left, the stop rule, the counter, the fallback and the value returned: a parameterised loop for two. The label search and the two walks of a push could be shared, 67 lines, by turning a closure of `_largest_flow` into a function template, in the hot path, for a branch that is not merged.
- **A shared loop for `path_lp` and `tree_lp`.** I had written that their rewrite on the column manager reopened the question: both were already on it on 2026-11-14, and the two loops have moved further apart since. Of the nine pieces that I listed as the same, two are identical, four are the same with a parameter, and three are not close [once]: the activation (`path_lp` purges its pool and has a selector), the stop block, the stats. A driver would need about seven hooks, four of them empty for one algorithm.
- **One helper for "the options of a solution applied to a case".** Six sites; three share one body (A3). `compute_solution_cases_pc_num` (`helper.hpp:236-276`) could join them with its order of summation kept, at the price of rewriting a function that six callers and a `decltype` rely on. `tree_lp.hpp:244-259` sums the gains of a vertex in the order of its entries, not of the options, and its value is the coefficient of a column: it stays. `path_lp.hpp:514-519` applies every option to the probabilities alone.
- **The six sources of `src/solver_interfaces` as one template.** Four are the same six lines; `tree_lp` and `path_lp` have their own shape. Each must stay a unit, and a member template would save about three lines in each of four files.
- **A claim of mine that was wrong.** "Nothing else distinguishes how `Greedy` and `Static` are called": `GreedyInterface` computes the status `interrupted` from the stop flag after the solve, and `Greedy::solve` reads that flag and reports its progress, also as a MIP start.
- **Already recorded, and left**: `path_lp::solve` has no caller (`path_lp.hpp:463-468`); it is the entry point that every other struct offers.

## 10. Found on the way

Not asked, not fixed. [once]

- `tree_lp.hpp:299-300` reads the duals after `model.solve()` without testing the status, where `path_lp.hpp:650` tests it: a master that is not optimal is not handled.
- An infeasible MIP start is silent in `flow_mip` and in the three Benders with lazy constraints: `_add_mip_start` hands it to the solver unchecked, and the Benders seed their cuts at it. It is fatal only if the solver then finds no solution. `_mip_start_solution` skips an unknown option name without a word (`mip_solver_base.hpp:768`).
- `path_lp.hpp:580-584` takes the longest option name when the lambda is created: undefined on an instance without options, which the commands refuse and a caller of the library can pass.
- The decremental steps divide by `instance.option_cost(i)` with no branch for a cost of 0 (`greedy.hpp:121-124`, `static.hpp:63-65`). The reformulation buys such options beforehand; a caller of the library on a raw instance gets an infinite ratio.
- `"Solving the optimization model"` is logged before the rounds in `multiflow_benders` (`multiflow_benders.hpp:594`) and right before the search in the three other MIPs.
- `CONTRIBUTING.md:16` names two helpers that build rows; `add_flow_rows` is a third since ea09ace, with three callers.
- `tree_lp.hpp:18` includes Gurobi's header and uses nothing of it.
- `benchmark/run.py:49` decides whether an algorithm has a time limit by the substring `--time-limit` anywhere in its listing, help texts included.

## 11. Implementation log

2026-11-17. The owner ruled on the note as a whole: "agree with all your points, implement in the suggested order". Read as: S0 and A1 to A9 are done as proposed, the owner's word given for A1 and A8 and for the design of A2; B1 and B3 are not done, since I recommended against them; B2 and the keep-list question of A9 carried no recommendation and stay open; E1 comes last.

- **Baseline.** main d5b126f rebuilt, kept as `test/parity/captures/bin/gecot-d5b126f`. Every step below is built with `make -j2`, without a compiler warning, then checked by the unit tests, by a capture compared with the baseline capture `s0-main-a`, and by the oracle.
- **The checks cannot see** a stop by the time limit or by Ctrl-C: the code of the stops was moved by reading only.

| Step | Commit | What was done | Checks |
|---|---|---|---|
| S0 | b537f63 | 170 cases more in `capture.py`: the three Benders with `--no-lazy-constraints`, `multiflow_benders --solver highs`, the two parameters of the rounds at 1e-3, the three parameters of `path_lp`, `tree_lp --evict-age 2` and with a start, eight refused starts, eight runs at `-vv`. Six configurations more in `check.py` | two captures of main: 1021 cases, 0 differ. The oracle on main: 2715 runs, 0 failures |
| A3 | b7ed7ba | `_buy` and `_remove` on three maps, `_scores` with the change as a lambda, `option_ranking`, `heuristic_start`, `std::erase_if`: 51 lines less over the three files (the note said about 115: the ranking and the opening went into `heuristics_common.hpp`) | 76 tests; 1021 cases, 0 differ; oracle 2715, 0 failures |
| A6 | 6cac66d | `_time_up` and `_set_time_left` with a share of the limit, `_start_defect`, `log_master`, `column_manager` and `reduced_cost_of`, `lowered_big_M_statistics` moved, `root_time_share` named | the same build and checks as A3 |
| A6, the flags | aaf8081 | `pruning`, `lazy_constraints`, `root_cuts`, `in_out`, `stored_arcs`, set by `add_flag_callback` | 1021 cases, 0 differ, the listings and the JSON parameters among them |
| A1 | 215ba22 | `multiflow_target_block<MaxPushes>` in its header (427 lines), `multiflow_benders.hpp` 730 to 351 lines, `solve()` 239 to 150; `_floor_at` and `_root_rounds` as members; the test `multiflow_cuts_are_valid_and_tight` | 77 tests; 1021 cases, 0 differ; oracle 2715, 0 failures; the suite below |
| A4 | b4e0e05 | `mip_solver_base` (776 to 438 lines), `flow_preprocessing` (174) and `benders_base` (259), each in its file; `_benders_search` dispatches to `_callback_search` and `_resolve_search`; one `big_Ms` member, in `flow_preprocessing`; `add_benders_options` and `add_preprocessing_options` take the narrower struct | 77 tests; 1021 cases, 0 differ; oracle 2715, 0 failures |
| A5 | 1db03ec | `tree_lp`: the pricing models in a first loop, the rows of the master in one block after it | 77 tests; 1021 cases, 0 differ |
| A7 | 2c852f7 | `compute_mip_start` in `src/solver_interfaces/abstract_mip_interface.cpp` | 77 tests; 1021 cases, 0 differ; the units below |
| A8 | 71e3705, then the last code commit | `contracted_graph_data::cut`, called by `flow_benders` and by `multicut_benders`, which adds its third term; the two sums through `ordered_sum`, which holds the `static_assert` | 77 tests; 1021 cases, 0 differ; oracle 2715, 0 failures |
| A9.1, A9.2 | 2ac83a7 | the contraction returned by `std::move`; the list form of the improvements built only for the tight big-Ms and for `contracted_graph_data` | the same checks |
| A9.3 to A9.5 | 02c3f64 | `static_flow_graph`, the one renumbering; an optional option per arc in `contracted_graph_data`, which is an aggregate; `compute_tight_big_M_maps` on the probability and the option of each arc | the same checks; the suite and the timings below |
| A2 | c8047dc | `path_lp`: `enumerate_pricer` and `search_pricer`, each with `target_pairs`, `price` and `stat`; `pair_duals`; `_rounded`; `lower_option_bounds` and `end_rounds` as lambdas. `relax()` 488 to 275 lines, its loop 268 to 114 | the same checks; the timings below |
| E1 | 685deb9 | `benchmark/defaults.md` | – |

- **What S0 found about the hash.** With `--no-lazy-constraints` on Gurobi, the model hashed is the last master, with the cuts of the search as rows (146 rows against 37 for `flow_benders` on aude at 7): these cases pin the coefficients of the cuts of `flow_benders` and `multicut_benders`, which the note said could only be read by hand. A capture of `tree_lp` hashes its master; with `--evict-age 2`, where no barrier solves the master last, it hashes a pricing model.
- **A6, a mistake caught before the checks.** The member `root_cuts` was shadowed by the counter of the same name in `solve()`, which would have switched the rounds off. `-Wshadow` is on: the build would have warned.
- **A1, the unit test.** For each vertex of three small instances, at each scenario and at two fractional points, with the sets A_s kept and rebuilt: the cut is at least the flow in every scenario; at a scenario its value is the flow and the cut is tight. With `MaxPushes` 0 and 1 the fallback is taken, and the same holds. With the terms of the improved copies removed from the cut, the test fails 2508 times.
- **A1, the suite.** `multiflow_benders` on the short suite at 30 s, main against A1: 31 runs solved on both sides, the 31 with the same rounds, cuts, lazy constraints, nodes and scores; 206.8 s against 208.3 s over them, within the 5 % that the suite varies by.
- **A5 keeps the session of the master first.** The LP is created and set before the pricing models, as before: created after them, a missing LP library would have been reported only once the preprocessing is done.
- **A7, measured.** One unit compiled alone, before and after: `flow_mip` 65.0 s and 4.11 GB, then 62.6 s and 3.99 GB; `multiflow_benders` 56.4 s and 3.19 GB, then 54.5 s and 3.03 GB; `path_lp` 34.7 s and 1.99 GB, then 33.0 s and 1.89 GB. The new unit takes 13.6 s and 1.34 GB. So 3 to 5 % of the peak memory of a unit, and no time: less than the note hoped.
- **A8, the assert holds.** `views::concat` of the two halves of the cut is a range on GCC 15.1, so mippp keeps the written order: the reading of the header that §5 marks [once] is now compiled. The captures with `--no-lazy-constraints` hash the cuts as rows, with their coefficients.
- **A8, a mistake of mine.** Its first capture ran while the build was still linking, on the binary of A7. The capture's metadata gave it away; it was redone, and since then the hash that a capture records is compared with the binary's before the comparison.
- **A9, how it differs from §5.** No struct is shared: `static_flow_graph` returns t, the graph and its four maps, and `contracted_graph_data` and `multiflow_target_block` each hold them, so `_largest_flow` reads the same members as before. `contracted_graph_data` holds the big-M of each improvement as an arc map and the big-M of t, in place of the vertex map that gave both: the same values.
- **A9, the commented block** of `compute_contracted_graph.hpp` is untouched. It returns the form with a list of options per arc, which no longer exists; it did not compile before either. Whether it stays is the owner's.
- **A9, measured.** `multiflow_benders` on the suite, main against A9: 31 runs solved on both sides, identical in rounds, cuts, lazy constraints, nodes and scores; 206.8 s against 208.3 s. The seconds before the solve on `urban-greening-large` at 4000, read in the log to 0.1 s, three runs each, main then A9: `flow_mip` 1.8 to 1.9, then 1.8; `flow_benders` 1.4, then 1.3; `multicut_benders` 1.6, then 1.4; `multiflow_benders` 1.1, then 0.9 to 1.0.
- **A2, measured.** Single-threaded, three runs each, main then A2: `bike-town` at 50, 192 rounds and 17 587 searches, 5.09 to 5.15 s, then 5.07 to 5.13 s; Quebec at 1000, 14 rounds and 19 451 searches, 1.86 to 1.87 s both; `--pricing enumerate` on Aix 30 × 30 at 8, 0.83 s both. The same rounds, searches, scores and bounds. The unit of `path_lp` compiles in 31.5 s and 1.83 GB, against 33.0 s and 1.89 GB.
- **Sizes.** The long functions and files are shorter: `multiflow_benders.hpp` 733 to 348 lines, `mip_solver_base.hpp` 776 to 438, `relax()` 488 to 275, the `solve()` of `multiflow_benders` 239 to 149, of `flow_benders` 165 to 135, of `multicut_benders` 174 to 146. The code as a whole is not: the solvers, the preprocessing and the interfaces go from 7187 to 7307 lines, with four new files and the structs of `path_lp` (961 to 1018 lines).
- **A review of what the checks cannot see**, by three readers on the diff d5b126f to c8047dc, each with a slice (the stops, the time limits and the searches; `path_lp` and the heuristics; the cuts and the graphs of a target), none of them building or running. They compared the old and the new statements path by path, and found no difference on a default build with a valid instance. What they found elsewhere, left as it is:
  - `tree_lp` logs the table of the preprocessing before it builds the rows of the master: if building them throws, the table is in the log where it was not.
  - `path_lp` below `-vv` no longer takes the longest option name, which was undefined on an instance without options (§10): a caller of the library with such an instance now gets past that line.
  - An improved probability written `-0.0`, which the parser accepts, was turned into `+0.0` by a `std::max` that the list form implied, in `flow_and_potentials` and in the tight big-Ms; it now stays `-0.0`, and gives zero coefficients of the other sign. A negative or NaN one, which only a caller of the library can pass, was clamped to 0 there and is not.
  - With `OPTIMIZE_FOR_NATIVE`, the compiler may contract a product and a subtraction into one operation, and the refactor can change where it does: the reduced cost of a column of `path_lp` is the place named. The parity rule is for the default flags; nobody compared two native builds.
  - Checked and equal, among others: every path of the two pricings (bound, column, memo, count, stop); the four ends of the rounds of `path_lp`, status, order, level and text; the two tests of a cut in `_root_rounds`; the cut of a target term by term against its two old spellings, and the association of the sum of `multicut_benders`; the Dijkstra of `flow_and_potentials`; the tight big-Ms site by site; the flags through `add_flag_callback` against CLI11's source.
- **The sum of `multicut_benders`**, the cut of a target plus its third term, was the one sum touched whose order nobody had compiled. `ordered_sum` now holds the assert of A8, and both sums go through it: both compile, so the three parts of that cut reach the solver in the written order.
- **The branch tip, built again from the committed tree**: no warning, 77 tests, 1021 cases with 0 differences against main, 2715 oracle runs with 0 failures.
- **Not done.** B1 and B3, as recommended. B2, which had no recommendation: `tree_lp` still solves again after a stop. The items of §10. The constants of §7.2 are still constants, `MaxPushes` apart.

## Appendix: the sampler

`gdb` runs the program in the background (`run &`); a thread of its Python posts `interrupt` every 0.1 s; the handler of the stop walks the frames of every thread, classifies the stop, and posts `continue &`. `gdb` does not pass the SIGINT of `interrupt` to the program, so the stop flag of gecot is not set. With `ptrace_scope` at 1, `gdb` has to be the parent of the program.

- For `flow_benders`, a stop is in the callback when a thread has frames of gecot above `GRBoptimize`, and in the separation when one of them is `flow_and_potentials` or a Dijkstra.
- For `multiflow_benders`, a stop is in a separation when a thread has a frame of `target_block`; each such thread is then counted by its innermost frame.
- The stops slow the run, by the time `gdb` takes to walk 17 threads: 150 to 300 samples in 30 s. The shares are of the wall-clock time of the slowed run.
- Run as `SAMPLE_OUT=out.txt SAMPLE_PERIOD=0.1 sh -c 'sleep 170 | gdb -q -x sampler.py --args build/gecot optimize flow_benders <instance> <budget> --time-limit 30 -q'`: the pipe keeps the standard input of `gdb` open while the program runs.

The sampler of `flow_benders`:

```python
# gdb wall-clock sampler: stops the program every PERIOD seconds and records,
# for the thread that runs gecot's Benders callback if any, which step it is in.
import gdb, threading, time, collections, os

PERIOD = float(os.environ.get("SAMPLE_PERIOD", "0.25"))
OUT = os.environ["SAMPLE_OUT"]
counts = collections.Counter()
raw = []
state = {"alive": True, "samples": 0}

def classify():
    """one label for the whole process at this instant: the callback if any
    thread runs it, else the build step, else the solver"""
    best = (0, "solver only")
    for thread in gdb.selected_inferior().threads():
        try:
            thread.switch()
            frame = gdb.newest_frame()
        except gdb.error:
            continue
        names = []
        while frame is not None and len(names) < 80:
            try:
                names.append(frame.name() or "?")
                frame = frame.older()
            except gdb.error:
                break
        if not any("gecot::" in n for n in names):
            continue
        # gecot frames above the solver's: a callback
        solver_at = next((k for k, n in enumerate(names) if "GRBoptimize" in n), None)
        above = names[:solver_at] if solver_at is not None else []
        if any("gecot::" in n or "mippp::" in n for n in above):
            if any("flow_and_potentials" in n or "dijkstra" in n.lower() or "_separate" in n or "separate(" in n or "_largest_flow" in n or "_value_and_potentials" in n for n in above):
                label = "callback: separation (flows, potentials)"
            elif any("add_lazy_constraint" in n for n in above):
                label = "callback: building and handing a cut"
            elif any("add_solution" in n or "_hand_back" in n for n in above):
                label = "callback: handing the candidate back"
            else:
                label = "callback: other"
            cand = (3, label)
        elif solver_at is not None:
            cand = (1, "solver only")
        elif any("tbb::" in n for n in names) and not any("solve<" in n for n in names):
            continue
        else:
            cand = (2, "before the search (preprocessing, model, seed cuts)")
        if cand[0] > best[0]:
            best = cand
            if len(raw) < 6 and cand[0] == 3:
                raw.append(" | ".join(n[:70] for n in names[:14]))
    return best[1]

def on_stop(event):
    if isinstance(event, gdb.SignalEvent) and event.stop_signal == "SIGINT":
        try:
            counts[classify()] += 1
            state["samples"] += 1
        except Exception as e:  # keep sampling
            counts["sampler error: " + str(e)[:60]] += 1
    def resume():
        try:
            gdb.execute("continue &")
        except gdb.error as e:
            counts["resume error: " + str(e)[:60]] += 1
    gdb.post_event(resume)

def on_exit(event):
    state["alive"] = False
    with open(OUT, "w") as f:
        total = sum(counts.values())
        f.write("samples %d, period %.2f s\n" % (total, PERIOD))
        for label, n in counts.most_common():
            f.write("%6.1f %%  %5d  %s\n" % (100.0 * n / max(total, 1), n, label))
        f.write("\n--- first stacks\n")
        for s in raw:
            f.write(s + "\n\n")
    gdb.execute("quit")

def ticker():
    while state["alive"]:
        time.sleep(PERIOD)
        if state["alive"]:
            gdb.post_event(lambda: gdb.execute("interrupt"))

gdb.execute("set pagination off")
gdb.execute("set confirm off")
gdb.execute("set print thread-events off")
gdb.events.stop.connect(on_stop)
gdb.events.exited.connect(on_exit)
gdb.execute("run &")
threading.Thread(target=ticker, daemon=True).start()
```

The sampler of `multiflow_benders`:

```python
# wall-clock sampler of multiflow_benders: per sample, whether separation runs,
# and for each thread in it, the innermost frames
import gdb, threading, time, collections, os
PERIOD = float(os.environ.get("SAMPLE_PERIOD", "0.1")); OUT = os.environ["SAMPLE_OUT"]
phase = collections.Counter(); inner = collections.Counter(); state = {"alive": True}
def classify():
    sep_threads = 0; in_solver = False; in_gecot = False
    for thread in gdb.selected_inferior().threads():
        try:
            thread.switch(); frame = gdb.newest_frame()
        except gdb.error:
            continue
        names = []
        while frame is not None and len(names) < 60:
            try:
                names.append(frame.name() or "?"); frame = frame.older()
            except gdb.error:
                break
        if any("target_block" in n and ("separate" in n or "_largest_flow" in n or "_source_arcs" in n) for n in names):
            sep_threads += 1
            top = names[0]
            if any(k in top for k in ("malloc", "free", "memset", "operator new", "operator delete", "_int_", "munmap", "mmap", "brk", "madvise")):
                key = "allocation (malloc/free/memset)"
            elif "_largest_flow" in top or any("_largest_flow" in n for n in names[:4]):
                key = "_largest_flow"
            elif "_source_arcs" in top or any("_source_arcs" in n for n in names[:4]):
                key = "_source_arcs"
            elif "separate" in top:
                key = "separate (own code)"
            else:
                key = "other: " + top[:50]
            inner[key] += 1
        elif any("GRBoptimize" in n for n in names) and any("gecot::" in n for n in names):
            in_solver = True
        elif any("gecot::solvers::multiflow_benders::solve" in n for n in names):
            in_gecot = True
    if sep_threads: phase["separation running (%s)" % ("1 thread" if sep_threads == 1 else "several threads")] += 1
    elif in_solver: phase["solver"] += 1
    else: phase["other gecot code"] += 1
def on_stop(event):
    if isinstance(event, gdb.SignalEvent) and event.stop_signal == "SIGINT":
        try: classify()
        except Exception as e: phase["sampler error " + str(e)[:40]] += 1
    gdb.post_event(lambda: gdb.execute("continue &"))
def on_exit(event):
    state["alive"] = False
    with open(OUT, "w") as f:
        t = sum(phase.values()); f.write("samples %d\n" % t)
        for k, n in phase.most_common(): f.write("%6.1f %% %5d %s\n" % (100.0*n/max(t,1), n, k))
        t = sum(inner.values()); f.write("thread-samples inside separation %d\n" % t)
        for k, n in inner.most_common(12): f.write("%6.1f %% %5d %s\n" % (100.0*n/max(t,1), n, k))
    gdb.execute("quit")
def ticker():
    while state["alive"]:
        time.sleep(PERIOD)
        if state["alive"]: gdb.post_event(lambda: gdb.execute("interrupt"))
gdb.execute("set pagination off"); gdb.execute("set confirm off"); gdb.execute("set print thread-events off")
gdb.events.stop.connect(on_stop); gdb.events.exited.connect(on_exit)
gdb.execute("run &"); threading.Thread(target=ticker, daemon=True).start()
```
