# Maintainability review: proposals to rule on

This note reviews the code of gecot at main fd12a84 (2026-11-14), once the problem formulation is considered frozen, and proposes refactorings and design changes for maintainability. It is written for the owner, who rules on each proposal. The rulings are recorded where they were given, and what was implemented from them in the Implementation log at the end.

- **Aims set by the owner.**
  - The command line stays the privileged target.
  - The door stays open for Python and R integrations, not yet investigated.
  - The mathematical formulation must be clear from self-documenting code, with comments held to one rule: a comment earns its place only if it names a mistake that the code alone would not prevent.
- **Rulings given for this review (2026-11-14).**
  - `docs/reference/notation.md`, the notation of the thesis and of the article, is canonical: the code follows it.
  - Internal changes, plus an additive output contract: valid JSON, a version field, listings as JSON. No renames of commands, parameters, JSON keys or statuses, and the logs stay as they are.
- **Sources.**
  - Ten read-only inventories of the repository: a notation concordance per formulation, every comment of `include/` and `src/` classified, the boundary between the library and the command line, the solver plumbing, the tests and tooling, and the toolchain facts for Python and R.
  - My own reading of `mip_solver_base.hpp`, `helper.hpp`, `concepts/instance.hpp`, the flow and Benders solvers, the abstract interfaces, `optimize.cpp`, `bound.cpp`, `common.cpp`, `trivial_reformulate.hpp` and `instance.hpp`.
  - Five skeptics, each asked to refute one cluster of my draft proposals against the code and against the sources of mippp, melon and CLI11 in the conan cache, and one architect asked to criticize the whole set. Their corrections are folded in, and §10 lists what they removed.
- **Evidence marks.**
  - **[read]** read in the code by an agent of the inventories. This is the default. What a proposal or a refusal rests on was read a second time, by a skeptic or by me; every line cited was then matched against the file by a script, on a fragment of its text.
  - **[once]** read by one agent only, and not by me.
  - **[gecot]** run with `build/gecot` of 2026-11-14 02:25, built from the same tree as fd12a84 (`git diff 1f87084 fd12a84` is empty). Nothing was rebuilt.
  - **[web]** an external fact about toolchains, read on 2026-11-14 by one agent and not checked again. Sources are in Appendix D.
- **Vocabulary.** *Parity* is the rule that a refactor must respect: with one cost and no conflict set, the results are bitwise identical, and a MIP or LP model receives the same sequence of variables, rows and columns with the same coefficients, since the path of a solver depends on them.

## Rulings of 2026-11-14 on the decisions of §2

Given by the owner after reading this note. Where a ruling contradicts a section below, the section is superseded and says so here.

1. **The program is stated by the code.** The model calls (`add_variables`, `add_constraints`, `add_column`) stay inline in `solve()` and form dense blocks, as in `flow_mip` and `flow_benders` today. The boilerplate is factored out of their way and reduced to a minimum. A block is preceded by a comment that names the family of constraints it builds and refers to the documentation. Consequences:
   - **Stage 2 changes shape.** A pass sorts its file into model calls, which stay visible, and what is not the model, which leaves `solve()`: the statistics, the preprocessing rows and their log, the progress bars, the handling of the start, the readback. No function builds rows. This supersedes "`solve()` in blocks" and the header block of §5.1, and the "program as built" of §5 everywhere.
   - **`flow_mip` and `flow_benders` are the reference style**, not the files to bring to the style of the others. Their names still change (§4), and the inline copy of the contraction still goes (§5.1).
   - **The docs carry what the code builds.** `flow.md` gets the variable v_t and its two rows; each family of constraints gets a stable anchor that a block comment can cite.
2. **Options are indexed by `i`**, in the code as in the docs. The other uses of `i` are renamed in the pass of each file.
3. **The indicator constraints become an option**, `--indicator-constraints`, a flag of `flow_mip` and `flow_benders`. It replaces their big-M rows by indicator constraints, chosen by `if constexpr`. On a solver without them (mippp has them for Gurobi and CPLEX) the run stops with an error, as `--lp-relaxation` does. The commented name lambdas and the older big-M block of `flow_mip.hpp` are removed.
4. **Names correspond to those of the command line.** Files, structs and interfaces of `path_lp` and `tree_lp` take these names. `static` is a keyword, so `Static` and `Greedy` keep their capital. Each rename is its own commit, checked by the harness: a renamed class changes the type names stored in the binary, so the object files cannot be compared.
5. **No formulation block remains**, so the formatter has nothing to reflow and `.clang-format` stays as it is. A header block is removed once its docs page says the same. Two say more than the docs today and move there first: the tight big-M (34 lines in `compute_tight_big_M_maps.hpp:23-56` against one sentence of `preprocessing.md`) and the pricing of `path_lp`.
6. **The fixes of §9 and the default tolerance.**
   - **§9.1, the restore of the qualities:** fixed if the cost is negligible; the owner recalls the subtraction as a performance choice. Restoring by assignment is the same loop over the vertices of the option, so the cost is to be measured, not assumed.
   - **§9.2, "feasible":** both definitions stay. The check of the command gets a name and `results.md` says what the field means.
   - **§9.3, the criterion:** it is simplified in the reformulation, and a case that the simplified criterion does not read is removed. The results then map the cases by name, not by position. This goes beyond the admission test of `all_options`: it is a new step of the reformulation.
   - **§9.4, the start file of `--reformulation path_lp`:** honored. With a file, greedy is not run.
   - **§9.5, the log lines:** "Removed … species" stays; it anticipates the removal of the cases above and becomes true with it. The label of `tree_lp` was not ruled and stays.
   - **§9.6, NumericFocus:** measured first, on the exact algorithms of the short benchmark suite, with and without.
   - **§9.7 and §9.8:** the three wordings of the docs and the statuses of `benchmark/compare.py` are corrected, as fixes with one sensible answer.
   - **F12, the default of `-t`:** one named default, 1e-7, for the structs and the command line. The unit tests that relied on 0.0 state their tolerance; their expected selections must not change.
7. **The unused instances are removed** (2a5967e): `Aix/N1`, `Aix/N2` and their two descriptors, `Aix/instance_70x70`, `Aix/instances.zip`, and the tracked files of `BioRevAix`. 31 files, 33.2 MB; `test/instances` now tracks 1.25 MB. Kept: `map.tiff` and the two scripts that read it, which can regenerate `instance_70x70`; `instance_40x40`, which the docs cite; `worst`, which nothing reads and which no longer parses, left because the decision did not name it. The 63 tests pass without the removed files.

**What is implemented first:** Stage 0, the docs of Stage 1, then the pass of `flow_mip`, with a report before the other files. The implementation log is at the end of this note.

## 1. Summary

- **The solvers belong to two generations.** `multicut_benders`, `multiflow_benders` and `path_lp` open with their program and name their variables after it (`x`, `z`, `theta`, `rho`). `flow_mip` and `flow_benders` state nothing and build `C_vars`, `X_vars`, `Xi_vars`, `F_var`, `Phi_t_vars`. The reference formulation of the tool is the one whose code is furthest from it.
- **The header blocks contradict the canonical notation, and each other.** q is the target quality in the docs, the source quality in the block of `multiflow_benders`, and both in one formula of `multicut_benders`. π is a probability in the docs and a dual in `path_lp` and `tree_lp`, where the code calls it `pi`. φ is a flow and the criterion in one block. Options are i in the docs and o in every header, the budget β against B. Appendix C has the table.
- **Formulation pieces are written several times, without a warning.** `flow_mip` repeats inline what `compute_contracted_graph_data` does. The dual flow is written in `flow_benders` and in `multicut_benders`. The re-injection of a cut candidate, with its three-line comment, is copied in the three Benders. `blocked`, `tier` and `key` are in `greedy` and in `static`. "The option fits alone" has five spellings, two of them with a tolerance under the name of the three without.
- **One name, five roles.** `feasibility_tol` is the solver's tolerance, the relative test of a cut, the absolute slack of the budget, the slack of the tight big-Ms and the rounding threshold of the relaxation. Bare constants repeat next to it: the MIP gap `1e-10` five times, the readback `> 0.5` eight times.
- **Defaults are declared twice.** CLI11 writes the default of an option into the member it is bound to, so the literal of the command line wins: `feasibility_tol` is `0.0` in the three structs and `1e-7` from every command. A caller of the library gets `0.0`.
- **The command layer holds what any front end needs.** `optimize.cpp` computes the raw score, the costs, the feasibility verdict and the best-bound rule between the run and three writers, and `bound.cpp` repeats half of the file. The JSON is printed by hand: strings are not escaped and nothing gives its version.
- **Nothing checks a refactor of the solver-backed algorithms.** The 63 unit tests cover the heuristics, the bounds, the preprocessing, the parser and the log tables: none runs a MIP, a Benders decomposition or a relaxation, and none touches `src/commands`. `test/cli_parity.sh` never fails and several of its cases test nothing. No dependency revision is pinned.
- **Comments.** Of 71 comments that the inventories listed for the cut, I would cut 33, rewrite 18, replace 3 by structure and keep 17 (Appendix B). The traps, about 90, are the valuable part, and a few of them can be carried by structure (§5.4).
- **Python and R.** A wrapper around the executable needs a valid, versioned JSON, and binaries: only v1.0 is released. An in-process binding needs libstdc++ 15 with C++26 where it is built, which CRAN's Windows toolchain and the manylinux images do not have [web]. The code-side cost of that door is small and can wait for a target.

## 2. Decisions needed first

Each changes what the stages below do.

1. **Where the program is stated.** The comment rule has no class for a formulation block. I treated such blocks as legitimate, which is not ruled. The evidence cuts both ways:
   - the blocks of `multicut_benders.hpp:29-53` and `path_formulation_lp.hpp:41-87` are the clearest statements in the code;
   - both are stale: `multicut_benders.hpp:48` still shows one budget row and no conflict row while `:201` calls `_add_budget_rows`, and `path_formulation_lp.hpp:71` gives a reduced cost without the dimensions and the conflict duals that `:878-889` subtract;
   - clang-format broke three of them (`multiflow_benders.hpp:47-48`, `path_formulation_lp.hpp:44-45` and `:758-760`).

   Two forms are possible, and the passes of §5 work under both:
   - **(a)** each header states its program *as built*, in build order, in the canonical notation, with a link to the docs page for the derivation. §5.1 gives the block that `flow_mip` would get;
   - **(b)** names and structure carry the program, and the header only links to the docs page.

   I recommend (a) with decision 5: the as-built program differs from the docs in ways the docs do not show (`flow_mip` has a variable per target that `flow.md` does not), and a link cannot say that.
2. **The index of an option in the code.** The docs write i. The newer code writes `o`; the older code already writes `i` (`flow_mip.hpp:54`, `:290`, `flow_benders.hpp:90`, the constrained labelling). `i` is also an improvement index at `multicut_benders.hpp:270` and a position at `:141`. Either `i` everywhere, the other uses renamed in the pass of each file, or `o` kept under formulas that say i with one line in notation.md. I recommend `i`.
3. **The commented switches of `flow_mip` and `flow_benders`.** 41 comment lines of `flow_mip.hpp`, plus the variable `case_id` at `:73` that only they use, and `flow_benders.hpp:149-150`:
   - name lambdas for the variables (`:46`, `:55`, `:83-86`, `:106-107`, `:165-167`, `:191-195`). `:55` names x `"Y_"`;
   - two `//* … /*/ … //*/` switches to indicator constraints (`:170-182`, `:240-262`), which exist in mippp for Gurobi and CPLEX only, so the switched form cannot compile on seven of the nine backends that `flow_mip` is instantiated for;
   - an older arc big-M block (`:228-238`).

   The keep-list of 2026-10 names the commented contraction block of `compute_contracted_graph.hpp`, not these. Experimental code to keep, or dead code: the pass of `flow_mip` needs the answer.
4. **File and struct names against the names of the command line.** `path_formulation_lp.hpp`, `path_formulation_lp`, `PathFormulationLPInterface` and `path_lp`; the same for `tree_lp`; `Greedy` and `Static` in CamelCase among snake_case structs (`static` is a keyword); `Instance`, the interfaces and `option_values` in the global namespace. To decide once, before the split of `path_lp` adds two names.
5. **clang-format and the formulation blocks.** `.clang-format` has no `ReflowComments` entry, so `make format` reflows them. Either `ReflowComments: false`, after checking that `make check-format` stays clean, or blocks kept under 80 columns.
6. **Each fix of §9**, since it changes bits, a verdict or a log line, and the single default tolerance of F12 (§6), which moves `Static{}` and `Greedy{}` off 0.0.
7. **The unused test data.** `test/instances` weighs 34.5 MB, of which about 33 MB is read by no test (`Aix/N1` alone is 23.8 MB), and three descriptors are in the 1.0 format that the schema now rejects. Removing them does not shrink a clone without a history rewrite.

## 3. Stage 0: safety net

Every later claim of "behaviour-preserving" is judged by this stage.

### S0.1 A parity harness in the repository

- **What exists.** [read]
  - `test/cli_parity.sh` compares two binaries by hand. It exits with 2 on bad usage and with 0 otherwise: no difference affects its status.
  - Its filter drops `Computation time` and `ran in` lines that no longer exist (`:45`) and not the `Time` line that `optimize.cpp:333-336` prints, so a run without `-o` differs whenever its time does.
  - Without `OLD_PREFIX=optimize NEW_PREFIX=optimize`, both binaries reject every case with the same "unknown command" error and every row reads "same". A run killed by its `timeout 10` reads "same" too.
  - Its cases G01 to G08 assume option abbreviations and E14, E34 and X34 pass `-m`: CLI11 supports neither, so they fail on both sides.
  - It has no `bound` case, no `path_lp` in its algorithm loop, no `--reformulation`, no instance with conflict sets, no `--max-threads 1`, and it compares neither stderr nor the JSON of any MIP, Benders or LP run.
  - `test/brute_force/check.py` is an oracle on small instances, to 1e-6, run by hand.
- **Proposal.** Two scripts, about 200 lines, replacing the stale cases:
  - `capture`: runs a matrix on one binary and stores, per case, stdout (a run without `-o`), the JSON (a run with `-o`) and stderr at `-v`, all with `--max-threads 1`;
  - `compare`: compares two captures and exits non-zero on a difference, after dropping the `*_time_ms` fields, the `Time` line, the `[   0.0s]` prefix, the rows of the progress tables (they are sampled on the wall clock, `common.cpp:289-296`), the "Ctrl-C stops" line and absolute paths.
  - The matrix: the instances `test`, `three-woods`, `two-species`, `Aude`, `Aix/instance_30x30` and `dks-30` at budgets 3 and 6 only (the exact algorithms time out at 9); every algorithm and every bound with their main parameters; `--help`, `-A`, `-M` and `-P` of every algorithm and method, which need no solver and pin the parameters.
  - The fields `variables`, `constraints`, `nonzeros`, `nodes`, `lazy_constraints`, `iterations` and `searches` of the report serve as fingerprints of the models.
  - A capture records the machine, the solver versions and the revisions of melon and mippp: the conan cache holds two revisions of `melon/1.0.0` whose `knapsack_bnb.hpp` differ [once], and no lockfile is tracked.
  - The first capture is made from the current build of main, before anything moves.
- **Limits.** It proves equal outputs and equal model sizes, not an equal order of model calls: mippp has no model writer. (Lifted for Gurobi since: a `gurobi.env` in the directory of a run makes Gurobi write its model, see the Implementation log.) `--max-threads 1` makes gecot's own code single-threaded, not the solver. `path_lp` is deterministic single-threaded only: on 2026-11-14, 54 `path_lp` cases of main matched themselves and the branch bit for bit that way [gecot].

### S0.2 The parity definition written down

Default release flags, without `OPTIMIZE_FOR_NATIVE`; `--max-threads 1`; the same machine, compiler, solver and dependency revisions. The compiler counts: the term order of §5.2 is a property of GCC 15.1 that 15.2 removes. With `-march=native` GCC may contract a multiply-add, and the contraction follows inlining, so a header move could change bits there: not verified.

### S0.3 Three static_asserts on the backends

`AbstractMipInterface::call` (`abstract_mip_interface.hpp:210-217`) turns an unsatisfied requirement into a runtime "cannot use solver" error. The Benders lambdas are constrained by `lazy_constraint_model` (`flow_benders_interface.cpp:7` and its two siblings) and the list of solvers offered is written apart (`abstract_mip_interface.hpp:71-72`). If a mippp release dropped a capability, the build would stay green and the command would fail. Three lines: gurobi, cplex and copt satisfy the concept.

### S0.4 Optional: continuous integration

- **What exists.** One workflow, the docs deployment.
- **Proposal.** A job that builds `gecot` and `gecot_test` and runs the 63 tests. Neither needs a solver installed: mippp loads solvers at run time. Building only `gecot_test` would compile none of the solver templates, which are instantiated in `src/solver_interfaces/*.cpp` alone.
- **Obstacles.** mippp 1.1.0 comes from a branch, so the job must create it from a pinned commit; whether melon 1.0.0 is on Conan Center is asserted by `installation.md` only; a GCC 15 toolchain; 2 to 2.5 GB per MIP translation unit; onetbb, spdlog and gtest built from source, so a cache.

## 4. Stage 1: notation

### F1 notation.md completed, with a short "in the code" list

- **Missing symbols.** η_r and ν_l (the duals of the budget and conflict rows, used by `relaxations.md`), C_k, r_0, Y_t and Y_g, x̄, the per-target variable of `flow_mip` (v_t exists for the Benders), and (·)⁺.
- **The list.** One table in notation.md, not a third place:

  | Symbol | Identifier |
  |---|---|
  | x_i, z_k, v_t | `x(i)`, `z(k)`, `v(t)` |
  | f_t, f^i_t, φ^t_a | `f_t`, `f_ti`, `phi(a)` |
  | M^t_u | `big_M[u]` |
  | ρ_u, σ, θ_g | `rho[u]`, `sigma`, `theta` |
  | γ_k, σ_p, λ_pi, η_r, ν_l | `gamma`, `sigma`, `lambda`, `eta`, `nu` |
  | w_u, q_u, w^i_u, q^i_u | `source_quality`, `target_quality`, `source_gain`, `target_gain` |
  | π_a, π^i_a, Π(s,t) | `probability`, `improved_probability`, `path_probability` |
  | c_{i,r}, β_r, Ω_l | `option_cost(i, r)`, `budget[r]`, `conflict_sets()[l]` |

- **Names that the canonical symbols now forbid.** `w` for the head of an arc (`multicut_benders.hpp:274`, `multiflow_benders.hpp:138`, `:160`, `:246`); `q` for a probability (`multiflow_benders.hpp:161`); `pi` for the contribution dual (`path_formulation_lp.hpp:711`, and `tree_lp`); `Phi` for flows where Φ is the set of options; `mus` and `lambda` in the tight big-M, where the sweep is over θ and `lambda` is the μ of the note (`compute_tight_big_M_maps.hpp:113-114`, `:154`).
- **Size and risk.** Docs only.

## Rulings of 2026-11-14, after the passes of Stage 2

Given by the owner once the passes of §5 and the pass of `multiflow_benders` were reported.

- **`multiflow_benders`:** the rounds numbered from 1 and the commit da6b556 (the search of a source apart, less memory, about 5 % more time) are accepted. The leads for its separation are kept in `misc/design/multiflow_benders_separation.md`.
- **Stage 3 (§6):** L2, L4 and F12 are accepted, and the five fixes of "D-safe". Not the parser (PR) nor the static library (L1).
- **Stage 4 (§7):** O1 and O2 are accepted. Not O3, O4 nor O5.
- **Stage 5 (§8):** a status line on the design notes, and the file modes and strays. Not the README; Windows and the releases stay with the owner.
- **Left aside in the passes:** the renames of §5.4 (`useless_vertices_map`, the `*_default_traits`, one spelling for the improved probability) and its three guards (the helper for the free variables, the `static_assert` on `budget_t`, the scale by `std::ldexp`) are to be done. The items of the knapsacks keep `i`, and `lazy_constraint_model` keeps its stronger form.
- **§9.6, NumericFocus:** measured on this machine, reduced: `flow_mip` and `multiflow_benders` on the short suite at 30 s per run, with and without, in a temporary build. The owner decides on the table.

## 5. Stage 2: one pass per file

Each pass treats one file once: the program as built (by decision 1), the identifiers after the symbols, the structure change, the named constants and the comments. Then a parity run. A sweep of comments first, then a sweep of renames, would touch each file four times and make every diff conflict with the next.

### 5.1 P1: flow_mip

- **The program as built**, which the header would state under decision 1(a):

  ```
  // The flow formulation of docs/formulations/flow.md, as built. For each
  // case k and each vertex t that can be a target, on the graph contracted
  // for t:
  //
  //     max  C(z)
  //     s.t. z_k <= Σ_t v_t
  //          v_t <= q_t f_t + Σ_i q^i_t f^i_t
  //          f^i_t <= f_t,   f^i_t <= M^t_t x_i
  //          Σ_{a∈δout(u)} φ_a + [u = t] f_t
  //              <= Σ_{b∈δin(u)} π_b φ_b + w_u + Σ_i w^i_u x_i      for each u
  //          φ_{a^i} <= M^t_u x_i                 for each improved copy a^i
  //          Σ_i c_{i,r} x_i <= β_r,   Σ_{i∈Ω_l} x_i <= 1,   x binary
  //
  // v_t is fixed to 0 for a vertex that can never be a target.
  ```

  `flow.md` has no v_t: the code builds one per vertex (`flow_mip.hpp:80-87`), fixes it to 0 on the vertices that are never targets (`:101`) and bounds z_k by their sum (`:270`). Either the docs page or the block must say so. Removing these variables would change the model.
- **Names.** `C_vars` → `z`, `X_vars` → `x`, `Xi_vars` → `v`, `F_var` → `f_t`, `F_prime_t_var` → `f_ti`, `Phi_t_vars` → `phi`. A rename leaves the model unchanged if each expression keeps its operand order: the rows must not be rewritten in the form of the docs, which moves terms across the inequality.
- **The inline copy of the contraction.** `flow_mip.hpp:112-160` repeats `compute_contracted_graph.hpp:98-173`: the filter of the vertices that need a big-M, the choice between plain and knapsack, the split between base probabilities and improvements, the tight call and its minimum. The data are the same (`[a][0]` and `[a][k]` agree because each arc has at most one improvement there).
  - `flow_mip` cannot call `compute_contracted_graph_data`: its `make_static_digraph` renumbers the vertices in ascending order, where melon's mutable graph iterates in descending order, so the rows and the columns would change order.
  - **Proposal.** Cut `compute_contracted_graph_data` in two stages. The first returns a named struct: the contracted graph before renumbering, the improvements, the big-Ms and the tight minimums. The second is the renumbering. `flow_mip` takes the first only. The struct replaces the 7-tuple of `compute_contracted_generalized_flow_graph.hpp:152-154`. The statistics of the tight big-Ms need both the knapsack value and the tightened one, so the struct keeps both.
  - `tree_formulation_lp.hpp:126-137` is a third copy of the filter. It is left alone (§11).
- **`solve()` in blocks, in the order of the code.** The order of the model calls is, per case: the v variables; then per target: f_t; per gain at t, f^i_t and its two rows; the φ variables; the conservation rows in one `add_constraints` call; the rows of the improved copies; the row of v_t; then the row of z_k. The blocks follow this order, not the order of the docs. The conservation rows stay one batched call: a loop of single rows would be another sequence of solver calls.
- **The OPT macro** (`helper.hpp:25`) has one use, `flow_mip.hpp:201`. Written out as `u == t ? std::make_optional(…) : std::nullopt` it is the same tokens, and the macro leaves the shared header.
- **Size.** About −60 lines in `flow_mip.hpp` with the switches of decision 3 removed, about −10 without.
- **Parity.** Same model calls; harness.
- **Why first.** It has no statement today, and its pass gives the reference for the density of comments that CONTRIBUTING.md will cite.

### 5.2 P2: flow_benders, multicut_benders, multiflow_benders

- **The four prologues, identical in place.** `flow_mip.hpp:41-57`, `flow_benders.hpp:77-92`, `multicut_benders.hpp:191-201` and `multiflow_benders.hpp:355-365` build the same thing in two orders and under two sets of names: tolerances, maximization, z, the objective through `formula_variable_visitor`, x, the budget and conflict rows. No mippp call depends on that order, and the variables come out in the same order. They become the same eight lines, left inline: they are the top of each program.
- **`_add_mip_start` for all.** The three Benders repeat its body (`multicut_benders.hpp:324-328`, `flow_benders.hpp:265-271`, `multiflow_benders.hpp:439-443`) and then seed cuts at the start. The helper returns the start, and each solver keeps the position of the call against its seed cuts.
- **One dual flow.** `flow_benders::_compute_dual_flow` (`:30-66`) and `multicut_benders::_dual_flow` (`:149-179`) run the same reversed Dijkstra. `multicut` multiplies ρ and the value by the target quality. One function returns the flow and the path probabilities; `multicut` scales afterwards, each target before the sum of its group, only the vertices reached. A single product gives the same bits.
- **One re-injection.** The comment "The solver drops a candidate that receives cuts and may never propose its options again" is at `flow_benders.hpp:255-257`, `multicut_benders.hpp:332-334` and `multiflow_benders.hpp:505-507`. A helper holds the step that follows the cuts (x from the rounded candidate, z from the case scores, `formula.set_values`, `add_solution`) and the comment, once. The evaluation before it stays per solver: the three sum the case scores in different orders.
- **What stays per solver.**
  - The cut expressions. `flow_benders.hpp:205-206` and `multicut_benders.hpp:280-281` store the same coefficient. But the terms of `A + B` are joined by mippp's `unordered_concat` (`mippp/detail/concat_view.hpp`), which returns `views::concat(r1, r2)` when that is a range and the swapped order otherwise, because GCC 15.1 rejects some operand orders (GCC bug 120934, fixed in 15.2). The compiler here is 15.1: the order of the terms can depend on the types of the operands, and a shared builder changes those types. It could reorder the nonzeros of every cut.
  - The line `if(!has_solution) return {_fallback_solution(…), report};` and the stat `lazy_constraints`: one line each, and the order of the stats is part of the JSON.
- **Names.** `cai` → `sigma`, `opt` → the value of f_t, `sol` and `master_solution` → `y` and `candidate`, `C_constr` → `z_row`.
- **Statements to correct.**
  - `multicut_benders.hpp:31`, `:48`, `:189-190`: one budget, no conflict row.
  - `multiflow_benders.hpp:60` and `benders.md:140` say the master is the one of `flow_benders`. It also bounds F_t by M_t (`:394-396`), which the docs page omits.
- **Left as they are, and documented.** `flow_benders` creates an f^i_t for a gain of 0 (`:140-151`); dropping it would change the model. `multiflow_benders.hpp:491` tests the root cuts with a literal `1e-7` that does not follow `-t`: it gets a name, not a new value.
- **Size.** About −70 lines over the three files.
- **Parity.** Same numbers and same model calls; harness, with Gurobi.

### 5.3 P3: greedy and static

- **One `fits_alone`.** `helper.hpp:167-172`, `mip_solver_base.hpp:168-172` (`affordable`) and `compute_big_M_map.hpp:100-104` (`fits`) are one predicate. `greedy.hpp:43-49` and `static.hpp:40-46` define lambdas of the same name that add `feasibility_tol`. The helper takes a tolerance, 0 by default: `x + 0.0` compares as `x`, signed zeros and NaN included. `compute_big_M_map` keeps its cached costs, for speed.
- **Shared between the two.** `blocked` (`greedy.hpp:80-87`, `static.hpp:207-215`), `tier` and `key` (`greedy.hpp:132-139`, `static.hpp:192-199`), the refusal of `--decremental`, the first loop, the update of the landscape. The bought options must be passed in their current order: `static` rebuilds the landscape in the order of its ranking and the sums of qualities follow it.
- **Moved.** `compute_options_cases_incr_pc_num` and `…_decr_pc_num` (`helper.hpp:285-419`), used by these two files alone, go next to them. `helper.hpp` is otherwise left in one piece.
- **Kept as they are.** The budget-left tests have three floating-point shapes (`greedy.hpp:225-226`, `static.hpp:139-140`, `static.hpp:219-220`); merging them would change the rounding at the boundary.
- **Comments to correct.**
  - `greedy.hpp:120-121` says the scores are sums whose order changes from run to run. `parallel_pc_num` is deterministic since cd966b7; the remaining source is §9.1.
  - `static.hpp:133` says "the cost spent in each dimension" where the trap is that `spent` starts from `purchased` in dimension 0 only, which is right because `--decremental` refuses several dimensions.
- **Size.** About −60 lines.
- **Parity.** The pins of `solvers_test.cpp:212` and the harness.

### 5.4 P4: preprocessing and bounds

Comments, names and constants only.

- **Names.** The typos `cost_to_penatlty` and `stong_counters`; `useless_vertices_map` and `useless_target_vertices_map` for the vertices that are never targets, where "useless" otherwise means an arc; `strong_arc_default_traits` and `useless_arc_default_traits`, which do not say which colour wins a tie; one spelling for the improved probability (`enhanced_prob`, `improved_prob`, `ip`); the renames of §4.
- **One function for two copies.** The sort of the labels after the parallel pushes, with its comment, is verbatim at `compute_strong_and_useless_arcs.hpp:245-250` and `compute_constrained_strong_and_useless_arcs.hpp:296-301`. `prob_to_length` is at `compute_strong_and_useless_arcs.hpp:137-142` and `compute_constrained_strong_and_useless_arcs.hpp:38-43`.
- **Traps that structure can carry.** A trap comment is replaced only by a static_assert, a test, a throw or a helper: an `assert` pins nothing, every build having `-DNDEBUG`.
  - `mip_solver_base.hpp:68-69`, the free variables written `{}`: a helper named for it, at its three sites.
  - `compute_tight_big_M_maps.hpp:141-142`, "λ = 0 first": `static_assert(mus[0] == 0.0)` with the array sorted.
  - `concepts/instance.hpp:22-23`, `budget_t` without a range interface: `static_assert(!std::ranges::range<budget_t>)`.
  - `trivial_reformulate.hpp:30`, "exactly 1 … s being a power of 2": the scale built from an integer exponent with `std::ldexp`, so the weight is 1 by construction.
- **Statements to correct.** `compute_contracted_generalized_flow_graph.hpp:96`: the guard is reachable only for a strong arc that leaves t, which the pruning removes after the override at `:59`; the labellings never produce one. `qkp_planes.hpp:70-71`: `target` is a score of the criterion, not the weighted score. `compute_contracted_graph.hpp:88-89` says "the graph flow_mip contracts", which `flow_mip` does not call.
- **Parity.** Binary unchanged for the renames; harness.

### 5.5 P5: path_lp and tree_lp

Light, because the split of `path_lp` is the owner's design.

- **Headers and names.** The blocks in the canonical notation (γ_k for the contribution dual, η_r, ν_l, d_i, 𝒞, w, Π, β). The reduced cost of an option as the code computes it: d_i = Σ_p max(0, λ_pi) − Σ_r c_{i,r} η_r − Σ_{l∋i} ν_l, of which only min(0, d_i) is used. `tree_lp`'s single budget and its cast of x stay as written (§11).
- **Two shared definitions.** The pricing margin `1e-7 * sigma + 1e-6` (`path_formulation_lp.hpp:730-732`, `tree_formulation_lp.hpp:379-381`) and the evictor with its warning (`:662-669`, `:335-342`) are verbatim. Nothing else is: `path_lp` reads a missing row as 0, sets a time limit and checks the status of the master; `tree_lp` re-solves after a stop.
- **For the split, recorded and not done.**
  - `relax()` (`path_formulation_lp.hpp:511-998`) returns the rounded solution, the report and the bounds of the options. A variant that rounds otherwise needs the fractional x and the validated start in that result.
  - The start is validated inside `relax()` because `_pairs` uses it.
  - The rounding buys the options of x = 0 in index order while they fit, and if the first solve of the master fails it is a first-fit by index.
  - `path_formulation_lp::solve` (`:503-508`) has no caller.
- **Parity.** Single-threaded harness.

### 5.6 P6: constants and the roles of the tolerance

- **A named constant next to its rationale, in its file.** Shared only where the sites must agree:

  | Value | Sites | Shared |
  |---|---|---|
  | MIP gap `1e-10` | `flow_mip.hpp:42`, `flow_benders.hpp:79`, `multicut_benders.hpp:192`, `multiflow_benders.hpp:356`, `tree_formulation_lp.hpp:95` | yes |
  | bought at `> 0.5` | `flow_mip.hpp:293`, `flow_benders.hpp:225`, `:290`, `multicut_benders.hpp:340`, `:368`, `multiflow_benders.hpp:514`, `:560`, `tree_formulation_lp.hpp:407` | yes |
  | pricing margin, eviction `1e-6` | `path_lp` and `tree_lp` | yes (§5.5) |
  | 2^62, the cap on a path length | `compute_strong_and_useless_arcs.hpp:81`, `compute_constrained_strong_and_useless_arcs.hpp:117` | yes |
  | grain 64 of the deterministic sums | `parallel_pc_num.hpp:27`, `split_lagrange.hpp:456` | yes |
  | tie window `1e-12` | `greedy.hpp:134`, `:154`, `:248`, `static.hpp:194` | in the heuristics |
  | near tie `1e-9` | `multiflow_benders.hpp:187`, `compute_tight_big_M_maps.hpp:233`, `split_lagrange.hpp:388` | no: three independent tests |
  | Gurobi `OptimalityTol` `1e-9` | `mip_solver_base.hpp:242` | — |

- **`feasibility_tol`.** Its role is named at each use: the tolerance of the solver (`flow_benders.hpp:80`), the relative test of a cut (`:246-247`), the slack of the budget (`mip_solver_base.hpp:338`), the slack of the tight big-Ms (`flow_mip.hpp:154`), the threshold of the relaxation (`flow_mip.hpp:292`). `-t` and every expression stay.
- **`_set_tolerances`** names its first parameter `optimality_tol`; it is the MIP gap, and Gurobi's `OptimalityTol` set at `mip_solver_base.hpp:242` is another parameter.
- **Parity.** Values unchanged.

### 5.7 P7: CONTRIBUTING.md

Written at the end of the stage: the comment rule with the answer to decision 1, `flow_mip.hpp` as the reference for density, the parity rule and how to run the harness, the build rule (one build at a time).

## 6. Stage 3: the command layer

Protected by the output captures of the harness.

### L2 One path for the two commands

- **What exists.** [read]
  - `optimize_command` (`optimize.cpp:193-449`) mixes the handling of the arguments, the run (`:228-244`), the evaluation of the result (`:246-302`) and three writers (`:304-442`).
  - `bound_command` has the same shape, and `optimize.cpp:44-178` and `bound.cpp:40-153` are the same skeleton, with the same comments, as are the heads of the two JSON files.
  - No test links any of it.
- **Proposal.** Prepare, run, evaluate into a plain struct, write. The struct carries what the writers print: the status, the selection on the raw instance, the score, the scores of the cases, the costs, the verdict, the two bounds (the scaled one of the stats, and the one shown, clamped to the score when feasible), the share, the stats in their order, the time.
- **Expressions to keep verbatim.** The scores of the cases multiplied by the scale and then the raw criterion (`optimize.cpp:247-252`), where the base score is the scaled criterion multiplied afterwards (`common.cpp:226-228`); the verdict `<= budget[r] + 1e-6 * std::max(1.0, budget[r])` (`:268`); the gap and the clamp (`:294-301`).
- **Size.** About −100 lines of duplication.
- **Parity.** stdout, JSON and CSV byte-identical.

### L1 A static library for what is not argument handling

`gecot_lib` is an INTERFACE target over all of `include/`; `instance.cpp` and `parse_instance.cpp` are compiled into the executable and again into the tests. A static target holds them with the interfaces, the registry, the path reformulation and the evaluation of L2, and the tests link it: the writers and the formatters become testable. It needs the PUBLIC links that only the executable has today (CLI11, nlohmann_json, the schema validator, csv-parser, phmap), `commands/common.hpp` split in two, and the optimization flags of `gecot` applied to it too.

### L4 The bound an algorithm proves, on its interface

`registry.cpp:38-50` keys five descriptions by the names of the algorithms and skips a name that matches none. A virtual returning an optional description replaces the list; the reason why `tree_lp` proves no bound (`:36-37`) moves to its interface. The text and the order of `-M` stay.

### F12 Defaults in one place

- **What exists.** [read] For `add_option(name, member)->default_val(v)`, CLI11 writes `v` into the member when the interface is built. For `add_option_function`, the default text is what the listing and the JSON report, and the initializer of the struct is the value used: `--big-m`, `--mus-sweep`, `--preprocessing`, `--grouping`, `--pricing`, `--theta`. Their texts are mapped to values by ternary chains whose last branch is a silent fallback.
- **Proposal.** One named default tolerance used by the three structs and the interfaces; the defaults of the command line captured from the members; one table of names per enum, used by the validator, the mapping and the listing.
- **Not neutral.** `Static{}` and `Greedy{}` leave 0.0, which moves the unit tests that rely on it (`solvers_test.cpp:63`, `:93`, `:108`).
- **Unverified.** How CLI11 formats a captured default: two readings of its source disagree (through `std::to_string`, exact in C++26 only, or through a stream with 6 digits). The check is the text of `-P` and the parameters of the JSON, both in the harness.
- **Related.** `Static{tol}` and `Greedy{tol}` (`abstract_mip_interface.hpp:115-116`) rely on the tolerance being the first member; designated initializers remove that.

### PR The parser

`parse_instance.cpp` is 1,041 lines, of which 460 are the schema. "A CSV file or an inline array" is written six times (`:504-538`, `:596-616`, `:711-741`, `:775-801`, `:841-864`, `:894-919`). One reader, and one shape of range check (§6, D-safe). About −150 lines. The error texts must stay byte-identical; `parse_instance_test` pins part of them.

### D-safe Fixes that change no valid run

| Defect | Evidence | Mark |
|---|---|---|
| `--mip-start` on a JSON file without `solution` gives an empty start, silently. With `--time-limit 0` the run then returns "buy nothing" with status 0, where it fails without the file | `abstract_mip_interface.hpp:50` uses the non-const `operator[]` | [gecot] |
| `1e400` in a CSV quality is accepted: "Qualities scaled by 1/inf", solution score `-nan`, status 0. `0e5000` gives a NaN that passes too. `nan` is refused, and an empty cell too by the reading of csv-parser | `parse_instance.cpp:702`, `:765`, `:831`, `:877` test `x < 0`; `:491` tests `!(x >= 0)` | [gecot] |
| `Instance::add_conflict_set` checks nothing: a duplicate gives a row coefficient of 2, an unknown id reads out of range. The checks of `add_option` and `set_cost_dimensions` are asserts, absent from every build | `instance.cpp:143-147`, `:127`, `:132` | [read] |
| The SIGINT handler is set to the default, not restored, and the stop flag stays set after an interrupted run | `common.cpp:337`, `:341` | [read] |
| `set_criterion(criterion_formula &&)` copies | `instance.cpp:162` | [read] |

## 7. Stage 4: the executable as a contract

Additive. After L2, so that the escaping is written once.

### O1 JSON always valid

- **What exists.** [read] The keys are built as `"\"" + key + "\":"` (`common.hpp:115-116`) and carry the names of cases, options and cost dimensions; the instance path is printed between quotes (`optimize.cpp:366-370`); the CSV prints `"\"{}\",{}"` (`:435`). A name with a quote or a backslash gives an invalid file, and the schema of the instance allows both. Doubles are printed with `{}`, `inf` included.
- **Proposal.** Strings go through one function, the nlohmann escape already used for the values of the parameters (`abstract_solver_interface.hpp:55-57`). Numbers stay with `std::format`: nlohmann would print `7.0` for a budget of 7. An absent or non-finite optional value is omitted, not `null`: `check.py` and `benchmark/run.py` read them with defaults and would fail on `None`. The text kept for `-t inf` stays. The CSV doubles its quotes. The path uses `generic_u8string`.
- **Bytes.** Unchanged for every instance of the repository: none of their names or paths has a quote, a backslash or a control character. The padding of the keys must be computed on the escaped key.

### O2 Version keys

`gecot_version` and `format_version`, each on its own line and never last in its object, so that the existing lines keep their bytes. Beyond what was named in the ruling, and additive too: the base score and the guaranteed share, which is printed on stdout only and cannot be recomputed from the file. The filter of the harness and the examples of `results.md` and `bound.md` follow.

### O3 Listings as JSON

The algorithms and the parameters of each (name, type, default, allowed values, help). CLI11 gives the name, the help, the type and the default of an option; the allowed values only inside a description, so the tables of F12 feed the listing. A wrapper builds its checks from it and `benchmark/run.py` stops reading the help text. Beyond what was named: the list of the statuses, of which the docs miss `master not solved` (`path_formulation_lp.hpp:697`).

### O4 Schemas

A schema of the two result files, checked by a unit test with the validator already linked into `gecot_test`. The schema of the instance published in the docs: a wrapper writes the instance and the budget text, and today the docs carry only the validator's error.

### O5 The error contract, and a docs page

Every failure exits with 1 after one line on stderr, `error: <state>: <message>` (`optimize.cpp:443-446`). Documented and tested as a contract. Distinct exit codes would be a surface change, not proposed. A page on calling gecot from Python or R through the executable, worded as the supported way today.

## 8. Stage 5: repository and release

- **Design notes.** `tight_big_m.tex` names `prep_mip`, `naive_benders` and `target_benders`; `conflicts_and_cost_vectors.md:3` says nothing is implemented; `path_lp.md` describes behaviour since changed. They are records of studies: a status line at the top of each, not an update.
- **Modes and strays.** 69 tracked files are executable, 25 of them sources and 26 data or documents; `test/instances/Aude/generate.py` is the same blob as `Aude/raw/generate.py`; `.gitignore` names a `doc/` directory that does not exist; `misc/mingw_gcc13` is referenced nowhere.
- **README.** Three `--help` outputs pasted (`README.md:27-124`), with a build date that disagrees with the one of `installation.md`. Links to `docs/reference/parameters.md`, and a script that compares the tables of the docs with the listing of O3.
- **Windows and the release.** `MakeW64.bat:2` uses a profile that is not tracked. The installer bundles `cbc.exe` (`CMakeLists.txt:156-162`) while mippp loads the Cbc library. The tests are off on Windows. Only v1.0 is released and nothing builds on macOS: wrappers need binaries before anything of §6.
- **Tests read the docs examples** (`solvers_test.cpp:230`, `:274`): editing an example of the documentation can break the suite. To know, not to change.

## 9. Stage 6: fixes that change bits, verdicts or logs

Each needs consent and a new capture.

1. **The restore of the qualities.** `compute_options_cases_incr_pc_num` restores the qualities by subtraction after each option (`helper.hpp:336-340`, `:409-413`) and the probabilities by assignment. (a + g) − g is not always a, so the score of an option can depend on the options before it in its TBB block, whose cut depends on the threads. `greedy` absorbs it in its 1e-12 window; `static` sorts without one (`static.hpp:115-118`, `:200-203`). Latent on the bundled instances, whose qualities make the round trip exact. Restoring by assignment removes it and changes bits only where today's result already depends on the cut. [read]
2. **Two definitions of "feasible".** `optimize.cpp:268` accepts a cost up to 1e-6 relative over the budget; `is_feasible` (`helper.hpp:154-164`) and the solvers use `-t`, absolute. With a `-t` above that margin the command warns about a selection that its solver accepted.
3. **all_options and the linear criteria.** `all_options.hpp:17` and `bound.cpp:210` use `is_non_decreasing`, which refuses 2z − z; `bound_criterion` and the MIP solvers accept it through its linear form (`criterion_analysis.hpp:172-184`). One admission test for all would print one more line for such a criterion.
4. **`--reformulation path_lp` drops a start file.** `mip-start=<file>` in `--reformulation-params` is read, then replaced by greedy's solution: the log shows "Computing the MIP start with greedy" and no refusal of a start that is over the budget, which `path_lp` alone refuses. Adding `mip-start-algorithm=none` fails with "--mip-start excludes --mip-start-algorithm". [gecot]
5. **Log lines.** "Removed {} options and {} species" (`common.cpp:214-216`) always counts 0 species. `tree_formulation_lp.hpp:262` logs "tree_decomposition relaxation".
6. **NumericFocus.** `mip_solver_base.hpp:243-244` sets it because Gurobi "pruned better solutions before the scores were scaled". Whether it is still needed now that they are scaled is an experiment, and it changes the path of the solver.
7. **Docs.**
   - `lazy_constraints` counts the cuts asked for: on CPLEX and COPT, mippp drops a lazy row that the candidate does not violate by the solver's tolerance [once].
   - "The first cost dimension in alphabetical order" (`preprocessing.md:207`, `parameters.md`): the code reads dimension 0, which for a JSON instance is the byte order of the names, uppercase first, and for a programmatic builder the order given.
   - `benders.md:140` on the master of `multiflow_benders` (§5.2).
8. **`benchmark/compare.py`** counts `converged`, `relative gap reached` and `no option left` as runs stopped at the limit (`:141-153`), against its own docstring.

## 10. Dropped after verification

- **A helper for the master skeleton.** `formula_variable_visitor` keeps a reference to the z variables (`mip_solver_base.hpp:72-73`): returned next to a z that the helper owns, it dangles. And the helper would hide the top of each program behind a call.
- **`flow_mip` on `compute_contracted_graph_data`.** It renumbers the model (§5.1).
- **A shared cut builder** for `flow_benders` and `multicut_benders` (§5.2).
- **A shared column-generation driver.** Two pieces are verbatim; the rest would be a parameterised loop for two algorithms, one of which is about to be split.
- **Constants for the stat keys.** Most are written once, and they are output: the schema of O4 pins them. The Python scripts and the docs cannot share C++ constants anyway.
- **Structs for the gain tuples and for `preprocessing_row`.** The first is the element type of the case concept, read by structured bindings everywhere; the second is variadic and fed to `std::apply`.
- **`helper.hpp` in three headers.** 422 lines; only the two evaluators of the heuristics move (§5.3).
- **Hooks for progress and stop threaded through the solvers, a parameter API, class invariants beyond three checks.** Code for a door not yet chosen. The entry "name=value" already exists (`path_reformulation.cpp:27-36`).
- **An `assert` in place of a trap comment.** It never runs.
- **A claim of mine that was wrong.** The default `num_mus = 10` of `compute_constrained_strong_and_useless_arcs` is used: four tests rely on it. The 0 of the struct means "labelling off" and is never passed to that function.

## 11. Not proposed: already ruled

- The experimental code and flags: `--big-m plain` and `tight`, `--nb-mus`, `--mus-sweep`, `--preprocessing`, `--all-targets`, `--no-pruning`, `--no-merging`, `--no-root-cuts`, `--no-in-out`, `random_chooser`, `chronometer`, `parse_solution`, the unused preprocessing traits, and the commented contraction block of `compute_contracted_graph.hpp:190-401`.
- `tree_lp`: no rounding, a single budget in its header and in its rows, no conflict set.
- No decremental port; pricing results not sorted; the double seeding of the constrained labelling.
- The split of `path_lp` in two variants: recorded in §5.5, not designed here.

## 12. Python and R

- **Two doors.**
  - **A wrapper around the executable.** It writes an instance, calls `gecot`, reads the JSON. The instance can already be one JSON document with inline tables, and the parameters are command-line text. What it needs from the code is §7; what it needs from the project is binaries (§8). It needs nothing of §6.
  - **An in-process binding.** It needs a facade without CLI11, files, stdout or process globals, and the toolchain of the library where the binding is built.
- **What the second door would cost in the code.** L2 gives the facade for the CLI's own sake. Then: the parameters as data (the tokens that `parse` already takes, once per fresh interface); progress and stop, which are two process-wide variables (`progress.hpp:27`, `:34`), so one run at a time per process; logging through a sink of the host; the instance checked by its class.
- **What blocks it is the toolchain.** [web] unless marked.
  - Six sites of the library need libstdc++ 15 with C++26: two `std::views::concat` (`tree_formulation_lp.hpp:281`, `path_formulation_lp.hpp:647`), three `std::flat_map`, one range format (`multicut_benders.hpp:242`) [read]. melon and mippp need C++23, so GCC 14 at least [read].
  - CRAN checks Windows with GCC 14 and its macOS toolchains predate what melon accepts. The manylinux images ship GCC 14.
  - CRAN forbids writing to stdout and stderr, ending the process and more than two threads. The library logs through spdlog, writes a progress bar to stderr and sends the solver's log to stdout.
  - A second TBB runtime in the host process, solver libraries that mippp never unloads, 2 to 2.5 GB per backend translation unit, and the GPLv3 licence also bear on it.
  - conda-forge and r-universe on Linux are the realistic first channels.
- **Recommendation.** Make the first door solid now. Do L2 because the CLI and the tests gain. Keep `include/gecot` free of CLI11 and nlohmann, which the header check already enforces. Do not write a C interface, hooks or parameter structs, and do not touch the six sites, until a target is chosen: they are cheap to change that day and pay nothing before.

## Recommended first stage

Stage 0, then F1, then the pass of `flow_mip`.

## Appendix A: latent traps, read and not reproduced

Besides the defects of §6 and §9. All [once], except the `qkp_planes` row, which I read: `best_u` is assigned only when a start improves on +∞.

| Trap | Evidence |
|---|---|
| The defaults `1e-7` and `1e-8` of the command line travel through CLI11 as text. One reading of its source has them formatted by `std::to_string`, which gives `1e-07` in C++26 only and `0.000000` before: a build with an older standard library would run with a tolerance and a resolution of 0. The other reading has a stream with 6 digits. Unverified (§6, F12) | CLI11 `Option.hpp`, `TypeTools.hpp` |
| After a second `parse()` on one interface, the listing and the JSON report the defaults while the members keep the values of the first parse. One parse per fresh interface is the contract; nothing states it | CLI11 `App_inl.hpp:720`; `abstract_solver_interface.hpp:28-32` |
| `--name=` with an empty value makes CLI11 take the next token as the value | CLI11 `App_inl.hpp:2415-2433` |
| `qkp_planes`: if every start gives a NaN bound, `best_u` stays empty and is indexed | `qkp_planes.hpp:252-261`, `:329` |
| `split_lagrange::pair_value` skips `check_structure`; on options that break rule R2 it throws `bad_optional_access`. Used by the tests only | `split_lagrange.hpp:543-562`, `:632` |
| The error of a bound below its solution's score is also what an infeasible selection above the bound triggers, and it blames the bound | `algorithm_bound_interface.hpp:57-61` |
| csv-parser builds a decimal digit by digit in long double: the same text can give other last bits from a CSV file than from an inline JSON table | csv-parser `csv.hpp:5385` |
| An option id out of range in a conflict set, from a programmatic builder, is undefined behaviour in `is_feasible` and in the rows | `instance.cpp:143-147` |
| The solver lists offered by the command line and the if-chain of `with_solver` are three hand-kept lists | `abstract_mip_interface.hpp:68-75`, `:128-140` |

## Appendix B: comments

### B.1 Counts of the inventories

One unit is a contiguous comment block, with its dominant class. The classes are an agent's reading and the rows are re-summed from its per-file tables, which the journals of this review hold. A mechanical count of the blocks (runs of `//` lines, plus trailing comments) gives 170, 152, 106 and 99 for the four rows: the totals hold, the classes are orders of magnitude.

| Area | Units | Contract | Trap | Mechanics | Formulation | Restatement | Reassurance | History | Sibling ref | Dead code |
|---|---|---|---|---|---|---|---|---|---|---|
| `include/gecot/solvers` | 174 | 37 | 31 | 21 | 44 | 19 | 3 | 1 | 4 | 10 |
| the rest of `include/gecot` | 139 | 44 | 27 | 10 | 23 | 23 | 3 | 0 | 3 | 1 |
| `src/` and the other headers | 117 | 46 | 30 | 15 | 8 | 2 | 2 | 0 | 1 | 0 |
| `test/` | 107 | 51 | 13 | 4 | 14 | 3 | 0 | 2 | 4 | 7 |

The units that no column counts are labels and formatter directives. The dead code of the second row is the ruled contraction block, 212 lines. The ten of the first row are the switches of decision 3.

### B.2 The 71 candidates, re-read

**Cut (33).** The line below says the same thing, or a name can.

- `mip_solver_base.hpp:144` and `tree_formulation_lp.hpp:228`: both describe `_is_target` more narrowly than its code, which also accepts a positive target quality and `--all-targets`. Rename it `_can_be_target`.
- `mip_solver_base.hpp:187` (`_average_size`: say arcs per target in the name), `:346` (`_add_budget_rows` also adds the conflict rows: say it in the name), `:363`.
- `path_formulation_lp.hpp:123`, `:680`, `:735`, `:908-909`; `tree_formulation_lp.hpp:333-334` (a copy of `path_formulation_lp.hpp:660-661`), `:352`, `:383`.
- `greedy.hpp:79`, `:89`, `:221-222`; `static.hpp:206`.
- `helper.hpp:138`, `:166`, `:174`, `:181`, `:220`.
- `qkp_planes.hpp:107`, `split_lagrange.hpp:345` ("per dimension").
- `compute_contracted_generalized_flow_graph.hpp:29`, `:55`, `:65`, `:67`, `:90`, `:124`: step labels of a 136-line function. Either cut, or the steps become named and carry them.
- `log_table.hpp:19`, `:52-53`.
- `bound.cpp:141`, a copy of `optimize.cpp:166` that goes with L2.
- `log_table_test.cpp:45`, which explains the arithmetic of the static_assert under it.

**Rewrite (18).** The insight is right and the failure, the reason or the tense is missing.

- `mip_solver_base.hpp:243`: history; see §9.6.
- `path_formulation_lp.hpp:948` and `tree_formulation_lp.hpp:453`: "for the same reason" points 300 lines up. State it in place: evicting changes the master, whose solution is unreadable until the next solve.
- `static.hpp:133`: name the trap (§5.3).
- `greedy.hpp:120-127`: the cause it gives is stale (§5.3). `greedy.hpp:324`: "as before the cost dimensions" is the parity rule in the past tense; state it as the invariant.
- `criterion_analysis.hpp:51-52`: keep the first clause, drop the reference to `trivial_reformulate_instance`.
- `qkp_planes.hpp:89`: it omits `fits_alone`. `qkp_planes.hpp:165`: name the failure, a task that fills another row would race.
- `compute_constrained_strong_and_useless_arcs.hpp:269`: give the reason, not "as in the strong loop".
- `compute_contracted_generalized_flow_graph.hpp:60`, `:94`: they restate the condition and omit the reason. `:96`: its reason is wrong (§5.4).
- `common.cpp:312-313`: keep the second clause. `trivial_reformulate.hpp:92-96`: the failures of the alternatives in the present tense.
- `parse_instance_test.cpp:249`, `solvers_test.cpp:210-211`, `preprocessing_test.cpp:514-516`: regression notes in the past tense. The commit that a pin was captured from is worth keeping, as a present fact.

**Replaced by structure (3).**

- `mip_solver_base.hpp:222-223`: `_solve` returns `std::pair<bool, solve_report>`; a named field says what the bool is.
- `path_formulation_lp.hpp:875-876`: one `lambda_plus` for the search and for the bounds of the options.
- `common.cpp:292-293`: named durations instead of a sentence that repeats `5s`, `2` and `1s`.

**Keep (17).** Listed for the cut, and each names a mistake or defines what the name cannot.

- `mip_solver_base.hpp:41-42`: why the concept requires the injection of solutions.
- `mip_solver_base.hpp:173`: why only the first budget is passed.
- `path_formulation_lp.hpp:660-661`: "so nonbasic" is why the eviction is safe. Once, with the shared evictor.
- `path_formulation_lp.hpp:852`: why the search stops at s.
- `multiflow_benders.hpp:548-549`: why the solver's bound is overridden.
- `qkp_planes.hpp:35-36`: the probabilities are those of the options that fit alone, which the name does not say.
- `compute_constrained_strong_and_useless_arcs.hpp:260-261`: why the early break is valid.
- `log_table.hpp:94-96`: the coupling with the prefix of the logs; `:145`.
- `progress.hpp:17`, `:18`: what the two counters count.
- `optimize.cpp:166`: where the names of a budget are checked.
- `common.hpp:50-51`, `:62-63`, `:83-88`.
- `log_table_test.cpp:49`, `bounds_test.cpp:842`.

### B.3 Dead code outside the keep-list

- `flow_mip.hpp` and `flow_benders.hpp`: decision 3.
- `parse_instance_test.cpp:19-32`, `:36`; `preprocessing_test.cpp:117`, `:119`, `:196-248` (two graphviz dumps): commented blocks, some pointing at instances that no longer exist.
- `parse_instance_test.cpp:81-95` and `preprocessing_test.cpp:167-184`, `:253-259` print to stdout during the tests.
- `path_formulation_lp::solve` (`path_formulation_lp.hpp:503-508`) has no caller.

## Appendix C: notation concordance

The canonical symbol, and what a header block or the code writes instead. Blocks: MC `multicut_benders.hpp:29-53`, MF `multiflow_benders.hpp:34-70`, P `path_formulation_lp.hpp:41-87`, T `tree_formulation_lp.hpp:34-74`, Q `qkp_planes.hpp:53-64`, SL `split_lagrange.hpp:41-54`, TB `compute_tight_big_M_maps.hpp:23-56`.

| Concept | notation.md | Written otherwise |
|---|---|---|
| option, its set | i ∈ Φ | o in every block; `o`, `option` and `i` in the code; `Phi` names the flows of `flow_mip` and of T |
| budget | β, β_r | B, B_r, B_0 in every block |
| conflict set | Ω_l | S in MF, Ω in P; S is also a set of options and the strong arcs |
| source quality, its gain | w_u, w^i_u | q_s, q_u in MC, MF, P, T; q^s in Q; Δq_uo, g_o, g^s_o for the gain |
| target quality, its gain | q_u, q^i_u | q_t in MC, P, T, but tq_t in MF, where q_t is the source quality of t; Δq_to, Δtq_to, g^t_o |
| arc probability, improved | π_a, π^i_a | p_a, p'_ao in the blocks; `enhanced_prob`, `improved_prob`, `ip` in the code |
| most reliable path | Π(s,t), Π̄ | P_st, P_ut, P^max, P̄, P0 |
| criterion | 𝒞 | φ in MC, MF, P, T and `criterion_analysis.hpp:19`; in MF φ is also the flow |
| case score | z_k | `C_vars` in `flow_mip` and `flow_benders` |
| scenario | x_i | `X_vars`, with the commented name "Y_"; X_o in T |
| flow reaching t, with a gain | f_t, f^i_t | F_t, F'_to in MF; F, F_o in T; `F_var`, `F_prime_t_var` |
| flow on an arc | φ^t_a | Φ_a in T; `Phi_t_vars` |
| value of a target | v_t | `Xi_vars` in `flow_mip`, absent from `flow.md` |
| big-M | M^t_u, M^t_t | M_ao, M_to in MC; m_sa, M_t in MF; M_u, M_t in T |
| potential, big-M dual | ρ_u, σ_{a^i} | `rho_values`, `cai` in `flow_benders`; `gain` in `multicut` |
| reference scenario of a cut | y | Y in MC; `solution`, `sol`, `current_solution` in `flow_benders` |
| contribution dual | γ_k | π_k in P and T; `pi` in the code |
| budget dual, conflict dual | η, (η_r, ν_l in relaxations.md) | μ, μ_r, ν_Ω in P and T |
| purchase dual, reduced costs | λ_pi, r_pS, d_i | λ_po, rc_pS, d_o |
| labelling multiplier, penalized length | μ, λ^μ_a | in TB, λ is the multiplier and `mus` the sweep |

One symbol, several meanings, in the docs themselves: σ (big-M dual and uniqueness dual), λ (lengths and purchase duals), θ (group value, split of the quadratic terms, dispersal distance), y (reference scenario and columns), ν (prices of `split_lagrange` and conflict duals). They live on different pages; the code should not add to them.

## Appendix D: toolchain facts for Python and R

Read on 2026-11-14 by one agent; the local facts [read], the others [web] and not checked again.

- **What the code needs.** [read]
  - C++26 and GCC 15 (`CMakeLists.txt:15-16`, `installation.md:21`). The library headers use `std::views::concat` twice, `std::flat_map` three times and a range format once: libstdc++ 15. They also use `std::print` (GCC 14), `std::ranges::to`, `views::as_rvalue`, `ranges::contains`.
  - melon 1.0.0 and mippp 1.1.0 are header-only and need C++23; melon refuses a libstdc++ older than 14 and a libc++ older than 20. mippp uses deducing this.
  - mippp loads the solvers at run time and never unloads them. The lookup goes through `MIPPP_<KEY>_LIBRARY`, then the loader's directories.
  - The library headers need melon, mippp, spdlog and TBB. CLI11, nlohmann_json, the schema validator, csv-parser and phmap are needed by the application only.
- **R.** [web]
  - An R package may ask for C++26 from R 4.5.0 "if a suitable compiler is available"; support does not mean complete support. <https://cran.r-project.org/doc/manuals/r-release/R-exts.html>
  - CRAN checks Windows with GCC 14.3 and Linux with GCC 15 or 16; Rtools45 is GCC 14. <https://cran.r-project.org/web/checks/check_flavors.html>, <https://cran.r-project.org/bin/windows/Rtools/rtools45/rtools.html>
  - CRAN policy: compiled code must not end the R process nor write to stdout or stderr; two threads at most; no binary executable in a source package. <https://cran.r-project.org/web/packages/policies.html>
  - r-universe builds binaries for the three systems with an environment close to CRAN's. <https://docs.r-universe.dev/publish/set-up.html>
  - Precedent for the first door: cmdstanr only calls compiled executables, installed apart. <https://mc-stan.org/cmdstanr/>
- **Python.** [web]
  - pybind11 needs C++11, nanobind C++17. <https://pybind11.readthedocs.io/en/stable/>, <https://nanobind.readthedocs.io/>
  - The manylinux images ship GCC 14; `build/gecot` needs GLIBCXX_3.4.32, which only manylinux_2_39 allows unless libstdc++ is linked statically. <https://github.com/pypa/manylinux>
  - conda-forge has GCC 15 on Linux and clang 21 with its own libc++ on macOS. <https://github.com/conda-forge/conda-forge-pinning-feedstock>
  - Two TBB runtimes in one process each keep a pool of threads. <https://uxlfoundation.github.io/oneTBB/main/tbb_userguide/Migration_Guide/Mixing_Two_Runtimes.html>
  - Precedent for the first door: PuLP's command-line solvers, which exchange files in a temporary directory. <https://www.coin-or.org/pulp/guides/how_to_configure_solvers.html>
- **Distribution today.** [read] Only tag v1.0 exists. The Linux package bundles libtbb, libstdc++ and libgcc; the Windows installer is described in §8; nothing builds on macOS.

## Appendix E: what a new algorithm touches today

A measure of the plumbing, from the inventory of the solver interfaces. [read]

| Change | Sites |
|---|---|
| A new algorithm | its header in `include/gecot/solvers`; an interface header; for a MIP algorithm, an interface `.cpp` and a line of `CMakeLists.txt:83-93`; `registry.cpp` (include and list, and the list of the proving algorithms if it proves a bound); `abstract_mip_interface.hpp:96` and `:114-116` if it can give a MIP start; six docs pages; `check.py` and the parity matrix |
| A new parameter | the member and its default; the option in the interface, with the default restated; the docs table. The listing and the JSON follow by themselves |
| A new backend | the include, the three lists and the if-chain of `abstract_mip_interface.hpp`; four docs pages; the licence files |
| A new bound | its header, an interface header, `registry.cpp`; three docs pages; `bounds_test.cpp` and `check.py` |

L4 and F12 remove the list of the proving algorithms and the restated defaults. The rest is the price of one translation unit per MIP interface, which bounds the memory of a build.

## Implementation log

One line per step done, with its commit, so that the work can be resumed from this note.

- 2026-11-14, 2a5967e: the unused instances removed (ruling 7).
- 2026-11-14, 70138a0: docs. notation.md completed with the symbols that the pages use and with the names of the code; `flow.md` states the variable v_t and gives an anchor to each family of constraints; the three wordings of §9.7. The point that `multiflow_benders` separates is written x̂ instead of x̄, which the other pages use for "every option bought".
- 2026-11-14, this commit: S0.1 and S0.2. `test/parity/capture.py` and `compare.py` replace `test/cli_parity.sh`, whose live cases are in the matrix. 739 cases, 50 s with `-j 4`. Two captures of the binary of main do not differ; seven differences injected by hand (a last digit of a score, a log line, a count of nodes, a help text, an exit code, a printed score, a missing case) are all reported. The baseline is `test/parity/captures/main-fd12a84`, with the binary it was made from in `captures/bin`; the directory is ignored by git and stays on this machine.
- 2026-11-14, f6575e7: the matrix widened to the big-M variants of the Benders, which share the contraction with `flow_mip`; 780 cases.
- 2026-11-14, 1aa3523: S0.3 and the pass of `flow_mip`, without change of behaviour. `compute_contracted_graph_data` is cut in two: `compute_contracted_flow_graph` returns a named struct, which `flow_mip` reads directly and the Benders after the renumbering. `solve()` holds the model calls in blocks named after the sections of `flow.md`; the statistics of the tight big-Ms, the commented code and the OPT macro are out of it; the MIP gap has a name. 63 tests; 779 of 779 cases identical to main. One case left the matrix: the LP relaxation on Aix 30 x 30 ends on one of two last digits from run to run, on the binary of main too (4 runs of 8 each).
- 2026-11-14, a22e988 and 5df8d6c: `--indicator-constraints` in `flow_mip` (ruling 3). The model calls stay inline: a refusal at the top of `solve()`, then `if constexpr` at the two sites. Checks:
  - with Gurobi, the same optimum as the big-M model on Aix 30 x 30, and 28 of 28 runs of `test/brute_force/check.py` exact;
  - with CPLEX, 24 of 24 on the instances that its Community Edition accepts here; the 13 others stop on that edition's size limit;
  - refused with `highs` and `copt` ("this solver has no indicator constraints") and with `--lp-relaxation`;
  - against main, the lines of the new parameter and the comma it adds to the line before set aside, 777 cases are identical; the 16 others are the 14 new cases and the two listings of parameters.
  `flow_benders` gets the option in its own pass.
- 2026-11-14, 8da04d2: the harness, after a review of it by four readers.
  - The model itself is now compared, for the runs that end in Gurobi: a `gurobi.env` with `ResultFile model.mps` in the directory of a run makes Gurobi write the model it solves, and the capture keeps its hash (`--keep-models` keeps the file). The file lists rows and columns in the order they were added, coefficients to 17 digits. It does not tell the order of the terms inside a row, the models of a run before the last one solved, the cuts added during a search, nor anything of another solver.
  - `compare.py` had a hole: a dropped line no longer separated two tables, and a table next to a search table would have left the comparison with it. A table is now closed by its third rule and by any other line. No case of the captures was in that situation.
  - Also: a timeout is reported as a difference; the comma that ends a JSON line is not compared, so a new last parameter adds only its own line; the version line is compared without its date; a capture can be renamed (`meta.json` records its path); `meta.json` records the compiler and the build type; the runs of `path_lp` and `tree_lp` of the surface cases have `--max-threads 1`; the abbreviated options of `cli_parity.sh` and the two refusals of `--indicator-constraints` are cases.
  - Checks: two captures of the binary of main, 0 of 822 cases differ, 508 with a model. 20 mutations of a capture behave as expected (14 seen, 6 of times and dates not seen); the script before the fix misses 5 of the 14 and reports the comma.
  - Result for the pass of `flow_mip`: main against a22e988, the lines of the new parameter set aside, 804 of 822 cases identical, the 508 models included. The 18 others are the 14 runs with the new option, the two listings of parameters and the two refusals.
- 2026-11-14, fb28c6b: the pass of `flow_mip` after its review, one build.
  - `contracted_flow_graph` is filled with designated initializers and read by member in the Benders stage, in place of eleven positional names; `improvement_option_map` (was `arc_option_map`, one letter from `arc_options_map`) and `improvement_big_M(a)` (was `arc_big_M`); the traps of `arc_no_map`, of `improvement_big_M` on an arc that is no improvement, and of the base `probability_map` of the Benders stage are written.
  - `flow_mip`: the block of v_t next to the loop that bounds it, the statistics in one line and not logged with the indicator constraints, each block comment names the sections of `flow.md` it builds.
  - Two `static_assert`s: gurobi and cplex have indicator constraints, as the help says.
  - Docs: the table "In the code" of notation.md gives the names of `flow_mip` only, as they are; with the indicator constraints, `constraints` and `nonzeros` count the linear rows only (measured with both solvers); `lazy_constraints` counts a cut that CPLEX or COPT are not given.
  - Checks: 63 tests; against the capture of a22e988, 820 of 822 cases identical, models included, the two others being the help text reworded; `check.py` on `flow_mip`, its two variants and the Benders with tight big-Ms, 222 runs, 0 failures.
  - Left for the Benders pass: one helper of the base for "strong and useless arcs, their row of the preprocessing table, the debug line", which the four solvers repeat.
  - Not done, for the owner: with `--indicator-constraints` the big-Ms are still computed; flow.md uses v for a sub-formula of the criterion and v_t for the value of a target.
  - Rebuilt from the commit: 0 of 822 cases differ from the capture made before it; with CPLEX, `check.py` on `flow_mip --indicator-constraints` gives 24 exact runs, the 13 others stopping on the size limit of its Community Edition.
  The capture of this state, `captures/branch-fb28c6b`, is the baseline of the next passes, with `captures/main-fd12a84`.

- 2026-11-14, the owner's answers to the report on `flow_mip`: (1) no big-M is computed under `--indicator-constraints`; (2) the variable of a sub-formula of the criterion takes another letter than the v of a target: the owner asked for z if free, it is the score of a case on that page, so ζ; (3) the label of `tree_lp` is left to me: it follows the rename to the name of the command line. Then: continue with the other algorithms.
- 2026-11-14, 6cf182a: the three answers above but the label, and the pass of the Benders (§5.2), one build.
  - The three `solve()` open with the eight lines of `flow_mip` (tolerances, z, the criterion, x, the budget and conflict rows), then hold their model calls in blocks named after `benders.md`. The header blocks are gone: `benders.md` got what they said more (the program of a source, the test of A_s with its 1e-9, the fallback of the separation, the 1e-7 of the cuts on the relaxation, M^t_t and the positive gains of `multiflow_benders`).
  - Shared: `contracted_graph_data::flow_and_potentials` is the one dual flow (`multicut_benders` scales it by the target quality afterwards); in the base, `_add_mip_start` returns the start, `_hand_back` carries the comment on the candidates once, `_bought_options` and `_is_bought` are the readback. The cut expressions stay per solver, with their operand order.
  - `flow_benders --indicator-constraints`: on the rows of the target quality gains; the cuts keep their big-Ms. Refused on `copt`.
  - `multiflow_benders`: `root_cut_tol` and `path_tie` name the two literals; `_compute_sources` and `_separate` are out of `solve()`.
  - Checks: 63 tests; against the capture of fb28c6b, 820 of 837 cases identical, models, cuts and nodes included, the 17 others being the new option (14 runs, a refusal, two listings); `check.py` on the Benders and on the two indicator options, 444 runs, 0 failures, CPLEX included for `flow_benders`.
  - Only one of the seven plan readers of this pass returned (`multiflow_benders`) before the session limit; the rest of the pass is from my own reading, then reviewed on the diff.
  - Put to the owner by that reader, not decided: `multiflow_benders` computes the knapsack big-M of every tail of an improvement and reads only the one of t; the fallback of its separation makes `value` an upper bound, which the callback takes as the true flow (read, not reproduced); `root_rounds` does not count the last round, which adds no cut.
  The capture of this state is `captures/branch-6cf182a`.

- 2026-11-14, d503ed1: greedy and static (§5.3). `heuristics_common.hpp` holds what the two share: `heuristic_landscape` (the qualities and probabilities with the options bought, `set`, `buy`, `remove`, `score`, and the two evaluators of `helper.hpp` as `scores_with` and `scores_without`), the tie rule (`tie_window`, `free_option_tier`, `ratio_key`), `is_blocked`, the refusal of `--decremental`. `fits_alone` takes a tolerance and serves the base too. The two comments of §5.3 are corrected. The budget-left tests keep their three shapes. Checks: 63 tests; 837 of 837 cases identical; the logs at `-vv` of 15 runs of the two heuristics identical to those of the binary of main.
- 2026-11-14, afc68ed: the rename alone (ruling 4). `path_lp.hpp`, `tree_lp.hpp`, `path_lp_interface.*`, `tree_lp_interface.*`, the structs `path_lp` and `tree_lp`, the classes `PathLpInterface` and `TreeLpInterface`; the trace line of `tree_lp` says `tree_lp relaxation` (decision left to me). 837 of 837 cases identical. The design notes keep the old paths: they are dated records.
- 2026-11-14, bc4efcd: `path_lp` and `tree_lp` (§5.5, light). The header blocks are gone: `relaxations.md` got the labels of the two pricings and their dominance rules, the rows of a pair that enter the master with its first column, the clamps of the reduced cost of an option, the first columns of `tree_lp`. Options are `i`, the dual of a contribution row is `gamma`, the value of a label is ω in the comments. `column_generation.hpp` holds the pricing margin and the evictor, once. `tree_lp` uses `mip_gap` and `_is_bought`. Nothing is restructured: the split of `path_lp` is the owner's. Checks: 63 tests; 837 of 837 cases identical; the `-vv` log of `tree_lp` identical to main's but for the label.
- 2026-11-14, 4c38f61: preprocessing and bounds (§5.4, §5.6). The block of the tight big-Ms is in `preprocessing.md` ("The tight constants"), those of `qkp_planes` and `split_lagrange` were already in `bounds.md`, which got the corridor of a pair. Options are `i` in the preprocessing, the base, `helper.hpp`, `trivial_reformulate.hpp` and `split_lagrange`; `qkp_planes` and `lp_knapsack` keep `i` for the items of their knapsacks. `probability_to_length`, `sort_labels` and `path_length_bound` are defined once for the two labellings; the multipliers of the tight big-Ms have a `static_assert` on their first value; the two typos are gone; the roles of `feasibility_tol` are on the member. Checks: 63 tests; 837 of 837 cases identical; `check.py` whole, 1048 runs, 0 failures.
  - Not done of §5.4, by choice: the renames of `useless_vertices_map`, of the `*_default_traits` and of `enhanced_prob` (three spellings, in the tests too); the helper for the free variables `{}`; `static_assert(!std::ranges::range<budget_t>)`; the scale by `std::ldexp`. None changes a statement of the mathematics.
- 2026-11-14, 543ad9f: the review of the Benders pass applied (two readers; no difference found in the model calls, the cut expressions or the shared helpers), and CONTRIBUTING.md (§5.7).
  - `lazy_constraint_model` now requires the two tolerances and the MIP start: the base sets them through helpers that skip a missing setter, so a backend that lost one would have kept its default silently.
  - `_hand_back` states what the caller must have written; `flow_benders` says that every option of the vertex gets an f^i_t; `flow_mip` and `multiflow_benders` write `z(k)`; the help of `flow_mip --indicator-constraints` says that `--big-m` is not read.
  - `benders.md`: a cut is valid for the scenarios for which the big-Ms are, those within the budget by default; the fallback of the separation drops the term of the improved copies only; the paragraph on the cuts on the relaxation reordered.
  - The helper of the base for "strong and useless arcs, their row, the debug line", left for this pass by fb28c6b, is dropped: the row has other columns in each solver and is added after the targets in two of them, and the debug line differs in `multiflow_benders`.
- 2026-11-14, a29927b: the review of the passes after the Benders applied (two readers on d503ed1 to 543ad9f; no difference found in the arithmetic of the heuristics, nor a rename that changed a meaning).
  - Comments: `tie_window` says that greedy alone uses the window and how the evaluators restore the qualities; the two blocks of greedy name `--worst`; the roles of `feasibility_tol` include those of `path_lp`; `lazy_constraint_model` says what a missing setter would do; `qkp_planes::target` is a score of the criterion (a statement of §5.4). The other statement of §5.4, the guard of `compute_contracted_generalized_flow_graph.hpp` "should not be needed", is left as it is.
  - `flow_mip` logs "Computing contractions", without "and big-Ms", under `--indicator-constraints`.
  - Docs: notation.md has the label of the pricing, k_j and Π^μ; the tight constants take the least per source and count the budget left with the slack `-t`; `tree_lp` prices only the sets within the budget, which tightens its relaxation; CONTRIBUTING.md states the two helpers that do build rows, the default restated in the interface, what a bound touches, what the hash of a model does not see.
  - The three files that the rename had left unformatted are formatted.
  - Checks: 63 tests; against the capture of 543ad9f, the only differences are the debug line of the 14 runs of `flow_mip --indicator-constraints`.
  - Against main (fd12a84), the lines of the new parameter set aside: 802 of 837 cases identical, the 536 models of the runs that end in Gurobi among them where main has one (508); the 35 others are the new option: 28 runs, 3 refusals, 4 listings of parameters.
  - A workflow of seven plan readers launched before the Benders pass used 1.7 M tokens and returned one plan before the session limit; the reviews that followed were two readers each, on a committed diff.
- 2026-11-14, the owner's answers on `multiflow_benders`: (1) only the big-M of t is to be computed; (2) the claim on the fallback is to be verified and a fix assessed; (3) `root_rounds` is to be fixed; and `compute_sources()` and `reach()` are to use melon (its vertices, its Dijkstra on the graph and on its reverse), with a pass on what the algorithm could do better.
- 2026-11-14, 7f7d9f2 to da6b556: that pass, six commits, each built and compared on the 66 cases of `multiflow_benders`.
  - 7f7d9f2, `root_rounds`: every relaxation solved to optimality counts, the last one included, and the rounds of the progress table are numbered from 1, as those of `path_lp` and `tree_lp` (they were numbered from 0, so the last row and the count agreed while both left a round out).
  - 1d57b67, the block of a target: a `melon::static_digraph` renumbered from `compute_contracted_flow_graph`, in which an improved copy is already an arc of its own, with its maps; `reach()` is melon's Dijkstra on the graph and on its reverse view, one object reused from a source to the next. Bit for bit the results of before.
  - 70fe8ad, M_t alone: the first stage computes no big-M, and the knapsack is solved for t. The models are the same: the constants of the vertices are independent, which was read and is now run.
  - 022b37d, the fallback. The claim holds: with the fallback forced for every source, the oracle failed on 14 of 111 runs, each with the status optimal on a worse solution. It did not fire once in 8 million separations of a source on ten instances: it is a guard against rounding. The new fallback takes the Dijkstra potentials over the arcs of A_s open at the point: a feasible dual solution at every point, and at a scenario the probabilities of connection, so the flow handed to the solver stays the true one. Forced, the oracle passes on 111 of 111.
  - b888030: no second search of labels after a push that fills no arc and takes the whole supply; the potentials reset where the last search set them. Bit for bit the same results.
  - da6b556: the search of a source is `_largest_flow`, its scratch `flow_search`; the tolerances and the guards have names; a source keeps m_sa for its improved copies only, where it kept a value, infinite, for every arc.
  - Where the time goes, measured: the searches of labels take 85 to 97 % of the CPU time of the separation (12.8 million searches, 68 s of CPU, on `species-natural` at 6000), the sets A_s being large (47 million pairs of a source and an arc there). `compute_sources` and the big-Ms are minor: 0.2 s and 5 ms on that instance.
  - Benchmark, short suite at 30 s per run, before the pass and after: 31 of 42 runs optimal in both. 29 of these have the same cuts, nodes, scores and bounds; the two others end their rounds on the relaxation on the clock, do more of them in the same time, and reach the same optimum. Run again with a time limit of 600 s, the two others have the cuts and the nodes of before too. Wall time of the 31, one run of the suite each, alone on the machine: 255 s before the pass, 220 s at b888030, 231 s at da6b556. So da6b556, which keeps m_sa for the improved copies only and moves the search into `_largest_flow`, costs about 5 % of the time and saves memory: peak on `species-natural` 1.45 GB before the pass, 1.42 GB at b888030, 1.00 GB at da6b556. `bool` flags in place of `char` ones, tried against aliasing, change nothing. Build time, which holds the rounds on the relaxation: 206 s, 179 s, 187 s.
  - Tried and dropped: one Dijkstra per target shared by the sources whose best path is not filled, their potentials taken from it. Valid, and the oracle passes, but no faster (226 s against 220 s on the 31 runs) and its cuts are weaker on `species-natural` (388 nodes against 14 at 12000).
  - Checks at da6b556: 63 tests; against the capture of a29927b, the 837 cases identical but for the numbering of the rounds and `root_rounds`; `check.py` whole, 1048 runs, 0 failures.
  - The guards forced at da6b556, in two temporary builds: with every search of labels giving up after its first vertex, and with every source that needs a second push giving up, the oracle passes on 111 of 111 runs each.
  - `label()` stays written by hand. melon's `bellman_ford_moore` is the same queue search, but it has no tolerance on an improvement and no guard: an arc taken forward then backward is a cycle of gain fl(π · 1/π), which rounding can put above 1, its "negative cycle". It would need the residual graph as a view, a semiring whose comparison carries `label_tie`, and its detection of cycles as the guard; the labels would change in their last digits.
  - Review: one reader on what the runs cannot see (two earlier ones were cut by the session limit). Not refuted: the fallback's potentials are a feasible dual solution at every point and its value exact at a scenario; the skipped search cannot make a cut invalid; no stale potential or capacity is read; nothing is shared between two separations. Its one remark, on the wording of `root_rounds` in results.md, is applied.
  - Leads not followed: Dijkstra for the first search of a source, which has no backward arc; updating the labels after a push in place of searching again; keeping the part of a source from a round to the next when no option of its arcs moved.
- 2026-11-14, this commit: the rulings above recorded, and `misc/design/multiflow_benders_separation.md`: what the separation does, what was measured, what was tried and dropped, five leads and how to measure a change.
- 2026-11-14, this commit: Stage 5 as ruled. A dated status line at the top of the five design notes. 53 files that are not scripts lose their executable bit (the nine scripts with a shebang keep it); the duplicate `test/instances/Aude/raw/generate.py` is removed; `.gitignore` no longer names `/doc/html` and `/doc/latex`. `misc/mingw_gcc13`, a profile for another compiler than the one `MakeW64.bat` names, is left: Windows stays with the owner.
- 2026-11-14, this commit: §9.8. `benchmark/compare.py` takes the own time of every run that ended by itself, the statuses `converged`, `relative gap reached` and `no option left` included, where it took the time limit; `solved` still counts the status `optimal` alone.
- 2026-11-14, this commit: Stage 3 as ruled (L2, L4, F12, the five safe fixes), §9.2, and what the passes had left aside.
  - L2: `include/commands/run_command.hpp` parses the command line that the two commands share (`parse_run_arguments`, with the words by which they differ in `run_command_words`). `optimize.cpp` and `bound.cpp` are each: the arguments, the run, `evaluate` into a plain struct (`optimize_outcome`, `bound_outcome`), then `print_summary`, `write_json` and `write_csv`, which only print. `print_json_head` and `print_json_instance` of `common.cpp` are the parts of the two result files that are the same. 737 lines of the two commands become 570, plus 164 shared.
  - §9.2: the check of the command is `check_feasibility`, with its margin named `budget_margin`; `results.md` says what `feasible` means and that it does not follow `-t`.
  - L4: `AbstractSolverInterface::bound_description()`; `registry.cpp` lists as a bound method every algorithm that gives one, where it held a second list of names. `tree_lp` gives none and says why.
  - F12: `gecot::default_feasibility_tol`, 1e-7, for `mip_solver_base`, `Greedy` and `Static`. Every `->default_val(...)` of an interface takes the member, so a default is stated once, in the struct. `add_choice_option` declares an option that chooses among names (`--big-m`, `--mus-sweep`, `--preprocessing`, `--grouping`, `--pricing`, `--theta`): the names, the default shown and the value set come from one table, where each had a chain of ternaries with a silent last branch. No unit test had to state a tolerance: those that build `Greedy{}` or `Static{}` now run with 1e-7 where they ran with 0, and their expected selections hold.
  - The safe fixes: `--mip-start` on a file without a `solution` object is an error (it was an empty start); a quality, a gain, a cost or a probability that is not finite is refused, which only a CSV file can hold (1e400 was read as infinity); `Instance` throws on an option given twice, a wrong number of costs, the dimensions set after an option, and a conflict set with an unknown option or the same one twice, where it had asserts that no build keeps; the command restores the handler of Ctrl-C that it found and resets its flag; `Static{.feasibility_tol = tol}`.
  - Left aside in §5.4, now done: `never_target_map` (was `useless_vertices_map` and `useless_target_vertices_map`), `blue_wins_ties_traits` and `red_wins_ties_traits` (were `strong_arc_default_traits` and `useless_arc_default_traits`), `improved_probability` for its three spellings; `free_variable()` in the visitor of the criterion; `static_assert(!std::ranges::range<budget_t>)`; the scale of the qualities by `std::ldexp`.
  - Checks: 65 tests, two of them new (the values that are not finite, per table of the CSV format; the refusals of `Instance`); against the capture of bb5afd4, 837 of 837 cases identical, help texts, listings, result files, logs and models; `check.py` whole, 1048 runs, 0 failures; no warning.
- 2026-11-14, this commit: Stage 4 as ruled (O1, O2) and §9.4.
  - O1: every text of a result file goes through `json_string` (`io_helper.hpp`, which `option_json_value` now uses too): the path of the instance, the status, and the keys of the objects, which are names of the instance (options, cases, cost dimensions). A statistic of a report that is not a finite number is left out. A name in the CSV file goes through `csv_field`, which doubles its quotes. Tried with an option named with a quote, one with a backslash and one with a comma and a tab, in a directory named with quotes: the two files parse and give the names back.
  - O2: `gecot_version` and `format_version` (1, `result_format_version`) open the two result files; `base_score` follows `initial_score` in both; `guaranteed_share` follows `feasible` in a solution file, when the summary prints one. The last two go beyond the version keys that the owner named: they are what a reader needs to recompute the share, and the share itself.
  - §9.4: the `path_lp` of `--reformulation path_lp` does not run greedy when `mip-start=<file>` is among its parameters; its parameters then show `mip-start-algorithm none`. Tried: the start of the file is used, and one that buys two options of a conflict set is refused by `path_lp` with its warning.
  - The default of `--reformulation` is read from its variable, as the others.
  - Docs: the two examples and the field tables of results.md and bound.md, the CSV format, the three sentences on the start of the reformulation.
  - Checks: 65 tests; against the capture of the commit before, 837 of 837 cases identical once the four new keys are set aside (638 cases write a result file); no warning.
- 2026-11-14, fa62d4d: §9.3, the criterion in the reformulation.
  - `simplified_criterion` (`criterion_analysis.hpp`): a linear part of the criterion that reads a case twice, or with weights that add up to 0, is written again as constant + Σ w_k z_k over its cases of non-zero weight. Every other part is returned as written: a normal form for all would change the order of the operations, so the last digits and the models, of the instances that have nothing to simplify. `biobj_aude`, whose sum holds a minimum, is not linear as a whole and keeps its two terms of `trout`.
  - `trivial_reformulate_instance` gives the algorithm the simplified criterion and the cases it reads, under new ids, and takes the scale from these cases only. A criterion that reads no case is an error (`the criterion reads no case: every selection of options has the same score`); such a run ended before with an arbitrary selection.
  - The results are per case of the instance as given: a case read takes its score from the instance of the algorithm, by its name; a case left out is scored on the raw instance with the options bought, which can improve it too. So the keys of `initial_pc_num` and `solution_pc_num`, and the lines of "Per species", do not depend on the criterion. Mine, not ruled: the owner may prefer these cases absent from the results.
  - The admission tests are untouched: `all_options` still uses `is_non_decreasing`, which now sees the simplified criterion, so 2z − z passes; a criterion with nothing to add up and a decreasing term inside, as `[-1, [-2, z]]`, is still refused by it alone.
  - "Removed … species" now counts the cases left out.
  - Tried on `two-species.json`, with `flow_mip` and the model that Gurobi solves: the criteria `squirrel` and 2·squirrel − squirrel + 0·hedgehog give the model of the file that holds the squirrel alone, and the same results, the score of the hedgehog included; a minimum over squirrel + hedgehog − hedgehog, refused before, gives the model of the same minimum written without the hedgehog; the minimum of the two species gives the model of before. An option that improves only a case left out is handled by every algorithm; the heuristics buy it when the budget is left, as any option without gain.
  - Checks: 69 tests, three of them new (the writings kept, the linear parts added up, the cases left out with their new ids, the scale and the refusal); 837 of 837 cases identical to the capture of c63695a, no instance of the matrix having a case unread or a part to add up; `check.py` whole with two new instances (the terms of a sum that add up to the second case of the file, and such a sum inside a minimum), 1234 runs, 0 failures.
  - Docs: preprocessing.md ("The instance given to the algorithms"), the note of instance.md on the criteria and its table of errors, which also gets the two messages on values that are not finite, results.md.
- 2026-11-14, 901ac31: §9.1, the restore of the qualities. `scores_with` and `scores_without` assign the quality of the landscape back where they subtracted or added the gain. The comment of `tie_window` and the sentence of heuristics.md no longer speak of the threads.
  - Cost, measured on three instances of the benchmark with many vertex improvements, three runs each, before and after: `greedy` 25.2 s and 25.3 s on `urban-greening-large` at 8000, 10.9 s and 10.9 s on `species-periurban` at 6000, 4.30 s and 4.31 s on `periurban-mixed` at 12000; `static` 0.90 s and 0.90 s, 0.30 s and 0.31 s, 0.13 s and 0.13 s. Within the noise. The scores and the selections are the same bit for bit.
  - The test (`an_option_scores_the_same_after_any_other`): 32 cases of one vertex with two options each; with the old restore it fails on the second option of every case, for both evaluators. Two earlier shapes passed on the old code and proved nothing: with 8 options TBB gives each its own block even on one thread, and with several vertices in a case the sum of the score hides the last digit.
  - Checks: 837 of 837 cases identical; `check.py` as above.
- 2026-11-14, §9.6, NumericFocus measured, for the owner to decide. A temporary build of the code before ee3fd23 with a switch; `flow_mip` and `multiflow_benders` on the short suite (42 runs each), 30 s per run, one run of each, with and without NumericFocus = 1.

  | | `flow_mip` with | without | `multiflow_benders` with | without |
  |---|---|---|---|---|
  | runs proved optimal | 12 | 12 | 31 | 30 |
  | time of the runs optimal in both | 67.8 s | 51.6 s | 215.7 s | 212.9 s |
  | runs without a bound at the limit | 14 | 12 | 0 | 0 |
  | at the limit, runs with the better score | 2 | 5 | 1 | 6 |

  - No run that ends optimal differs: the 12 optima of `flow_mip` and the 30 of `multiflow_benders` are the same with and without, and the same between the two algorithms, to 1e-9. No selection is infeasible.
  - `flow_mip` without it is faster on its two long solved runs (`cover-40` at 16: 18.3 s to 9.8 s; `faircover-40` at 16: 16.5 s to 10.0 s) and gets a bound on two more runs.
  - `multiflow_benders` loses one run without it: `bike-squirrel-suburb` at 200 is optimal in 16.8 s with it and at the limit without (gap 0.1 %).
  - One run per setting: a difference of a few percent is noise, the two above are not. The switch (`GECOT_NUMERIC_FOCUS_OFF`) is in no commit.
- 2026-11-14, this commit: the review of ee3fd23 to 901ac31, two readers, on what the captures and the oracle cannot see.
  - Not refuted, most of it by runs: the shared command line gives the messages and the orders of before (help, listings, `-P` alone, an unknown name, an algorithm that proves no bound, a missing argument); the results per case with a case left out, with options bought statically, several costs, `--reformulation path_lp`, `tree_lp` over the budget and no option left; the simplified criterion keeps its value and the new ids are right (880 runs against the brute force on 34 criteria); the restore by assignment; the choice options; the start file of the reformulation; the handler of Ctrl-C; the examples of the docs against the output of their commands.
  - Applied. The names of the instance in a result file keep their bytes (`json_name`): c63695a replaced a byte that is not UTF-8, so the option ids of a CSV file in Latin-1 no longer were those of the instance, two of them could become one key, and the file given back as a start lost them without a word. Such a file is read in the encoding of the names, as before; its other strings are escaped. The row of `guaranteed_share` in results.md says which bound it takes; optimize.md says that a value of `--reformulation-params` cannot hold a space; the comment of the test of the values that are not finite no longer speaks of NaN, which the CSV reader refuses by itself.
  - Found, older than these commits, not touched, for the owner:
    1. Weights of 1e-8 and smaller in the criterion, on cases whose scores differ as much. Two cases, the squirrel of `two-species.json` and a copy with its qualities times 1e4, criterion squirrel + 1e-8 × copy, budget 5: `path_lp` gives a bound of 35.80 under its own solution of 57.52 (optimum 70.8), and `gecot optimize` prints an upper bound of 57.52 with "at least 100.0 %", since it takes the larger of the bound and the score; `gecot bound` refuses the same bound. With a minimum, a weight of 1e-10 and qualities times 1e5, `flow_mip` ends `optimal` at 20.96 where the optimum is 28.76. The scale is taken from the largest case score, not from the criterion: with such a weight the criterion is near 1e-3, and 1e-5 in the second instance, in the unit of the solver. The same two instances with their criterion multiplied by 1e8 and 1e10, which are the same problems, give a valid bound and the optimum: the magnitude of the criterion is the cause. Weights of 1e-6 and larger gave no wrong result. In the 813 result files of the captures and of the benchmarks that have a bound and a feasible solution, the bound is never under the score by more than 7e-15: a check in `evaluate` that warns and drops a bound under the score by more than 1e-6 would change none of them.
    2. Qualities of 1e154 and more: the scores overflow, the result files hold `inf` and `-nan` and are no JSON, the command exits 0.
    3. `greedy` buys options that lower a criterion that decreases with the scores (`[-1, "squirrel"]`: −28.76 where buying nothing gives −18.4).
    4. A product with a factor 0 is a constant for `linear_form` only if the 0 comes before its second factor that reads a case: `product(squirrel, 0, hedgehog)` is refused as reading no case, `product(squirrel, hedgehog, 0)` runs.
  - Checks: 69 tests; 837 of 837 cases identical to the capture of 901ac31; the Latin-1 instance of the reader gives its ids back.

## Rulings of 2026-11-15, after the merge

The branch was squash-merged into main as fac89d0. The owner then agreed with the five recommendations of the report:

1. A bound under the score of its own solution is no longer shown; the scale of the criterion is a task of its own.
2. A case that the criterion does not read stays in the results.
3. A criterion that reads no case is refused.
4. The names that are not UTF-8 keep their bytes in a result file.
5. The four findings of the review that are older than these commits are fixed. The report gave no recommendation for this item; agreement was read as fixing them.

On NumericFocus, the owner asked for an option that enables it, or its equivalent for the solver chosen, off by default.

- 2026-11-15, fb49c22 (branch `numeric-focus`): the check of the bound, and the four findings of item 5.
  - `gecot optimize` drops an algorithm's bound that is under the score of its solution by more than `bound_margin` (1e-6 of the score, or of 1): it warns, shows no upper bound and no guaranteed share; the report keeps `best_bound` and `gap` as the algorithm gave them. `guaranteed_gain_share` applies the same margin, so the progress table shows no share either. `gecot bound` keeps its own refusal, at 1e-9.
  - The qualities of a case whose scores could exceed the largest double are refused when the instance is reformulated: as a probability is at most 1, no score exceeds the product of the sums of the qualities with every gain. The scores printed by both commands are checked to be finite: only a criterion of huge weights or products could still make one infinite.
  - The heuristics buy no option that lowers the score, and `--decremental` goes on removing while a removal raises it; only a criterion that decreases with the score of a case allows either. `greedy --worst` keeps no such rule. `greedy --decremental` tests the criterion first: on a non-decreasing one, the extra evaluation would cost the time of a step.
  - `linear_form` gives 0 for a product with a factor 0 wherever the 0 stands: the same constant was linear in one writing and not in the other, so refused as reading no case in one and run in the other.
  - `all_options` and the line "All options bound" of `gecot bound` take `has_non_decreasing_criterion`, the test of the other bounds and of the MIP solvers: `[-1, [-2, z]]` is 2 z.
  - Checks: 71 tests; the five new or extended ones fail on the code of main (built against it); 837 of 837 cases identical to main; the reproducers of the review give the warning, the refusal, −18.4 where greedy gave −28.76, and the refusal of `product(squirrel, hedgehog, 0)`.
- 2026-11-15, 20a1f6e: `--numeric-focus`, off by default, for every algorithm that takes `--solver`, on every model it solves: the MIP of `flow_mip` and the Benders, the masters of `path_lp` and `tree_lp`, and the pricing MIPs of `tree_lp`. Without the flag nothing is set; before, Gurobi's MIPs had NumericFocus 1.
  - The settings, each the first level above the default, from the installed headers and the solvers' documentation:

    | Solver | Setting | Value | Models |
    |---|---|---|---|
    | Gurobi 13.0 | `NumericFocus` | 1 (0 to 3) | LP and MIP |
    | CPLEX 22.1 | `CPXPARAM_Emphasis_Numerical`, id 1083 | 1 (on or off) | LP and MIP |
    | Xpress 9.6, 9.9 | `XPRS_NUMERICALEMPHASIS`, id 8416 | 1, mild (default −1, automatic; 0 asks for speed) | LP and MIP |
    | MOSEK 11.0 | `MSK_IPAR_MIO_NUMERICAL_EMPHASIS_LEVEL`, id 84 | 1 (0 to 2) | MIP |
    | COPT 8.0, HiGHS 1.10, CBC, GLPK | none | | |
    | SCIP 8.0 | the numerics emphasis, through `SCIPsetEmphasis`, which mippp's table does not load | | |

  - The ids are written out: mippp's tables do not name them. MOSEK's is the position of the name in an enum ordered by names, as are the ids that mippp itself writes out for MOSEK (100, 110), so a parameter added or removed before it would move those too, which the versions that mippp validates (11.0 to 11.3) rule out. A solver without a setting warns once per kind of model, as for `--time-limit`.
  - Checks: on the three woods, with each of the nine solvers and `flow_mip`, `path_lp`, `tree_lp` and `multicut_benders` where the solver is accepted, the optimum 42.44, the setting in the solver's own log for the MIPs of Gurobi, CPLEX and Xpress, the warning for the others; a scratch program sets the option on a model of each of the seven kinds and reads the value back through the solver's getter: 1 for each, after the defaults 0, 0, −1 and 0. 71 tests. Against main, the lines of the new flag set aside, 112 of 837 cases differ, every one in what the solver's path changes: nodes (52 lines), the share 99.9 or 100 at the optimum (19), the last digits of a bound (9), lazy cuts (5), the help text (11), and 16 selections, all optimal before and after with the same score to the last bit (aix30 at 8 and dks30, which have ties); one score changes, under `--lp-relaxation`, where the relaxation's value is the same. No status changes. `check.py` whole, 1234 runs, 0 failures. The benchmark of §9.6 was the measurement.
- 2026-11-15, 0f49bb7: the review of the two branches (two readers, on what the captures and the oracle do not see), its findings on this branch.
  - `greedy` checked the sign on the option that its tie rule takes. The window of a tie grows with 1 / the cost of the best option, so an option of id below it and of negative gain could be tied with it: the purchases stopped, though options that raise the score still fitted. The tie now skips the options that lower the score, and the purchases stop only when the best lowers it. Before the sign rule, the same tie bought the option that lowers the score.
  - The decremental steps, once the cost fits, stopped on the option of smallest loss per unit of cost, whose removal can raise the score by less than the window while that of another raises it more. Both heuristics now remove, among the options whose removal raises the score, the one they would have taken; `greedy` with the same choice and tie as before the cost fits, `static` in its order.
  - `multiflow_benders` solves its rounds on the relaxation before `_solve`: it sets the option where it builds the model.
  - The help, the warning and parameters.md said the other solvers have no such setting: SCIP has its numerics emphasis, which mippp cannot set. results.md says the margin of the bound is 1e-6 under a score below 1, as the code does.
  - Checks: 73 tests, the two new ones fail before the fix; 837 cases identical to the capture of 20a1f6e but the 11 listings of the reworded help; `check.py` on the heuristics and `multiflow_benders`, 145 runs, 0 failures.
  - Found, left as it is: a score under 1 is checked against a margin of 1e-6, not 1e-6 of itself, as the budget is; an instance of qualities near 1e-5 has scores near 1e-9, which the qualities are never scaled up to correct, and a bound of 0 then passes.
- 2026-11-15, e4f51c0: the test of the qualities of fb49c22 refused instances that main ran: it took every gain, of options over the budget too, and counted sources and targets that never connect. The product of the sums of the qualities, with the gains of the options that fit alone, is now a bound only: when it overflows, the scores with those options are computed, and only one that is not finite refuses the instance. Two unit tests, a source and a target of 1e200 that never connect and a gain of 1e155 over the budget; 73 tests; the capture as for 0f49bb7.
- 2026-11-15, branch `criterion-scale` (on `numeric-focus`): the scale of the criterion, the task of item 1.
  - `compute_criterion_scale`: the power of 2 that brings the size of the criterion to the largest score of the cases it reads; 1 when they are within a factor 2^10, and at most 2^±100. The size is that of its terms, a score of a case times the constants above it: the largest term of a sum, the smallest of a minimum, which holds its value, a product of several formulas that read cases as a whole; a constant added is left out. A linear criterion whose cases have one weight w, a single case or their sum, has the ratio 1 / |w| at every solution, and its scores are not computed. Otherwise both are taken where nothing is bought and with every option that fits alone in the budget, which bound them at every solution, and the factor is the middle of the two ratios: when the two grow by at most 2^20 together from one point to the other, the criterion of every solution, multiplied, is within 2^10 of its largest score; when they grow more, or the criterion is 0 where nothing is bought, the factor is 1, as on main, since none would serve every solution.
  - `formula_scale_visitor` puts the factor on each case, so that each row of a minimum gets the size of the scores too: `[c, z]`, or c first in `[w, z]`. The MIP solvers' visitor now takes a product of literal constants and one other factor wherever the constants stand; on `[w, f]`, f reading cases, it builds the rows of before, and `[w, k]` of two constants is one fixed variable where it was two. Powers of 2: the criterion of the instance given to the algorithm is exactly c / s times that of the raw instance, s the square of the scale of the qualities, and with c = 1 it keeps its writing. `prepared_instance` has two scales, `score_scale`, s / c, for the scores of the criterion, and `case_score_scale`, s, for those of the cases. `-v` logs the factor.
  - The scores with the options that fit, which the test of the qualities of e4f51c0 computes only when its bound overflows, give the factor too: computed once, and only for a criterion that is not linear of one weight.
  - The mechanism: weights of 1e-8 and 1e-10 put the criterion near 1e-3 and 1e-5 in the unit of the solver, under its absolute tolerances, and the duals of the rows of the cases near 1e-8, under the margin of 1e-6 of the pricings of `path_lp` and `tree_lp`.
  - Four rules before this one, each broken by a reader, each case a unit test now. The value of the criterion where nothing is bought counted its constant (z + 1e8 got 2^-22, wrong optima), and a case at 0 there and heavily weighted got 2^40 (Gurobi: unbounded model). The largest term with every option missed a minimum held by a tiny term on cases alike (min(z, 1e-10 z'), wrong optima as on main), and counted the options over the budget (a case that only they improve set a tiny factor, wrong optima where main is right). The size with the options that fit alone counted options that exclude one another, by a conflict set, the budget or a second cost (a case that only both improve set 2^-40, wrong optima where main is right; the reverse, 2^33, an unbounded model). The first rule rounded the ratio before comparing it with 2^10, so 2^9.5 was enough; without a cap, c near 2^1000 first in a product overflowed before a tiny constant after it.
  - Cost: none for a single case or a sum of cases (static on a 250 × 250 grid where each option adds 15,625 sources, 15.8 s on main and here; computing the scores there took 9.7 s more); for the others, one evaluation with the options that fit, at most 0.04 s on the instances of the benchmark.
  - Which instances it changes: of the instances of the repository, the four of the benchmark whose criterion weighs a case of score near 1e6 or more by about 1e-6, the species ones by 1 over its initial score as instance.md recommends, the bike-and-squirrel ones in an affine term: factors 2^20 (`species-natural`) and 2^21 (`species-periurban`, `bike-squirrel-town`, `bike-squirrel-suburb`). None of the tests, docs examples or captured instances.
  - Checks: 74 tests; 837 cases identical to the capture of 20a1f6e but the 11 listings of 0f49bb7; `check.py` gets four instances: the squirrel of `two-species.json` and a copy 1e8 and 1e10 times larger, weighted by 1e-8 in a sum and 1e-10 in a minimum, the two 1e5 times larger under a minimum held by its term of weight 1e-10, and a case weighted by 1e12 that only an option over every budget improves. `check.py` whole passes them, 1699 runs, 0 failures. Before the scale, 28 of the 186 runs on the first two failed (`flow_mip` optimal on 18.4 where the optimum is 35.4, the bounds of `path_lp` and `multiflow_benders` under the optimum); the third and fourth fail under the rule of a69c2e1. The case of options that exclude one another is a unit test only: weighted by 1e12, it failed 9 runs under the rule of 09757fc, but this build needs `--numeric-focus` for the Benders from 1e11 (main had it); by 1e10, HiGHS fails as on main; by 1e9 or less, the optima of 09757fc survive. With it at 1e10, `check.py` ran 1792 runs, the 3 failures being HiGHS on it.
  - The eight runs of the short suite on the four instances, 30 s each, against those of §9.6 without NumericFocus and without the scale. `multiflow_benders` has the same optima where both are optimal, and `bike-squirrel-suburb` at 200, the run that NumericFocus was needed for, is optimal again (15.7 s, at the limit without). At the limit the gaps go either way: `species-periurban` at 2500 0.013 % (0.037 % without), at 6000 1.13 % (0.31 %), `bike-squirrel-suburb` at 500 4.02 % (4.35 %). `flow_mip` reaches the limit on all eight in every version, and whether it has a bound at 30 s depends on its root relaxation ending just before or after the limit: on `bike-squirrel-suburb` at 200 it ends at 30.2 s with the scale and 45.9 s on main, and `species-natural` at 6000, with the same factor, had a bound in one run of this branch and none in the next. One run each.
  - Found, not done: a criterion dominated by an added constant, as z + 1e12, where the relative gap of the solver spans the gains. `multiflow_benders` ends optimal at 1000000000021 for 1000000000028.8, as main does (with `--numeric-focus`, 1000000000029), and the masters of `tree_lp` and `path_lp` are not solved, as on main. The fix would be to take an added constant out of the criterion given to the solver, and add it back to its scores. A product with a factor that is a formula without cases, as product(product(2, 3), z), is still refused by the MIP solvers, as on main. A weight of 1e11 or more on a case leaves coefficients that far apart: Gurobi's Benders masters then need `--numeric-focus`, which main had always on, and HiGHS fails from 1e9, as on main.

## Rulings of 2026-11-15, on the findings of the scale

Both branches were squash-merged into main (2c7bb6b, 91c7aae). On what they left found and not done, the owner ruled:

1. A criterion dominated by an added constant is not changed: a warning is printed.
2. The qualities are scaled up as well as down; the exponents, as the factors are powers of 2, may be stored as signed integers (left to me; done).
3. `product(product(2, 3), z)` is fixed if it is easy, as by a check after the reformulation; otherwise it stays refused.

- 2026-11-15, branch `scale-exponents`: the three.
  - `reformulation` holds `quality_exponent` and `criterion_exponent`: the qualities of the instance given to the algorithm are those of the raw instance times 2^quality_exponent, the scores of its cases times 2^(2 quality_exponent), its criterion times 2^(2 quality_exponent + criterion_exponent). `formula_scale_visitor` works on the exponents and `std::ldexp`; where the factor was a division by a power of 2, the values are those of before.
  - The exponent of the qualities is −⌊log2(M / 1e5) / 2⌋, M the largest initial score of the cases read, up as down. Up, it is at most 100, and never so far that the bound on a score of a solution, the product of the sums of the qualities with the gains that fit, passes 4e5, nor a sum of qualities 2^500. From the initial scores alone, a case that the options raise from 1e-6 to 18 ended near 5e12, where `multicut_benders` failed and main is right: found before the report, on instances whose sources and targets an arc of probability 0 links until an option raises it. Down, as on main. Changed: every instance whose largest initial case score is under 1e5. In the benchmark, `dks` (2^9), `faircover` (2^6), `cover-40` (2^5), `cover-80` (2^4), `urban-greening` (2^2) and `rural-planting` (2^1); `urban-greening-large`, bounded, is left as on main.
  - `simplified_criterion` replaces each formula that reads no case by its value, the number that the criterion computes for it; the MIP solvers' test of a product runs on the reformulated criterion, so `product(product(2, 3), z)` reaches them as `product(6, z)`.
  - `prepare_instance` warns when the value of the criterion where nothing is bought, the options bought statically applied, is more than 2^20 times its size: the margin of 1e-6 of the check of a bound, relative to the whole value, is then the size of its terms. None of the instances of the repository warns but the reproducers.
  - Checks: 76 tests (the exponents up and down, the cap, the bound on the scores of a solution, the limit by the sums of qualities, the folding; a test that compared a reformulated optimum with the raw one directly now converts it). Against the capture of main, 717 of 837 cases differ, the small instances being scaled up now. No status changes. The exact algorithms change selections only among optima of the same score (`flow_mip` 12, `flow_benders` 10, `multicut_benders` 6, `multiflow_benders` 5, on `aix30` at 8 and `dks30`), but for `flow_mip --lp-relaxation` on `dks30` at 3, whose relaxation keeps its value 4.0286 and gives the selection of score 2 where it gave 4.0286. `greedy` and `static` change nothing. The bounds of `gecot bound path_lp` move in their last digits, tighter (30.42000754 to 30.42000054 on `conflicts`), as its absolute margins are relatively smaller; the solutions of `path_lp` move either way. Three documented numbers on the three woods followed: the bound of `path_lp` at 3 in first-run.md, the share and the bound of the JSON sample of results.md. `check.py` compares up to 1e-6 of a value under 1 too, where it allowed 1e-6 absolute, and gets three instances. The squirrel with scores near 1e-9: main's `multicut_benders`, `flow_benders` and `multiflow_benders` end optimal on 1.84e-9, the score of buying nothing, for optima of 2.56e-9 to 4.244e-9. A criterion with a product of constants in a product, which main's MIP algorithms refuse. Sources and targets that options link, the score going from 1e-6 to tens, on which `multicut_benders` fails under the scale-up from the initial scores alone. 1978 runs, 0 failures; main passes the others under the relative tolerance.
  - The runs of the short suite on the instances that the scale-up changes, 30 s each, against the runs of §9.6 without NumericFocus, whose models are main's there (`dks` and `rural-planting` from a first build whose exponents for them are these). The same runs are optimal, with the same optima: 15 of 21 for `multiflow_benders`, 11 for `flow_mip`. At the limit the gaps are mostly larger. `multiflow_benders` on `cover-40` at 16: 5.9 % against 4.8 %. On `cover-80` at 16 and 24: 6.7 % and 16.3 % against 5.3 % and 13.9 %. On `faircover-80` at 16: 13.2 % against 11.1 %, and `flow_mip` there 16.2 % against 10.8 %. Smaller: `multiflow_benders` on `faircover-40` at 16, 3.2 % against 3.7 %, and `flow_mip` on `cover-80` at 24, 18.5 % against 19.9 %. `flow_mip` on `urban-greening` at 15000 ends at 30 s with 2032.6 and no bound, where main has 3261.1: the root bound is the same, and only Gurobi's heuristics at the root find another solution, in two runs each; at 150 s both reach 3261.137. Repeated runs give the same numbers: the differences are those of the scaled models, not noise. The scale-up buys correct optima on scores near 1e-9, not speed on these.

For the owner: nothing ruled is left on the branches. Not ruled, so not started: the parser (PR), the static library (L1), O3 to O5, the README, Windows and the releases. Found, not done: SCIP's numerics emphasis needs `SCIPsetEmphasis` in mippp's table, and MOSEK's setting could be set by its name with `MSK_putnaintparam`, which mippp does not load either; a weight of 1e11 or more on a case, for which Gurobi's Benders masters need `--numeric-focus`, on by default before; the check of a bound keeps a margin of 1e-6, not 1e-6 of the score, under a score below 1.
