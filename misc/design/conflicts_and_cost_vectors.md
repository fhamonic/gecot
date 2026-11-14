# Conflict sets and cost vectors: design note

This note studies two extensions of gecot: **conflict sets** of options, at most one option of each set bought (SOS1 on binaries), and **cost vectors**, one budget per cost dimension. It is written for the owner and for whoever implements them. It is a study only: nothing is implemented, and main is at e2585fc (2026-11-13).

- **Sources.**
  - Six read-only passes over the code: instance and parser; command layer; MIP and Benders; preprocessing; `path_lp` and `tree_lp`; heuristics and bounds.
  - After each pass, a skeptic tried to refute every claim of the form "stays correct without a change".
  - A completeness critic, then six passes on the gaps it found.
  - A literature search.
  - About 230 code sites were classified.
- **Evidence marks.**
  - **[read]** means read in the code, and is the default.
  - **[toy]** means a Python or scipy model of the cited lines or of the formulation, run in a scratchpad, not gecot.
  - **[gecot]** means run with the prebuilt `build/gecot` of 2026-11-13 04:30. Nothing was rebuilt.
  - Every [toy] figure below was run a second time for this note and reproduced, with three exceptions run only once:
    - the LP column-generation runs of §5.2 (300 seeds);
    - the max-aggregate count of §5.5;
    - the B + 1e-7 == B observation of §4.

    The gap passes had no skeptic of their own, so this rerun is their check.
- **Notation.** The docs already use S, d and superscript indices for other things, so this note uses:
  - cost dimensions r ∈ R, option costs c_{o,r} ≥ 0 and budgets β_r ≥ 0;
  - conflict sets Ω_1 … Ω_L ⊆ Φ.

## Summary

- **Both extensions are packing rows.** Each is a row A x ≤ b with A ≥ 0:
  - a budget dimension r is Σ_o c_{o,r} x_o ≤ β_r;
  - a conflict set is Σ_{o∈Ω} x_o ≤ 1.

  Under "at most one", the feasible set stays down-closed and the score stays monotone.
- **Every bound and every preprocessing conclusion stays valid without change, with one exception.** The exception is the `--reformulation path_lp` cutoff (item 6 below), the only conclusion that needs a feasible incumbent. A conclusion that holds for every x of a superset of the feasible set holds for the feasible set. This covers the strong and useless arcs, the constrained labels, the knapsack and tight big-Ms, the Benders cuts and seeds, `all_options`, `lp_knapsack`, `qkp_planes`, `split_lagrange`, and `path_lp`'s Lagrangian bound and option bounds. None of the seven preprocessing headers and none of the bounds breaks; they only get looser. No skeptic found a counterexample to any of these claims.
- **What breaks is everything that builds, keeps or returns a selection, and the reformulation in front of the algorithms.** Grouped:
  1. `trivial_reformulate_instance` buys every cost-0 option statically. It also copies one scalar cost and no conflict set, and `restrict_instance` does the same (trivial_reformulate.hpp:78-87, 101-116). Every algorithm then runs without the new rows.
  2. `optimize.cpp:266` reports every raw cost-0 option as bought.
  3. The MIP masters need the rows (flow_mip.hpp:57, multicut_benders.hpp:201, flow_benders.hpp:92, multiflow_benders.hpp:365).
  4. The start checks test the scalar budget only: `_fallback_solution`, path_lp's start guard and tree_lp's start guard (mip_solver_base.hpp:333, path_formulation_lp.hpp:496, tree_formulation_lp.hpp:325).
  5. path_lp's fill (path_formulation_lp.hpp:913-918) and tree_lp's cast of x ≠ 0 (tree_formulation_lp.hpp:466) ignore the new rows.
  6. The `--reformulation path_lp` cutoff is built on that fill (path_reformulation.cpp:52). It is the one preprocessing conclusion of the "there exists a feasible x" kind, so it does not transfer.
  7. Greedy and static, in both modes. The decremental modes start from all options, which violates every conflict set.
  8. `optimize.cpp:257` takes max(bound, score) without a guard, so it can certify an infeasible selection.
  9. `benchmark/compare.py:82-84` adopts any score as the best known.
- **Use linear rows, not native SOS1.**
  - On binaries, the row is the convex hull of "at most one", while SOS1 leaves the LP relaxation at the box and acts only through branching.
  - Rows keep multiflow's root phase and `--lp-relaxation` real LPs.
  - Rows work on all 9 `--solver` choices.
  - mippp 1.1.0 declares `has_sos1_constraints` (model_concepts.hpp:570) but no backend implements `add_sos1_constraint`. I checked by grep: only the C API declarations exist (`GRBaddsos`, `CPXaddsos`, `COPT_AddSOSs`, `Cbc_addSOS`).
  - Gurobi's documentation recommends linear constraints over SOS for this structure.
  - The closest ecological precedent writes the same linear row (King et al. 2017).
- **Data model.**
  - Store the two kinds typed in `Instance`:
    - named cost dimensions with a cost vector per option;
    - conflict sets with an inverse index from option to sets.
  - Derive a sparse view `packing_rows(instance, budget)` for the consumers that only need rows: model rows, feasibility and multipliers.
  - Pass the budget as a struct with no conversion to `double`, and delete the scalar `option_cost(o)`. Every budget and cost site then fails to compile until it is revisited (by reading; not compiled).
  - Keeping `double budget` for the first dimension, with the others in the instance, leaves 16 sites that compile and silently return infeasible selections, plus the two remaps (§2.2).
  - Conflict sets get no help from the compiler: the two id remaps must carry them through one shared helper.
- **Free options.** Buy an option statically only if its column is zero: cost 0 in every dimension and member of no conflict set. A cost-0 member of a set stays a decision variable. The two tempting alternatives are refuted on `docs/examples/three-woods.json` (§3.1, recomputed [toy] and cross-checked against gecot).
  - Because of this rule, cost-0 decision variables reach the algorithms for the first time:
    - greedy then computes inf and NaN ratios;
    - static sorts NaN keys, which is undefined behaviour;
    - `knapsack_bnb`'s search can blow up;
    - the constrained μ-grid becomes a silent no-op.

    So the rule and its consumers must land together (§3.2).
  - The rule fixes correctness, not quality. Any ratio greedy, today's included (gain/0 = +inf), buys a free set member first and so locks out its set-mates. On three-woods at B=5 greedy ends at 28.76, the refuted row of §3.1. An upgrade move inside the set recovers the optimum 31.04 [toy] (§5.3).
- **Effort: my aggregate of the readers' per-subsystem estimates**, not a measured figure. It includes three items one skeptic found missing from the first estimates: the main remap, the cost-0 handling and the benchmark flag.
  - Correctness for conflicts: about 400-600 lines plus tests, mostly instance, reformulation, heuristics and the feasibility helper.
  - Cost vectors: about 300-400 further lines, mostly mechanical: about 20 signatures, the parser, the CLI and the outputs.
  - Tightenings: optional, each 10-60 lines, each to be measured (§6).

  The owner decisions are listed in §8.

## 1. The problem and the readings

```
max  C((PC_num(G_k^x))_k)
s.t. Σ_o c_{o,r} x_o ≤ β_r      for every dimension r ∈ R    (today |R| = 1)
     Σ_{o∈Ω_l} x_o ≤ 1          for every conflict set l      (new)
     x ∈ {0,1}^Φ
```

- **Conflicts: at most one**, and sets may overlap: a pairwise conflict is a set of two.
  - "Exactly one" makes the feasible set non-down-closed. §7 lists what changes.
  - "Disjoint only" simplifies the storage and enables the multiple-choice knapsack tightenings (§6).
- **Cost vectors: several hard budget rows over one purchase vector.** Per-region and per-year budgets are dimensions whose costs are 0 outside their region or year.
- **Readings left out.**
  - Costs as extra objectives.
  - A weighted sum of costs as the only budget, as Marxan with Zones does. That is a scalarization, useful only as the greedy ratio (§5.3).
- **When a conflict row binds.** Improvements of one arc already compete: the best probability counts (problem.md).
  - Alternatives nested on one arc, such as removal against a fish ladder, never gain from being bought together, so the row does not change the optimum. It only forbids wasteful purchases, which greedy makes because it buys while anything fits (heuristics.md).
  - The row changes the optimum in three cases:
    - for alternatives on vertex qualities, whose gains add up;
    - for alternatives spread over several arcs that are not nested;
    - for alternatives serving different cases.
  - The `species-*` benchmark instances have the third kind: at each crossing site, a crossing for every species and a rope bridge for the arboreal ones (benchmark/README.md:300-318).
  - The user docs should say this, so that users do not over-constrain.

## 2. Data model, formats, command line, outputs

### 2.1 Instance and concept

- **Costs.** Store them per option, or per dimension with `num_options()` anchored on `_options_names.size()`.
  - Trap: with per-dimension storage and `num_options()` unchanged (src/instance.cpp:102), `num_options()` returns |R|. Every option map and every option id is then wrong, silently, even for d = 1 files. Only parse_instance_test.cpp:40 would catch it.
  - Add `num_cost_dimensions()`, `cost_dimension_name(r)` and `cost_dimension_from_name(name)`.
  - `option_cost(o)` returns the vector, or becomes `option_cost(o, r)`.
  - Delete `set_option_cost`, which has no caller.
- **Conflict sets.** A list of sets: CSR offsets plus members, or `vector<vector<option_t>>`. Add the set names for messages, an inverse index `option_conflict_sets(o)`, and `add_conflict_set(name, members)`.
  - For disjoint sets only, one `optional<size_t>` group per option suffices.
- **`instance_c` (concepts/instance.hpp:105-119).** Require the new accessors rather than make them optional: an instance type that silently reports no conflicts reproduces every break of the summary. `Instance` is the only model today, so the change is cheap.
- **Generic rows versus typed storage.** Typed storage, with a derived row view.
  - The budget's right-hand side comes from the command line, while a conflict's right-hand side is the constant 1 from the instance.
  - Several consumers use the unit structure: O(degree) set-mate tests, the multiple-choice knapsack, Q zeroing, clique merging.
  - The tolerance differs too: relative per budget dimension, exact count for a conflict.

### 2.2 Where the budget vector lives

About 20 signatures take `double budget` today: `AbstractSolverInterface::solve`, `AbstractBoundInterface::compute`, `relax`, `solve_path_reformulated`, `prepare_instance`, `with_mip_start`, and so on. The gap pass compared three designs:

| Design | Sites that compile unrevisited and return an infeasible selection |
|---|---|
| (i) a `budget_t` struct, no conversion to double, `option_cost` returning the vector | **0**, by reading (not compiled). Every use of β is arithmetic, a comparison, a mippp `<=` with a double right-hand side, or a `{:g}` format, so it fails to compile. |
| (i-b) as (i), but keeping a scalar `option_cost` = c_1 | 1 certain break (optimize.cpp:266 prints an option of cost (0,5) as bought). Also 5 places where only the β side fails to compile, inviting a mismatched (c_1, β_j) fix that makes a label, big-M or bound invalid. |
| (ii) keep `double budget` = β_1 and put β_2…β_d in the instance as rows | **16**:<br>- the 4 MILP master rows;<br>- tree_lp's master row;<br>- greedy (4 sites: :108, :168-176, :205, :225-231) and static (2 sites: :109, :173);<br>- path_lp's fill and start guard;<br>- `_fallback_solution`;<br>- the free test;<br>- `option_value`.<br><br>Also the two remaps, which drop the extra rows under every design (18 in all). |
| (iii) β stored in the reformulated `Instance` | Viable only under five conditions:<br>- every builder can set β;<br>- an unset β throws;<br>- `restrict_instance` copies it;<br>- path_lp's `_pairs` and `_pair_columns` keep an explicit capacity;<br>- the concept gains `budget()`.<br><br>It also brings two silent hazards and wider test changes. |

Recommendation: (i).
- Use a struct, not `std::vector` or `std::span`. libstdc++ 15 formats ranges, so the JSON `{}` sites would compile and print `[10]` at d = 1 (optimize.cpp:349-351, bound.cpp:252-253).
- With a vector type, CLI11 would also turn `-B` into a multi-value option, which takes the method's passthrough values (cli_parity V01-V03, B13).

### 2.3 Instance format

| | Recommended | Alternative |
|---|---|---|
| cost dimensions | `cost` as an object of named dimensions: `"csv_columns": {"cost": {"money": "cost_eur", "labor": "days"}}` for a CSV table, `"cost": {"money": 2, "labor": 1}` inline. No object means one dimension named `cost`, so every current file is d = 1. | a long table (option_id, dimension, cost) with 0 when missing, for many sparse dimensions such as one budget per region |
| conflict sets | a top-level `"conflicts"` table of (set_id, option_id) rows, as a `csv_file` or inline, in the existing table idiom with `csv_columns`. Sets may overlap, and a pairwise conflict is a set of two. | a `group` column in the options table, for disjoint sets only |

- **Format choice decides how older binaries fail.** Prefer formats that older binaries refuse loudly.
  - The old schema has `additionalProperties: false` at the top level (parse_instance.cpp:385), so a `"conflicts"` key is refused. An object in `csv_columns.cost` or an object-valued inline `cost` is also refused.
  - Older binaries silently ignore three things:
    - a `group` column or extra cost columns read by default name, because unread CSV columns are ignored;
    - extra keys in inline option items, which have no `additionalProperties` (parse_instance.cpp:36-51).

    They would then return solutions that violate the new rows.
- **Implementation trap.** `csv_columns.cost` as an object collides with `parse_columns_aliases` (parse_instance.cpp:396), which assigns the value to a string and throws a nlohmann `type_error`. It needs its own parsing.
- **Validation.**
  - Reject a duplicated (set, option) row, or deduplicate it. Lowered to a row it is a coefficient 2 against a right-hand side of 1. Then the generic rule "drop o if a_{r,o} > b_r", or the MIP row, forbids a buyable option. On three-woods it loses the optimum (35.40 to 28.76) [toy].
  - An empty set is an error. A singleton set is vacuous under at-most-one: warn or accept.

### 2.4 Command line

- **Today's parsing.** [gecot] `gecot optimize greedy three-woods.json 10,5` runs with budget **105**, and `1,000` with 1000. CLI11 strips `_`, `'` and the C locale's group separator `,` before parsing (TypeTools.hpp:1183-1190). A French decimal comma `2,5` is therefore read as 25 today.
  - A repeated `-B` fails: "At most 1 required but received 2".
  - `money=10` fails with "Could not convert".
- **Proposal.** Keep the positional budget as one string token, parsed by hand without group separators:
  - a bare number for d = 1, exactly as today;
  - `name=value,…` or a list in the dimensions' order for d > 1.

  Check the arity and the names in `prepare_instance`, after parsing. Rejecting `10,5` for d = 1 deliberately breaks the accidental behaviour, and catches the French decimal comma.
- Never make the positional a CLI11 vector: it would take the method's passthrough values.
- **Validation per dimension.** Each β_r must be finite and ≥ 0 (optimize.cpp:160, bound.cpp:136).
  - β_r = 0 stays legal, so any ratio or weight that divides by β_r must skip that dimension.
  - Whether a missing dimension is unconstrained or an error is an owner decision.

### 2.5 Outputs

- **Format.** Keep the scalar `budget` and `solution_cost` keys and the text lines when d = 1, so that cli_parity's "files same" holds. For d > 1, an object keyed by dimension name, or an array.
- **Log line.** The budget-share line (common.cpp:190-196) becomes one share per dimension. Under conflicts it overstates how tight the budget is: with Ω = {a, b}, c = 10 each and β = 10, it prints 50 % although the budget never binds.
- **Feasibility in the JSON.** Write a `feasible` flag, or the list of violated rows, in the JSON.
  - `compare.py` takes the maximum score with no cost, status or feasibility filter (compare.py:82-84), and `--save-best` ratchets it into the tracked `best_known.csv`. This is already latent today with tree_lp's over-budget casts.
  - `run.py` must carry the flag, and `compare.py` must skip infeasible runs.
- `parse_mip_start` reads only `solution`, so the MIP-start round trip is unaffected.

## 3. The reformulation

### 3.1 Free options

Today `trivial_reformulate_instance` applies every option of cost 0 to the cases and drops every option whose cost exceeds the budget (trivial_reformulate.hpp:78-87). So every reformulated option has 0 < c_o ≤ B, an invariant that several consumers rely on (§3.2).

On three-woods, make `plantation` free and declare the conflict {plantation, underpass} ([toy], recomputed for this note; the evaluator matches gecot: initial score 18.4, unconstrained B=5 optimum 35.4 {plantation, underpass} [gecot]):

| rule | B = 3 | B = 5 |
|---|---|---|
| true optimum | {hedgerow, plantation} 28.76 | {hedgerow, underpass} 31.04 |
| today: buy the free option, ignore the conflict | {plantation, underpass} 35.40, **infeasible** | {all three} 42.44, **infeasible** |
| buy the free option and delete its set-mates | 28.76 | {hedgerow, plantation} 28.76, **suboptimal** |

The correct rule takes one ordered pass:
1. Drop the options over the budget in any dimension (c_{o,r} > β_r for some r).
2. Restrict every set to the surviving options, and drop the sets with at most one member.
3. Buy statically exactly the options whose column is zero on every remaining row: cost 0 in every dimension and member of no remaining set. The other cost-0 options stay variables.

Points on that pass:
- **Vacuous rows.** Dropping the budget rows that the surviving options cannot exceed is valid. Applied to d = 1, it changes today's behaviour: on three-woods at B=7 every option becomes static, the run ends in "no option left" with a false warning, and the guaranteed share is not printed.
  - Apply it to conflict rows only, and to budget dimensions only while another row remains. Alternatively, rule on it.
- **Fixpoint.** Rules 1-3 reach a fixpoint in one pass. Any later reduction that shrinks sets needs an outer loop: dominance between set-mates, or options that improve nothing once the probability-1 components are merged (trivial_reformulate.hpp:162-194). Such a reduction can make a set vacuous, which creates a new static option, which can create new probability-1 arcs.
- **The printed selection.** `optimize.cpp:263-269` must read the static set that the reformulation returns, not `raw_instance.option_cost(o) == 0`. Otherwise:
  - a free option that the solver did not choose is printed as bought, together with the set-mate it did choose;
  - a set made vacuous by the drops prints its free member as not bought, while the score includes its gains.

  Return the static set in `prepared_instance` (common.hpp:48-58). If it can contain options of positive cost (step 3 after a drop), the printed cost must sum raw costs over the printed selection.
- **One remap helper.** `restrict_instance` (the `--reformulation path_lp` path) needs the same remap of sets and cost vectors. Make it one helper used by both builders.
  - `restrict_instance` applies nothing statically (:115), so cost-0 variables reach the inner solver there even after a perfect reformulation.
- **`base_score`** (owner decision). It is the score of the reformulated instance with nothing bought, and it now excludes the free options inside sets. results.md:38 says it is "the score without any paid option".
  - On three-woods with hedgerow and plantation free and conflicting, underpass at 3 and B=3, the guaranteed share of the same run prints 74.3 % under the first reading and 59.6 % under the "best free-only feasible score" reading [toy].
  - The check bound ≥ base_score (bound.cpp:195) stays valid by monotonicity.

### 3.2 Cost-0 decision variables downstream

Once cost-0 options can be variables (inside sets under the rule above, anywhere after `restrict_instance`, or on one dimension when a consumer projects a cost vector):

| consumer | outcome |
|---|---|
| greedy.hpp:102-104, tie window | 1/0 = inf: every lower-id candidate counts as tied with the best, so greedy buys a finite-ratio option instead of the free one. Example [toy]: o0 (c=1, g=1), o1 (c=1, g=10), free o2 (g=5), B=1 gives {o0, o2} 106 against {o1, o2} 115. |
| greedy.hpp:114-117, 186-190, ratios | +inf, NaN (0/0) or −inf. A NaN in `min_element`/`max_element` breaks the strict weak order (UB). |
| greedy.hpp:128, 203, score update `ratio × cost` | inf·0 = NaN poisons `previous_score`; every later ratio is NaN and greedy picks in list order. |
| static.hpp:96-106, 160-169 | NaN keys in `std::ranges::sort`: UB. [toy] port of libstdc++ 15.1: one NaN key misorders the finite keys in 1,934 of 3,000 runs. +inf keys are fine. |
| compute_constrained_strong_and_useless_arcs.hpp:96-115 | A cost-0 option that shortens an arc gives max_mu = +inf. It is clamped to 2^62/max(B,1), min_mu is lifted to cap/1000, and every positive pass becomes a no-op. The labels stay valid, but the budget-aware gain is lost silently. Fix: skip `arc_cost == 0` in the grid loop. |
| melon `knapsack_bnb` (knapsack_bnb.hpp:86-87) | The value is exact, but the search blows up: ratio +inf, and backtracking explores every "without i" branch unless v_i is at least the gap. [toy] port: 52 → 48,133 backtracks for 0 → 10 small cost-0 items. Moving the positive-gain cost-0 items into the constant M (compute_big_M_map.hpp:114-118, compute_tight_big_M_maps.hpp:188-196) is exact against the conflict-blind knapsack, and only looser against a set-aware one. |
| compute_tight_big_M_maps.hpp:118-129 toll, qkp_planes, lp_knapsack, split_lagrange, tree_lp pricing row, path_lp label dominance | fine. path_lp's early break (path_formulation_lp.hpp:180-197) stays exact: dominance implies e.weight ≤ l.weight, and the break tests a strict `>`. |

No test generator draws cost 0 (bounds_test.cpp:104, 141): add one.

## 4. One feasibility helper, its call points and tolerances

`violations(instance, selection, budget)` tests every budget dimension within its tolerance and counts each conflict set exactly. A separate incremental `fits(o, residuals)` keeps a residual per row, for the fills and the heuristics.

Every selection a user sees passes through `solution` at optimize.cpp:222-233, then `option_value`, then text, JSON and CSV. The minimal set of check points:
- **(a) One output gate** in `optimize`, after :246 and before :257, on the printed selection over `raw_instance` (so static purchases count).
  - It warns, as tree_lp's cast stands by ruling, and writes the JSON flag.
  - It replaces the scalar warning at :259.
- **(b) The three start guards** keep their policies and only swap the predicate:
  - `_fallback_solution` (mip_solver_base.hpp:333) throws, and must: the fallback returns the start;
  - path_lp (:496) warns and ignores, and must: :924-925 can return the start;
  - tree_lp (:325) warns and ignores. It never returns its start, but while the cast of x ≠ 0 stands the guard is still required: a conflicting start column can make the master's unique optimum x = (½, ½) on a set, and the cast prints both members (§5.2). With a per-row fill instead of the cast, the guard is for tightness only.
- **(c) `optimize.cpp:257`.**
  - Absorb only score − bound ≤ 1e-9·max(1, |score|) on a gate-accepted selection, as `algorithm_bound_interface.hpp:55-59` does.
  - Otherwise keep the raw bound and print no share.
  - Today an infeasible start returned by multiflow's fallback, after a root round with Ctrl-C, or an infeasible path_lp fill, prints "at least 100.0 % of the best possible gain".

Notes on the check points:
- `with_mip_start` is not a single point.
  - A check before its early return (abstract_mip_interface.hpp:109) misses the `--mip-start-algorithm` start built at :114-119.
  - `StaticInterface` and `GreedyInterface` never call it.
  - The path reformulation's own selections bypass it.
- Not needed: a check at path_reformulation.cpp:52, once path_lp returns feasible selections.
  - The option bounds hold for every x, so they stay valid whatever the incumbent is.
  - An infeasible incumbent that scores above the optimum is never displaced (:86) and reaches gate (a).
  - Only the early "optimal" at :65-66 would be wrong, and the gate flags it.
- **Two tolerances, two roles.**
  - *Acceptance* uses the solver's absolute `-t`, one per dimension (default 1e-7: abstract_solver_interface.hpp:76-79). It covers the start guards, the fallback, path_lp's enumerate filter, the fill and the heuristics.
    - path_lp passes `budget + feasibility_tol` to `_pairs` (:505), so the filter, the start guard and the fill share one threshold today. Keep that coupling per dimension: a filter stricter than the fill puts L below the returned score.
  - *Audit* uses 1e-6·max(1, β_r), as at optimize.cpp:259, and only warns.
  - Conflicts: always an exact count.
  - [toy] B + 1e-7 == B once B ≥ 2^30, so an absolute tolerance vanishes on a dimension in m² of that size. That is an argument for the relative audit.

## 5. Per subsystem

### 5.1 flow_mip and the three Benders

- **Rows.** One helper in `mip_solver_base` adds every packing row: the d budget rows and one `xsum(Ω, X_vars) <= 1` per set. Call it at the four budget-row sites.
  - The rows must be static master rows, not lazy ones:
    - multiflow's root phase (multiflow_benders.hpp:459-501) has no callback, so lazy rows would be missing from every root LP;
    - flow_mip has no callback at all;
    - a row is the convex hull of "at most one";
    - solver heuristics and starts would otherwise be checked only through the callback.
  - The injection contract (model_concepts.hpp:838-849) is not a reason. A callback can simply not inject a violating candidate (the MIP skeptic's correction).
- **Unaffected [read; skeptic: holds].**
  - The Benders optimality cuts: their duals do not depend on x (benders.md:35).
  - The seeds at ∅ and at the start.
  - The multiflow in-out center, a convex combination of LP optima, so it satisfies the linear rows (and cut validity does not need it to).
  - The `add_solution` candidates, which are master candidates rounded at 0.5 and so inherit every static row.
  - The knapsack and tight big-Ms, computed over a superset.
- **Fallback.** `_fallback_solution` must test every row (§4).
- **`--lp-relaxation`** of flow_mip (:291-296) rounds x ≥ 1 − tol. It is feasible under at-most-one.
- **Not verified:** how Gurobi, CPLEX and COPT treat a complete MIP start that violates a row (reject or repair). gecot can repair a start under at-most-one by dropping set members.

### 5.2 path_lp and tree_lp

- **Master rows.** Add d − 1 more budget rows and one row per set: path (:550) and tree (:208). These are the only model changes.
- **Pricing and bounds** [read; toy LP column generation on 300 seeds, qualitative].
  - Neither pricing reads the budget dual: y columns have no entry in rows on x only.
  - L = z_RMP + Σ_p max(0, max_S rc_pS) absorbs extra rows through z_RMP.
  - The option bounds (path :834-844) stay valid under at-most-one even if the new duals are left out.
  - The dual row of x_o becomes Σ_p λ_po ≤ Σ_r c_{o,r} μ_r + Σ_{l∋o} ν_l.
  - path_lp's master has no x_o ≤ 1 row (path_lp.md §1.2). That is harmless while every option has a positive coefficient in some row, and conflict rows keep it so for cost-0 members.
- **Enumerate filter** (:303-305, :396): test every dimension, and drop subsets with two members of one set. That is valid, and tighter: every integer solution's columns are conflict-free.
  - The product count of conflict-free subsets, ∏(|Ω ∩ Q| + 1), lifts the 20-option limit for alternatives at one patch.
- **The fill** (:913-918) must keep a residual per row. It skips o if a set of o already has a bought member, or if any dimension would exceed β_r + tol_r.
  - Rows in the master do not repair the fill. [toy] with x from a master that already holds every row, the fill still buys {a, b} of one set, or an option over dimension 2.
- **tree_lp's cast of x ≠ 0 (:466) cannot be made feasible by rows alone.** Evidence: a scipy/HiGHS model of the converged master [toy, rerun], and a derivation by hand.
  - Setup: two disjoint sets {a, b} and {d, e}, unit costs, B = 4. Four targets each need one of the pairs {a,d}, {b,e}, {a,e}, {b,d}, value 1 each. Every tree is conflict-free, and both set rows are in the master.
  - The cast buys all four at cost 4 ≤ B, so the over-budget warning stays silent. Both sets are violated, and the support scores 4 against an integer optimum of 1.
  - Derivation. A target's value is at most the min over the options its trees need, by the per-target convexity row and the purchase links. So
    LP ≤ [min(x_a,x_d) + min(x_b,x_e)] + [min(x_a,x_e) + min(x_b,x_d)],
    and each bracket is at most min(x_a + x_b, x_d + x_e) ≤ 1.
    - The value 2 needs both brackets equal to 1. The first forces x_a = x_d and x_b = x_e, the second x_a = x_e and x_b = x_d, and x_a + x_b = 1 then gives x = ½ everywhere.
    - So the optimum is unique, and no re-solve (:458-461) helps.
  - As a gecot landscape, this needs four two-arc corridors, where a or b improves the first arc and d or e the second. That is plausible but not built.
  - Overlapping pairwise sets {a,b}, {b,c}, {a,c} give x = ½ everywhere too (derived).
  - Caveat from the gap pass, a single run: with a single set and a slack budget, 1,500 random toys gave no conflict violation within the budget. The case above needs two interlocked sets.
  - So either tree_lp gets path_lp's per-row fill, or its output stays infeasible under conflicts. The fill reverses the no-rounding ruling of 2026-08-29 (owner decision, as path_lp's did on 2026-11-03).
  - With only the cast kept, the master rows, the pricing rows (:121, over `X.keys()`) and the start guard (:325) are still required. Without them the LP's own integral optimum is infeasible: x = (1,1) on a set [toy].
- **Under cost vectors only,** a fractional cast overshoots one dimension: the class of output the cast ruling accepts at d = 1. Add the per-dimension rows anyway.
- **`--reformulation path_lp`** (path_reformulation.cpp).
  - The option bounds hold for every x and stay valid.
  - The cutoff (:52) needs a feasible incumbent, so it is valid only once the fill and the start guard are fixed.
  - Today an infeasible {a, b} fill scores above every feasible selection, prunes the options of the optimum, and can return with status "optimal" (:65-66).
  - `restrict_instance` must carry the rows (§3.1).

### 5.3 Heuristics (greedy, static)

- **Incremental.** After a purchase, erase the set-mates of the bought option from the candidates: greedy :170 and :225, static :173.
  - Today, with alternatives a (c=1) and b (c=2) and B = 3, both modes buy both. Greedy keeps buying while anything fits, even at zero gain.
  - After a decremental phase, greedy re-filters the removed options by budget only (:170-176), so it buys back the removed mate of a kept option.
- **Decremental.** Both modes start from all options, so every set with two or more members is violated. With Ω = {a, b}, c = 1 and 2, and B = 3, the loop never runs and {a, b} is returned. `--only-dec` has the same defect.
  - Phase 1: while some set is over-full, remove among its members, by smallest loss.
    - For a set of two, "remove the smaller loss" keeps the member that scores better alone with the rest [read].
    - **Static must re-evaluate the losses within phase 1.** Its losses are computed once at the start, and members of a set of three can mask one another. [toy] Three alternatives on one arc, p = 0.9, 0.9, 0.5: every loss is 0.0 at the start, and the walk keeps the worst one (2.5 against 2.9).
    - Phase 1 must not walk static's existing loss/cost order, which can remove the better member with no budget pressure.
    - Exact ties will be routine, for example equal improved probabilities. `std::ranges::sort` is unstable, so add a secondary key: larger cost first, then larger id.
  - Phase 2: today's budget loop over every dimension.
    - Static must recompute its losses once at the phase-1 result, if phase 1 removed anything. [toy] Phase 2 on the stale order removes b, whose loss was masked by its removed mate, and keeps d (4.6 against 5.0).
    - Its walk (static.hpp:108) must also skip the phase-1 removals, which `options` still holds. Otherwise their cost is subtracted twice and the walk stops over the budget.
  - Under at-most-one with d = 1 and no conflicts, phase 1 never runs, so today's selections are unchanged.
- **The ratio for cost vectors** (owner decision; the default proposed here).
  - The scaled MKP efficiency Δscore / Σ_r c_{o,r}/β_r, Puchinger, Raidl & Pferschy 2010, is solver-free and lets no dimension dominate.
  - Write it with a reference dimension r₀, the first with β_{r₀} > 0:
    D_o = c_{o,r₀} + Σ_{r≠r₀, β_r>0} c_{o,r}·β_{r₀}/β_r.
  - It orders exactly as Σ_r c_{o,r}/β_r, and at d = 1 it is c_o with no arithmetic.
  - The plain c/β is **not** bitwise neutral. [toy] With c = (49, 17) and B = 59, static and greedy pick other options on specific gains, and fl(fl(g/D)·D) differs from g in 16.7 % of random draws. It also gives NaN at β = 0.
  - The current tests cannot see such a change. Every Aude cost is 1, and in a model of greedy and static, with the score update deliberately left wrong, all 18 expected sets of solvers_test.cpp still match while the score drifts by up to 9.4e10 [toy, rerun]. Parity needs a check with non-unit costs.
  - Alternatives: Senju–Toyoda weights for decremental, or LP-dual weights from path_lp, which gecot computes; dual-based efficiencies gave the smallest cores in the MKP literature.
- **Cost 0** (owner decision 11). The code needs an explicit rule, to avoid the inf and NaN of §3.2. The candidate rule:
  - incremental: rank cost-0 candidates first, by absolute gain;
  - decremental: a cost-0 option is a removal candidate only while one of its sets is over-full.

  That rule does what today's +inf ratio does, deterministically, so it is the "buy the free member, lose its set-mates" choice of §3.1. On three-woods at B=5, greedy buys plantation, loses underpass, and ends at {hedgerow, plantation} 28.76 against the optimum 31.04 [toy].

  Other ranking rules for a free set member, such as charging it the cost of its cheapest mate, are heuristics too. The upgrade move below is what recovers 31.04.
- **Score update.** Update `previous_score` by the score difference, not by ratio × cost (greedy :128, :203). Ratio × cost drifts once the denominator is a scalarized D_o, and gives NaN at cost 0.
- **Upgrade moves** (recommended together with conflicts). Replace a bought option by a set-mate when Δc > 0 has a good ratio, and take it outright when Δc ≤ 0 with a gain.
  - This is the local search behind the partition-matroid guarantee for submodular objectives, an analogy only.
  - Examples where it matters [toy]:
    - a (c=1, gain 10), b (c=5, gain 40), Ω = {a, b}, B = 6: greedy keeps a (10), and the optimum is b (40);
    - three-woods at B=5 above: the swap plantation → underpass, Δc = 3, which fits the 3 left after hedgerow, gives 31.04.

### 5.4 Bounds

All of these stay valid with no change [read; skeptic: holds]: `all_options`, `lp_knapsack` on one row, `qkp_planes`, `split_lagrange` (valid for every ν), and `criterion_analysis`. Two things need care.
- **Shared predicate.** `split_lagrange`'s budget filters at :91 (check_structure) and :608 (case_data) must stay one predicate. If only :91 becomes per-dimension, an R1 violation is hidden and the bound becomes invalid. Example: β = (1,2), and an arc improved by o1 (c=(1,1)) and o2 (c=(1,5)). Reuse the predicate in `qkp_planes` :45 and :96.
- **The Polyak aim.** The aim of `split_lagrange` (:437-442) is a knapsack rounding that ignores conflicts and the other dimensions. It can exceed the optimum, which makes the step negative. The bound stays valid, only its convergence suffers.
- **Test oracle.** The brute-force oracle (bounds_test.cpp:27) must skip masks that violate a row. Otherwise a correct conflict-aware bound fails `EXPECT_GE` against the optimum of a superset.

### 5.5 Preprocessing

None of the seven headers breaks, under at-most-one, under exactly-one, or with any valid scalar relaxation row for cost vectors [read; skeptic: 14 of 14 claims hold]. The proofs use monotonicity and the row inequalities, never down-closedness. Further, `lagrangian_prep_review.md:108` notes that the reductions only lower F_t for scenarios outside the labels' validity set, so a conflict-violating x is under-scored, never inflated.

For cost vectors, one scalar row must be passed.
- Valid choices:
  - one dimension;
  - the minimum over dimensions of the per-dimension knapsack values;
  - a surrogate Σ_r μ_r c_{o,r} ≤ Σ_r μ_r β_r with μ ≥ 0, which is exact at the LP duals.
- Invalid: any max-type aggregate of costs. Counterexample: c = (1,0) and (0,1) with β = (1,1) [toy: 831 of 3,000 random instances].
- The minimum over dimensions multiplies `knapsack_bnb`'s worst case by d. The single-row hexagon case at B=151 already took 8-20 h. The d-dimensional knapsack has no FPTAS for d ≥ 2, so do not add an exact multi-dimensional knapsack.
- Fix the μ-grid zero share (§3.2) for vector costs as well: a projection gives zero cost to non-free options.

## 6. Tightenings, after correctness, each to be measured

| tightening | where | validity guard | cost |
|---|---|---|---|
| drop o's set-mates in the tight big-M: mate mask in `reach_scratch`, keep the λ=0 shortcut (:120) only for options without mates, skip mates in the toll loop (:122-129) and in the knapsack items (:186) | compute_tight_big_M_maps.hpp | holds under at-most-one and exactly-one; needs the conflict rows in every consuming model (flow_mip :154, flow_benders :139, multicut :231); multiflow hard-codes `knapsack_bnb` (:390), so it never sees it | about 15-20 lines, **no extra Dijkstra**; dropping mates from P̄ in `through` costs one reverse Dijkstra per conflicted option (≤ 7/6) |
| multiple-choice knapsack LP (upper hull per set, greedy on slopes; Sinha & Zoltners 1979) for `knapsack_dantzig`; min(bnb, MCKP LP) for `knapsack_bnb` | big-M maps, `lp_knapsack` users | disjoint classes, or a partition of overlapping sets into parts inside sets | about 60 lines |
| per-class max of gains in the plain big-M and in multiflow's q_s^max | compute_big_M_map.hpp:34-38, multiflow_benders.hpp:177-179 | **aggregate per option first**: after the SCC merge and the strong-arc contraction, one vertex holds several entries of one option. [toy] Entries (3,o1), (4,o1), (5,o2) with {o1,o2}: the max over entries is 5, while buying o1 alone gives 7. Options in no set are summed: dropping them fell below the optimum in 1,139 of 4,000 [toy]. With overlapping sets, sum the max of every set plus the free options (0 of 4,000 violations) | about 30 lines |
| Q_ij = 0 for set-mates | qkp_planes | exact (x_i x_j = 0) | about 10 lines |
| Q_ij = 0 for pairs that do not fit | qkp_planes | only with a tolerance ≥ every producer's: an exact float test lets a static solution at B + 5e-8 score above the bound [toy] | small, risky |
| surrogate μ_r = 1/β_r | knapsacks, `lp_knapsack` | **μ_r = 0 where β_r = 0**: otherwise the capacity is inf·0 = NaN, `lp_knapsack` returns 0 (invalid: 0 against 9 [toy]) and `knapsack_bnb` breaks its precondition; assert a finite capacity in `lp_knapsack` | small |
| MCKP hull fill or ranking in static | static.hpp:173 | an increment is eligible only while its predecessor is the set's current member; otherwise the `continue` idiom books the wrong Δc (S = {a(1,10), b(5,14), d(6,14.9)}, B=3: books 2, buys d at 6 [toy]) | about 40 lines |
| clique rows for overlapping pairwise conflicts | masters | none (a valid inequality); solvers already separate conflict-graph cliques, so the gain per backend is unmeasured | a clique routine |
| conflict-free subsets at s and t in path_lp search; per-set argmax for quality options | path_formulation_lp.hpp | none | about 15 lines each |
| set duals in split_lagrange's subgradient; μ_r for vector budgets | split_lagrange.hpp | ν ≥ 0 | about 30 lines |
| split_lagrange R1 refuses alternatives on one arc (the typical conflict) | split_lagrange.hpp:88-101 | subdividing the arc (u→w_k→v, dummy w_k of quality 0) passes R1-R3; a per-(vertex, last option) dominance is needed only for options spanning shared endpoints | instance-side |

For the multi-row constrained labels, conflict multipliers in the labelling pay little.
- They add labels only where one competing red path would use two mates in series.
- They add nothing for alternatives on the same arc, for parallel corridors, or for vertex-gain alternatives.
- A large fixed ν turns a pass into a no-op and uses up the shared 2^62 offset.

Put the effort into the tight big-M mates and the MCKP LP first.

## 7. Exactly one, disjoint only

**"Exactly one"** makes the rows equalities, so the feasible set is no longer down-closed.

Still valid: every superset argument, which is every bound, big-M, label and cut, the seeds at ∅, the in-out center, and bound ≥ base_score.

What breaks:
- **The empty selection is infeasible.** This hits "no option left" (optimize.cpp:224-228), base_score as a feasible reference, and flow_mip's `--lp-relaxation` rounding, which can leave a set empty.
- **Feasibility itself can be hard.** Disjoint sets with one budget are feasible iff Σ_Ω min c ≤ B, an up-front check. Overlapping sets (exact cover), or disjoint sets with d ≥ 2 (partition), make even finding a feasible point NP-complete. A heuristic may then return none, and `solve_result` has no "no selection" state.
- **Reformulation.**
  - A singleton set forces its option, and that option's cost leaves the budget.
  - An over-budget drop can empty a set.
  - Forcing and dropping iterate to a fixpoint.
  - `restrict_instance` and `_mip_start_solution`'s skip of removed names (mip_solver_base.hpp:367) can empty a set.
- **path_lp's option bounds become invalid** unless the free duals ρ of the equalities enter them with their sign [toy].
- **Fills and heuristics.**
  - The fills need a completion step.
  - The decremental phase must never remove a set's last member, and can get stuck: it then needs swaps.
  - The incremental phase must cover every set, possibly with zero-gain options.
- **The masters can be infeasible.**
  - flow_mip and the Benders throw "infeasible model" (mip_solver_base.hpp:305-308).
  - path_lp breaks with "master not solved" and returns a first-fit fill.
  - tree_lp has no status check after its solve (:356). On Gurobi the dual query then throws; other backends were not read.

Native SOS1 cannot express "exactly one" at all.

**Disjoint only** removes nothing from the correctness work. It simplifies three things:
- storage: one group per option, and a `group` column could serve as the format;
- the multiple-choice knapsack, which then needs no partition step;
- clique merging, which becomes moot.

## 8. Decisions for the owner

**Decided by the owner, 2026-11-13.** The first implementation goes on branch `conflicts-and-cost-dimensions`.

| Decision | Ruling |
|---|---|
| Items 1 and 5 | at most one option per set; sets may overlap; a top-level `conflicts` table of (set_id, option_id) rows; `cost` as an object of named dimensions |
| Item 2 | cost-0 members of a live set stay variables |
| Item 6 | the budget is one token: a bare number for one dimension, `name=value,…` for several |
| Scope | flow_mip, multicut_benders and flow_benders first; every other algorithm and bound refuses such instances for now |

The melon conflict graph the owner suggested is deferred to the heuristics, which need neighbour queries. The MIP rows need the declared cliques.

The other items below remain open.

1. **Conflict semantics.** At most one, recommended; exactly one; or a flag per set. Overlapping sets or disjoint only.
2. **Free options inside a set.** Keep them as cost-0 variables (recommended), which then needs the cost-0 rules of §3.2 and §5.3. Or forbid cost 0 for set members at parse time. "Buy and delete the set-mates" loses the optimum (§3.1).
3. **base_score.** Whether it keeps its meaning ("nothing paid bought") once free set members are variables, and what the guaranteed share is relative to (§3.1).
4. **Vacuous budget rows.** Whether they are dropped (this changes d = 1 behaviour when B ≥ total cost) or only vacuous conflict rows are.
5. **Instance format.**
   - The `conflicts` table, or a `group` column.
   - Cost dimensions as an object in `cost` / `csv_columns.cost`, or a long table.
   - Whether to refuse unknown keys in inline option items.
6. **Command line.**
   - The budget syntax for d > 1.
   - Rejecting `10,5`, `1,000` and `10_5` for d = 1, which are accepted today as 105, 1000 and 105.
   - A missing dimension: unconstrained, or an error.
   - A scalar budget on a d > 1 instance: an error, or the same value for every dimension.
7. **Outputs.**
   - Scalar keys kept at d = 1.
   - For d > 1, an object or an array.
   - A `feasible` flag, or a list of the violations, in the JSON.
8. **A start that violates a row.** Throw everywhere, warn and ignore everywhere, or keep each site's policy and swap only the predicate (recommended, §4).
9. **tree_lp under conflicts.** Its cast can return a selection that violates both sets of a pair even with every row in the master (§5.2). Either add path_lp's per-row fill, which reverses the no-rounding ruling, or keep the cast and report the violation.
10. **Tolerance per dimension.**
    - Acceptance: an absolute `-t` per dimension, or relative to β_r.
    - Audit: 1e-6·max(1, β_r).
11. **The heuristics.**
    - The ratio for cost vectors: the scaled Σ_r c_{o,r}/β_r with a reference dimension (recommended), max_r, Senju–Toyoda, LP-dual weights, or weights as a parameter.
    - The ranking of a cost-0 set member (§5.3).
    - Upgrade moves for greedy. Without them, greedy reproduces the refuted "buy the free member" row of §3.1.
12. **Bounds.** Whether they may call an LP solver for the d-row or overlapping-set LP, or stay solver-free (surrogate or Lagrangian multipliers in the existing subgradient loops).
13. **Tightenings in scope.** Which rows of §6, and whether melon gets an MCKP routine (no runs across classes; never the gap or timeout modes for big-Ms).
14. **Notation in the docs.** r ∈ R, c_{o,r}, β_r and Ω_l, as in this note, or other symbols.

## 9. Order of work, if pursued

1. **Data.**
   - `Instance`, the concept and the parser with schema, validation and format.
   - `budget_t`, with the scalar `option_cost` deleted, and the about 20 signatures the compiler then lists.
   - One remap helper for both instance builders.
   - The free rule and the static set, read by `optimize`'s print.
2. **Feasibility.**
   - `violations` and `fits`.
   - The output gate with its JSON flag.
   - The three start guards.
   - The guard on optimize.cpp:257.
   - The test oracle, and generators that draw conflict sets, cost 0 and d = 2.
3. **Rows** in the four MIP masters, path_lp's master, and tree_lp's master and pricing.
4. **Selections and heuristics.**
   - path_lp's enumerate filter and fill.
   - The tree_lp decision.
   - Greedy and static: incremental, decremental phases, cost 0, ratio, score update.
5. **Bounds and preprocessing hygiene.**
   - The shared "fits alone in every dimension" predicate (split_lagrange :91 and :608, qkp_planes :45 and :96, the tight big-M :78 and :123, mip_solver_base :169).
   - The μ-grid zero-share skip.
6. **Tightenings** of §6, each measured on instances with real conflicts.
   - `species-*` crossings with rope bridges are multi-case alternatives at one site, where the row changes the optimum.
   - `options_details.csv` gives each option's cells, from which an area dimension can be derived beside the k€ cost (benchmark/README.md:225-232).
7. **Docs.** These pages become false once free set members are variables, or once fills and heuristics respect rows:
   - the "Free and unaffordable options" paragraph of `docs/usage/instance.md`;
   - `docs/usage/bound.md:43`;
   - `docs/usage/results.md:38` (base score) and `:58` (tree_lp over budget);
   - `docs/formulations/preprocessing.md:14`;
   - `docs/formulations/relaxations.md:82-84`.

   Add a paragraph on when a conflict row binds (§1), the new instance keys, and the budget syntax.
8. **Parity at d = 1 without conflicts.**
   - cli_parity, the solvers' expected sets, and a non-unit-cost check of greedy and static, since the Aude costs are all 1.
   - Repeat runs where path_lp and tree_lp vary from run to run.

## 10. Literature

- **King, S., O'Hanley, J.R., Newbold, L.R., Kemp, P.S., Diebel, M.W. (2017).** A toolkit for optimizing fish passage barrier mitigation actions. *J. Applied Ecology* 54(2):599-611, doi:10.1111/1365-2664.12706.
  - Each barrier has a set of mutually exclusive projects (removal, fish pass…), written as the linear row Σ_{i∈S_j} x_ji ≤ 1, not as SOS1, under one budget.
  - Their River Wey case study used a single project per barrier, so no solver behaviour with alternatives is reported anywhere I found.
  - Read: author manuscript.
- **Neeson et al. (2015).** *PNAS* 112(19):6236-6241, doi:10.1073/pnas.1423812112.
  - Basin-wide planning was 9× more efficient than per-watershed planning, and a lump sum up to 10× more efficient than annual allocations.
  - Per-region and per-year budget rows can cost a lot, so it is worth reporting the "price of the split" by solving with and without them.
  - Read: abstract only.
- **prioritizr** (source read).
  - Zones are exclusive through the linear rows Σ_z x_jz ≤ 1, or = 1 under mandatory allocation.
  - Extra budgets are generic linear constraints.
- **Marxan with Zones** (manual read). Several costs are folded into one weighted cost under a soft threshold: the scalarization to avoid for hard budgets.
- **Gurobi reference.** It is "often more efficient to capture SOS structure using linear constraints", and presolve often reformulates SOS that way.
- **Atamtürk, Nemhauser & Savelsbergh (2000).** *EJOR* 121(1):40-55, doi:10.1016/S0377-2217(99)00015-6. Conflict graphs and clique cuts in MIP solvers. Read: search summaries only.
- **Multiple-choice knapsack.**
  - Sinha & Zoltners (1979), *Oper. Res.* 27(3):503-515, doi:10.1287/opre.27.3.503: the LP is solved by a greedy on the slopes of each class's upper hull.
  - Dyer (1984) and Zemel (1984) solve the LP in O(n).
  - Kellerer, Pferschy & Pisinger (2004), *Knapsack Problems*, chapters 9 and 11.
  - Read: secondary sources only.
- **Puchinger, Raidl & Pferschy (2010).** *INFORMS J. Computing* 22(2):250-265. MKP efficiency measures: simple; scaled p/Σ_i w_i/c_i; Senju–Toyoda; LP-dual relevance. Dual-based efficiencies gave the smallest cores. Read: technical report.
- **Senju & Toyoda (1968)**, doi:10.1287/mnsc.15.4.B196. **Loulou & Michaelides (1979)**, doi:10.1287/opre.27.6.1101. **Pirkul (1987); Chu & Beasley (1998); Fréville (2004)**, doi:10.1016/S0377-2217(03)00274-1. Greedy rules and surrogate bounds for the MKP. Read: bibliographic data only.
- **Knapsack with conflicts.**
  - Pferschy & Schauer (2009), doi:10.7155/jgaa.00186.
  - Bettinelli, Cacchiani & Malaguti (2017), doi:10.1287/ijoc.2016.0742.
  - Coniglio, Furini & San Segundo (2021), doi:10.1016/j.ejor.2020.07.023: bounds from a multiple-choice knapsack relaxation over a clique partition, the same idea as §6 for overlapping sets.
  - Read: search summaries only.
- **Kulik, Shachnai & Tamir (2013).** *Math. Oper. Res.* 38(4):729-739, doi:10.1287/moor.2013.0592. Submodular maximization under d knapsacks.
  - The guarantees come from relaxation plus rounding, not from a ratio greedy.
  - The analogy to path_lp plus a fill is loose, since PC_num is not submodular.
  - Read: preprint.
- **Sarpatwar, Schieber & Shachnai (2019).** *ORL* 47(1):1-6, doi:10.1016/j.orl.2018.11.002. One knapsack plus a matroid: disjoint at-most-one sets form a partition matroid. Ratio greedy with local search. Read: abstract.
- **Wu, Sheldon & Zilberstein (2015), IJCAI.** Connectivity objectives are not submodular, and greedy can be arbitrarily bad. Read: full text. **Xue, Fern & Sheldon (2012), AAAI.** Schedule a single-budget design under per-period budgets afterwards, rather than adding per-year rows. Read: full text.
