# path_rr: design note

Design note on gecot's `path_rr` solver, the path formulation relaxation solved by column generation. It is written for the owner and future maintainers, not as user documentation. Dated 2026-10-29, it lists every method that proved useful for `path_rr`, landed or not, with its mathematics made explicit, and the methods that were measured and set aside, so that "useful" reads as a selection. The code on branch `path-rr-rewrite`, on top of `path-rr-search-fixes`, was rewritten to follow the structure of §1 and §2; §2.8 maps the two. Its results are bit-identical to `path-rr-search-fixes` on 34 single-threaded runs covering both pricings, `--mip-start`, `--max-activations`, `--evict-age` and `--relative-gap`. On branch `path-rr-simplify` (2026-11-02), both pricings and the setup run one label search on the case graph, backwards from the target (§2.1, §2.4, §2.5): the split graph and the backtrack are gone, search prices every option shape, and `--mip-start` keeps the column without options (§2.2).

- **Sources.** Main at 029471d; the two fixes on branch `path-rr-search-fixes`; the unmerged branch `path-rr-smoothing`; and a study of 2026-10-28/29 that prototyped six directions toward a quality guarantee (a feasible incumbent plus a valid bound). An adversarial verifier reviewed five directions, all "sound with fixes". The tighter-relaxation write-up has no separate review.
- **Citations.** A commit sha marks a measurement taken when that commit was validated. "study: <direction>" marks a study result, with ", verifier" when the verifier measured or corrected it. "study: decision page" marks the study's synthesis. Where an analyst and the verifier disagree, the verifier's figure is given.
- **Instances.** bike (3,723 vertices, 1,642 single-arc options, 58,966 pairs), green (an H3 grid with star options, 268,575 pairs), Quebec (260 options), Aude, Aix 30x30, 40x40 and 70x70, Aix N1 and N2, two bike crops (`bike_small`, 352 pairs, 348 in the Lagrangian prototype's count; `bike_mid`, 2,167 pairs) and the small test instances `worst`, `test`, `min3`, `ce1`, `cx1`.

## Summary

- **The relaxation is tight where column generation converges.** Converged, the path LP is at most 0.042% above the optimum on bike_small, at most 0.030% on bike_mid, 0.01–0.03% on Quebec with enumerate (the default search LP: +0.01% / +0.09% at B=1000 / 10000) and 0.03–0.17% on Aix. Aude's 0.5–2.3% is closed at B=2 and B=3 by column-cover cuts. Full bike's converged LP has never been measured.
- **The guarantee is a feasible incumbent plus the Lagrangian bound** L = z_RMP + Σ_p max(0, max_S rc_pS), valid after every complete pricing round. path_rr computes L (1af6651) and, since branch `path-rr-feasible` (2026-11-03), returns a budget-feasible rounding of x with L as its `best_bound`, so `gecot optimize` prints a gap (§3.1). Before, it returned the master's x cast to a set, which exceeded the budget.
- **In the code** (main 029471d): pairs no option can change folded into a constant; lazy rows; start columns from `--mip-start`; enumerate pricing (budget-aware, any option shape, exponential); search pricing (default; budget-free; single-arc and star options); the max-value skip, the early stop and the unchanged-duals reuse; the dual clamp; `--max-activations`, `--evict-age`, `--relative-gap`. Two validity fixes, for an invalid bound and a crash, wait on `path-rr-search-fixes`.
- **Useful, not landed** (§3, in the decision page's three tiers). *Small and generic, land first:* time limits and an LP status check; an incremental evaluator. *Land after review:* local search; pruned search pricing; lazy budget pricing, a knapsack-hull end phase and column-cover cuts for instances that converge; generic exact pricing for every option shape (it certifies Aix N2 within 0.38%, where nothing else runs). *Research, not ready to land:* the direct Lagrangian dual; a barrier master with value scaling, and dual completion (modest on bike_small, 449 → 342 rounds; unproven alone on green); pair tolerance.
- **What stalls, and why** (§4): bike and green. A useful bound needs purchase duals on most (pair, option) entries. Under such duals one pair search costs 20–300 ms instead of 0.4 ms, and bike has 58,966 pairs. Under `--max-activations` most pairs never get rows, price every option as free, and keep the bound near the all-free value: 7.72e6 on bike B=1000, against prep_mip's 6.459e6. Green is limited by coverage and by the master LP, about 90% of a round.

## 1. Problem and relaxation

### 1.1 The problem

- **Options** o with cost c_o and a budget B. gecot's trivial reformulation buys zero-cost options statically, so every remaining option has c_o > 0.
- **Cases** k, each a digraph G_k with arc probabilities p_a and vertex qualities q_v as a source and as a target. An option improves arcs (p'_oa) and qualities (gains g_ov ≥ 0 as a source and as a target; the parser rejects negative gains).
- **Evaluation** of an option set X (helper.hpp):

```
p_a(X)     = max(p_a, max_{o∈X improving a} p'_oa)      an arc improved by several options takes the max
q_v(X)     = q_v + Σ_{o∈X} g_ov                          quality gains add up
P_st(X)    = most reliable s → t path probability under p(X)
v_p(X)     = q_s(X) q_t(X) P_st(X),   p = (s, t), s = t included
Score_k(X) = Σ_{p∈k} v_p(X)
maximize φ(Score_1(X), …, Score_K(X))   s.t.  Σ_{o∈X} c_o ≤ B
```

- **Monotone.** p_a(X) and q_v(X) are nondecreasing in X, hence so is v_p(X). Every bound below relies on it.
- **Criterion φ:** sums, min, nonnegative scalings and constants of case scores. It is concave and nondecreasing in each case score. Products of case variables are rejected (`formula_variable_visitor`).
- **Caveat: negative factors.** The parser accepts a negative constant factor, but every criterion variable has lower bound 0 (mippp's default, in `formula_variable_visitor`), so a term c·e with c < 0 forces e = 0 and the term is dropped. Measured on Aude B=3 with sum([2, trout], [−1, trout]), which equals trout: mip returns the empty solution as "optimal" with a bound of 0 (the optimum is 1,120,213.88), and path_rr's Lagrangian bound is 0.001, below the empty landscape's 864,655.72 (main 029471d, 2026-10-29). It affects every solver built on `formula_variable_visitor`, not only path_rr. A negative factor also breaks monotonicity (§3.10) and, on a min term, the concavity §3.8 uses to compute φ*. A negative additive constant, sum(k0, −50), solves on main (119.115; study: master-acceleration, verifier).

### 1.2 The path LP

One column per (pair p = (s, t), option set S), of value v_pS = q_s(S) q_t(S) P_st(S): S is the set of options bought on the path, plus its endpoint gains.

```
max  φ(z_1, …, z_K)                                        linearised by formula_variable_visitor
s.t. z_k − Σ_{p∈k} Σ_S v_pS y_pS ≤ C_k     [π_k ≥ 0]        per case; C_k: the constant pairs (§2.1)
     Σ_S y_pS ≤ 1                          [σ_p ≥ 0]        per pair: uniqueness
     Σ_{S∋o} y_pS − x_o ≤ 0                [λ_po ≥ 0]       per (pair, option): purchase, lazy (§2.2)
     Σ_o c_o x_o ≤ B                       [μ ≥ 0]          budget
     x ≥ 0, y ≥ 0                                           no x_o ≤ 1 row

dual constraint of y_pS:  rc_pS = π_k v_pS − σ_p − Σ_{o∈S} λ_po ≤ 0
dual constraint of x_o:   Σ_p λ_po ≤ c_o μ
```

- The duals are ≥ 0 because the rows are ≤ rows of a maximisation, whatever φ. All eight LP backends return them so, up to tolerance: Gurobi returned −7.6e-7 on Quebec B=10000 (8429409).
- x has no upper bound. That is harmless for the LP value, since c_o > 0 and Σ_{S∋o} y_pS ≤ 1, but it shapes the duals: x_o's dual row has no term for x_o ≤ 1 (§2.3, §4.6).
- The pairs are coupled only through x: each pair picks its option set independently.

### 1.3 Why it is a relaxation

- **z_IP ≤ z_LP(𝒞)** for any column family 𝒞 that contains every column with c(S) ≤ B. Take a budget-feasible X and set x = 1[X]. For each pair put y_{p,S_p} = 1, with S_p the options of X on p's best path plus its endpoint gains. Then v_{pS_p} = v_p(X), Σ_{S∋o} y_pS = [o ∈ S_p] ≤ x_o and z_k = Score_k(X). For z_budget this holds under exact cost arithmetic only (§2.4).
- **The hierarchy.**

```
z_IP ≤ z_PPC ≤ z_hull ≤ z_budget ≤ z_search
z_hull ≤ z_hull^S ≤ z_search                     z_hull^S and z_budget are not comparable (§3.8)
z_IP ≤ z_tree ≤ z_budget

z_search  columns of any cost: search pricing (§2.5)
z_budget  columns with c(S) ≤ B: enumerate pricing (§2.4)
z_hull    z_budget with x ∈ conv{x ∈ {0,1}^O : c·x ≤ B}: the Lagrangian dual of the purchase rows
          with budget columns (Geoffrion; §3.6b, §3.8)
z_hull^S  z_search with x in the same hull: the Lagrangian dual with search columns (§3.8)
z_PPC     per-pair consistency: each pair a distribution ν_p over budget-feasible full sets X with Σ_{X∋o} ν_pX = x_o;
          the best any family of single-pair inequalities can reach (§3.6c)
z_tree    tree_rr: one column per (target, option set) shared by all sources of the target
```

- With unit costs and an integer B the knapsack polytope is integral, so z_hull = z_budget and z_hull^S = z_search (Aude, Aix70 B=2).
- The first chain is the analyst's (study: tighter-relaxation, no review). It follows from standard arguments.
- **Against tree_rr.** tree_rr has per-(target, option) purchase rows and the budget row inside its pricing MILP. Splitting a tree column (t, X) into its pair columns (s, t, X_s ⊆ X) maps any tree-LP point into the budget path LP, hence z_tree ≤ z_budget. This is a derivation: tree_rr's column definition was not re-read in full. It concerns exact converged values. tree_rr's own relaxation_value is not a bound, since its pricing is inexact (2089fd9). Observed: optimum ≤ tree ≤ path (0b3f2b2). Aude B=3: tree_rr's DW bound with the knapsack pricing row was +0.30% above the optimum, against path enumerate's +0.85% and search's +1.64% (2026-10-27 ablation, before 2089fd9).
- **What path_rr gains over tree_rr** is convergence. Quebec B=1000: path_rr converges in 6.7 s (0b3f2b2), while tree_rr has not converged at 600 s, its master 0.7% below the optimum (c983ada). Quebec B=10000: 65 s at 3.3 GB (0b3f2b2), while tree_rr's master is 45% below the optimum at 600 s, under the empty landscape's score (c983ada).

### 1.4 Measured tightness

| Instance | LP | Above the optimum | Source |
|---|---|---|---|
| bike_small, 7 budgets | search, converged | +0.0004–0.042% (0.01–7.6% of the gain) | study: tighter-relaxation |
| bike_mid B=200 / 900 / 1800 | search, Lagrangian upper bound | ≤ +0.030% / +0.023% / +0.030% | study: tighter-relaxation |
| Quebec B=1000 / 10000 | enumerate | +0.010% / +0.027% | 0b3f2b2 |
| Quebec B=1000 / 10000 | search | equal to enumerate / +0.064% above enumerate | c7754f5 |
| Aix30 B=4, B=8; Aix40 B=2, B=4; Aix70 B=2 | enumerate | +0.025, 0.133, 0.034, 0.109, 0.173% | study: tighter-relaxation |
| Aude B=1 / 2 / 3 / 5 / 8 | enumerate | 0 / +0.4918 / +0.8535 / +2.2582 / +0.9901% | study: tighter-relaxation; B=8 from the f0da701 comparison runs |
| Aude B=2 / 3 / 5 / 8 | search | +1.36 / +1.64 / +2.26 / +0.99%; equal to enumerate at B=5 and 8 | study: reduction, tighter-relaxation, lagrangian-dual |
| worst B=3 | search / enumerate | 11.13 / 8.06, optimum 7.46 | study: decision page |
| full bike B=1000 | search | never converged; 6.42–6.43e6 extrapolated | study: tighter-relaxation |

- The Aix gaps are corroborated by the verified generic-pricing results (0.034% on Aix40 B=2, 0.17% on Aix70 B=2).
- **Aude's gap is not structural.** Column-cover cuts close it at B=2 and B=3 (§3.6c). It is structural only for multiplier methods, since unit costs give z_hull = z_budget. This corrects the Lagrangian analyst's claim.
- **Full bike is an extrapolation.** Its round-1 columns carry about twice as many options as bike_mid's (median 32 against 14), so its gap could be larger. The extrapolated 6.42–6.43e6 would certify the 6.419e6 incumbent within about 0.05–0.2% (study: tighter-relaxation). §3.8 and §4 rest on this unmeasured number.

## 2. Column generation, as implemented

Main at 029471d, plus `path-rr-search-fixes` where noted; §2.8 maps each part to the rewritten code. One round:

```
solve the master; read π, σ, λ
price every pair in parallel (tbb): enumerate scan, or skip / reuse / search
if no pair was cut short: L = z_RMP + Σ_p r̄_p; keep the lowest; test --relative-gap
pool ← priced columns; re-price pool and master columns at the new duals; purge pool columns with rc ≤ 0
activate the columns with rc > margin(p), at most --max-activations of them
stop if a complete round activated nothing; on --time-limit stop, re-solving the master if it changed
evict unattractive master columns (--evict-age)
return the options by decreasing x that fit in the budget, or the start if it scores more (§3.1)
```

Multithreaded runs vary in iteration count from run to run, with the same final values: pricing results arrive in thread order and are not sorted, by ruling. Parity checks use `--max-threads 1`.

### 2.1 Pairs and constants

- **Endpoints, per case.** Sources: vertices with q_v > 0 as a source, or with an option giving them a source gain. Targets likewise. Pairs: sources × targets, s = t included (0b3f2b2).
- **base_p = v_{p∅} and V_p**, the largest column value of the pricing's family. Search mode: V_p = v_p(O), every option bought and the budget ignored; it is the maximum by monotonicity. Enumerate mode: the maximum over the enumerated budget-feasible columns.
- **Folding rule** (c7754f5, both modes). If V_p ≤ base_p, every column of p is worth base_p, so p is dropped and base_p added to C_k. It is exact, for any criterion: nothing is relaxed.
- **Setup** is parallel over targets (`path-rr-simplify`; over sources before). Search mode runs three label searches per target with every option free (weight 0, labels compared on value alone: a most-reliable-path Dijkstra backwards from t), with no option allowed (base_p), with all (V_p and its column) and with the start's options. Per-target constants and pair lists are concatenated in target order, so the model is the same from run to run.
- **Counts.**
  - Quebec B=1000: 146,987 pairs in search mode against 131,525 in enumerate mode, whose budget-limited V_p folds more. B=10000: 181,521 against 181,441. Aude B=3: 1,062 against 474 (c7754f5 and 8ef534c runs).
  - Aix 30x30: 2,209 pairs (47 sources, 47²), 2,101 kept at B=4 in enumerate mode.
  - bike: 58,966 of the 392 × 162 source-target pairs; green: 268,575.
- **Caveat.** On main, search's V_p inherits the center-copy bug of §2.5: on cx1 the only pair was folded at V = 0.3 while the optimum is 0.54. 560ce44 fixes it.

### 2.2 Master: lazy rows, start columns

- **Lazy rows** (8ef534c). A pair's σ_p row is created with its first master column, and its λ_po row with its first master column containing o.
  - Before that the row holds no column (0 ≤ 1, or −x_o ≤ 0) and is redundant. Dual 0 extends the RMP's optimal dual and leaves x_o's dual row unchanged, so pricing a missing row at 0 is exact column generation, and L stays valid.
  - Quebec B=1000, enumerate: 460k → 134k rows, 6.7 → 3.9 s, 405 → 207 MB. B=10000: 2.59M → 351k rows, 64.7 → 32.7 s, 3.3 → 2.2 GB. The 2.2 GB left are the 20.5M stored columns (8ef534c).
  - The consequence (§4.3): a pair without rows sees every option as free.
- **Start columns** (0b3f2b2 for enumerate, c7754f5 for search). Each kept pair starts with its base column, when base_p > 0, and S⁰_p = argmax{v_pS : S ⊆ X_0}.
  - X_0 is the `--mip-start` set if c(X_0) ≤ B + feasibility_tol; an over-budget start is ignored with a warning. Without a start, X_0 = ∅ and S⁰_p is the base column.
  - Before `path-rr-simplify`, S⁰_p replaced the base column, so `--mip-start` lost the base columns: 1.27× slower than no start over 13 search-priced pairs (Quebec B=1000: 65 rounds against 14). With both: 0.92× over the search-priced pairs, 0.75× over 8 enumerate-priced ones, Quebec B=1000 in 11 rounds (2026-11-01 ablation).
  - With x = X_0 these columns are feasible, and the RMP holds nothing better, so the first master value is score(X_0).
  - Search mode computes S⁰_p with the setup's label search restricted to X_0. Enumerate takes the best pooled column ⊆ X_0; that is exact, because a column dominating a subset of X_0 is itself a subset of X_0.
  - Checked: with prep_mip's optimum as the start, the first master equals it to about 1e-15 on 8 Aude and Aix instance-budget pairs (0b3f2b2). Search-mode start masters are exact (c7754f5).
  - The start sets the master's first value, not the bound. Greedy leaves budget slack, so μ = 0 and every λ_po = 0 (§4.6). path_rr returns the start only if it scores more than the rounding of x (§3.1).

### 2.3 Reduced cost and the Lagrangian bound

```
rc_pS = π_k v_pS − σ_p − Σ_{o∈S} λ_po                λ_po = 0 for a missing row
L     = z_RMP + Σ_p r̄_p,     r̄_p ≥ max(0, max_{S∈𝒞_p} rc_pS)
```

- **Why it is valid.** (π, σ + r̄, λ, μ) is feasible for the dual of the master over all of 𝒞: every y column now has rc_pS − r̄_p ≤ 0, and the x, z and criterion columns are all in the RMP. Its dual objective is Σ_k π_k C_k + Σ_p (σ_p + r̄_p) + B μ, plus the criterion's constants. The purchase rows have right-hand side 0, and the RMP's optimal dual objective is z_RMP, so this equals z_RMP + Σ_p r̄_p. Hence L ≥ z_LP(𝒞) ≥ z_IP. In search mode 𝒞 is every column and L bounds z_search; in enumerate mode it bounds z_budget.
- **In the code** (1af6651):
  - enumerate: r̄_p = max(0, best rc over the pair's stored columns);
  - search: r̄_p = margin(p) for skipped, stopped and unproductive reused pairs, and the best rc when a column above margin(p) is found;
  - margin(p) = 1e-7 σ_p + 1e-6, the same test as the activation;
  - after a round where no pair was cut short, L is logged, and the lowest is reported as `best_bound` (`lagrangian_bound` before `path-rr-feasible`).
- **Conditions.**
  - Missing rows have dual 0 (§2.2).
  - **λ ≥ 0, the dual clamp** (8429409). Search charges λ̃_po = max(0, λ_po). melon's biobjective_dijkstra needs nonnegative lengths; with a negative dual, labels improve around cycles and the backtrack loses their predecessors (intermittent; shifting the duals by −1e-6 reproduced it every time). z_RMP uses the raw duals, so the clamped point can violate x_o's dual row, and L can understate the rigorous bound by Σ_o max(0, Σ_p λ̃_po − c_o μ). That is about 1e-6 per row (study: reduction, verifier); the tolerance-free L of §3.9 adds the term. Enumerate scans with raw duals, and its dominance proof (§2.4) also assumes λ ≥ 0.
  - Each pair's term bounds its best reduced cost: exact pricing, or margin(p) where a valid test showed nothing above it. Search is exact only for the shapes it accepts, and only with 560ce44 (§2.5).
  - The RMP was solved to optimality. The status is not checked (§3.2).
  - The round is complete: a round cut short by the time limit gives no bound.
  - π_k ≥ 0: the sign of a ≤ row's dual in a max LP, for any criterion (§1.1).
- **Checked.**
  - Bike's round-1 L is 7,816,736.299526224, equal to the Lagrangian prototype's L(0) (study: lagrangian-dual). An independent recomputation equals `lagrangian_bound` to about 1e-10 relative on every dump (study: reduction).
  - No printed bound below the converged LP* in 19 bike_small and 5 Quebec runs; none below the brute-force optimum on 64 tiny converged instance-budgets or 40 random search-shaped instances (study: master-acceleration, lagrangian-dual, verifiers).
  - 0 bound violations in 9,504 fuzz runs with `--max-activations`, `--evict-age` and `--relative-gap` (study: generic-pricing, verifier).
  - On Quebec the bound is within 1e-4 of its converged value after 7 rounds (search, B=1000 and B=10000), against 15 master solves to converge (mippp dual_smoothing §7).
- **`report.best_bound`** since `path-rr-feasible`: `gecot optimize` prints `best_bound` and the gap against the returned solution, which now fits the budget (§3.1). Against the former over-budget cast the gap came out at −0.08% on Quebec (1af6651).
- **`--relative-gap g`** stops once L_best − z_RMP ≤ g·z_RMP, with status "relative gap reached". z_RMP is not a lower bound on z_IP, so this certifies LP convergence within g, not an optimality gap.

### 2.4 Pricing by enumeration

- **Path labels** (`_labels`; `_path_labels` per source until `path-rr-simplify`). Per target, a label-setting search backwards on G_k with labels (P, S, c(S)), P including q_t(A) for the subset A of t's quality options it starts with, popped by decreasing P then increasing c(S), and checked against the labels that already left the vertex (§2.5).
  - Along a = (u, v), the base arc gives P·p_a with S unchanged. Each option o improving a gives P·p'_oa with S ∪ {o}, if c(S) + c_o ≤ B; it is free when o ∈ S.
  - Dominance: ℓ1 ≽ ℓ2 iff P1 ≥ P2 and S1 ⊆ S2. Extension preserves it (same factor, same added option, c(S1 ∪ {o}) ≤ c(S2 ∪ {o})).
  - Max semantics come through dominance, since the base arc and the improved arcs are all relaxed.
- **Pair columns** (`_pair_columns`). Each label reaching s is combined with each subset A of the options with a source gain at s (at most 20 there and at t, else it throws). S' = S ∪ A must fit the budget; its value is (q_s + Σ_{o∈S'} g_os)(q_t + Σ_{o∈S'} g_ot)·P. A column is kept if positive and non-dominated under (v ≥, S ⊆).
- **Dual independence.**

```
S1 ⊆ S2, v_pS1 ≥ v_pS2:   rc_pS1 − rc_pS2 = π_k (v_pS1 − v_pS2) + Σ_{o∈S2∖S1} λ_po ≥ 0    when π_k, λ ≥ 0
```

- So the pool computed once holds an argmax for every dual vector, and one scan per round is exact over {c(S) ≤ B}. The LP is z_budget.
- **Floating point.** `_labels` (enumerate's keep test) and `_pair_columns` drop a set on a strict `cost > budget`. Since `path-rr-feasible`, `_pairs` receives B + feasibility_tol, the feasibility of the start check and of the returned solution (§3.1), so every set gecot accepts is a column. Before, a set of cost exactly B whose float sum rounds above it (0.1 + 0.2 + 0.3 > 0.6), or with B < c ≤ B + tol, was lost, and z_IP ≤ z_budget held under exact cost arithmetic only.
- **Generic** for every option shape: multi-arc options, several options per arc, quality options at both ends. **Exponential** in the number of affordable options per path.
- **Measured.**
  - Fronts: Aude about 1,000 columns; Aix 30x30 33k labels (76k with endpoint options) in 0.05 s; Aix 40x40 under 7k; Quebec B=1000 391k labels (729k); Quebec B=10000 11.7M labels (28M), enumerated in 5 s (0b3f2b2).
  - Times at landing: Aude 0.01–0.06 s; Aix 30x30 0.35–0.77 s; Quebec B=1000 6.7 s; Quebec B=10000 65 s at 3.3 GB (0b3f2b2); 3.9 s and 32.7 s with lazy rows (8ef534c).
  - Against prep_mip (f0da701). prep_mip is faster on Quebec (2.2 against 3.8 s at B=1000, 8.4 against 33 s at B=10000) and on Aude (all under 0.1 s). path_rr is faster on Aix: 1.7–3.3× on 30x30, 2.5× on 40x40, and 20 s / 0.6 GB against 47 s / 2.2 GB on 70x70 B=2.
  - It explodes when many cheap options lie along a path: green even at B=0.5; bike never finishes, even at B=50, with up to 1.6e15 non-dominated labels per pair at B=1000 (8429409); Aix N2 B=100 stays in setup over 200 s (study: generic-pricing).
- **Role since 029471d.** Opt-in `--pricing enumerate`, for small instances whose options search cannot price: Aix, test, min3. N1 and N2 are out of reach of both pricings (enumerate stays in setup on N2; N1 was never run). Where both run, it gives the tighter z_budget: Aude B=3 1,129,774.9224 against 1,138,555.6616 for search.
- **Fix needed.** Check the clock inside the enumeration parallel_for (§3.2).

### 2.5 Pricing by search

The pricing problem of pair p = (s, t) in case k:

```
r_p = max over option sets S and paths P of   π_k q_s(S) q_t(S) Π_{a∈P} p_a(S) − Σ_{o∈S} λ̃_po − σ_p
```

- **Labels** (`_labels`, branch `path-rr-simplify`). One label-setting search per pair and round, backwards from t on G_k itself. A label (w, W, T) is a path u → t: w = q_t(A) Π p_a, with A the subset of t's quality options it starts with (one start label per subset, at most 2^20); W = Σ λ̃_po over the options bought; T, its live options, those it bought and can still use for free. Along an arc a it takes p_a, or p'_oa > p_a for each option o of a, buying o (W += λ̃_po) unless o ∈ T. Labels hold their predecessor and the option bought, so S is read back at s; there is no backtrack.
- **Live options** (`option_reach`). At a vertex u, T keeps the options with a quality gain at s or at t, those whose arcs share no vertex, and those of which u is a vertex all the arcs share, if one improves an in-arc of u. Any other later use of o goes through a shared vertex the path crossed when it bought o, so through a cycle. A single-arc option never stays live, a star only at its center.
- **Dominance.** ℓ1 ≽ ℓ2 iff w1 ≥ w2 and W1 + Σ_{o∈T2∖T1} λ̃_po ≤ W2, the rule of §3.7: for any completion, ℓ1 pays at most λ̃(T2∖T1) more than ℓ2 for the options ℓ2 uses again. Without live options it is the (w, W) Pareto rule of the former search. Labels pop by decreasing w, then increasing W, and a label is checked only against those that already left its vertex, kept by decreasing W, as in Martins' algorithm: on two objectives the last one decides.
- **At s.** Each quality option of s that is not live, its gains at s summed (an option can carry several entries, §3.7), is bought iff π_k g_os w > λ̃_po; then rc = π_k q_s w − W − σ_p. Labels stop at s.
- **Pruning.** A label is dropped unless π_k q̄_s w − W − σ_p > margin(p), with q̄_s the source quality with every gain: w only decreases and W only grows, so r̄_p = max(margin(p), best rc). It extends the former early stop, which had W = 0; the potentials of §3.5 would tighten q̄_s w further.
- **Exactness.** For every option shape, given λ̃ ≥ 0 (the clamp, 8429409). The budget is ignored, so the LP is z_search.
  - Fuzz, 1,300 random instances with duplicate quality entries, probability-1 arcs, several options per arc and multi-arc options, both pricings: no bound below mip's optimum, no bound below the LP, no LP below the optimum, no search LP below enumerate's.
  - Two defects found on the way and fixed: per-entry pricing of an option's duplicate gains at s (7 bounds below the optimum in 300 instances), and an option with a gain at t dropped from T before the path reached its arcs (1 in 300).
- **Measured** against the former search (`path-rr-simplify`, 2026-11-02; 21 instance-budget pairs, 120 s).
  - Same LP: 94 single-threaded runs over both pricings, `--mip-start`, `--max-activations 100` and `--evict-age 3` agree to ≤ 8.6e-11, with the same runs converged.
  - Time, median of 2 multi-threaded runs: 0.94× over 13 search-priced pairs (worst bike_small B=50, 1.69 s against 1.32 s; best twomin B=4, 0.63×), 0.92× over 8 enumerate-priced ones. bike_small B=400 single-threaded: 51.8 s against 50.9 s. Quebec B=1000: 19,451 searches against 33,642.
  - Green B=5 and full bike B=1000, `--max-activations 5000`, 600 s: unchanged, the master LP still dominates (green 49 rounds, gap 10.60% against 10.58%; bike 9 rounds, 23.0%), with 2.1× and 2.7× fewer searches thanks to the pairs without a positive dual.
  - Search mode now runs on the shapes it refused: Aix30 B=4/8/12, Aix40 B=4, r1, r3 and worst converge in 0.01–2.9 s, at or above enumerate's LP (worst 11.13 against 8.06). Aix70 B=2 does not (3 rounds in 120 s, its general options stay live): enumerate converges in 25 s.
- **Not kept.** A search per target, sharing one label search among the pairs to t, with a label dominating those of no more value that buy a superset of its options of positive dual for some pair to t: exact and with 15–30× fewer searches on Quebec and Aude, but on bike_small 10,163 labels per search against 105 on Quebec, and the 120 s limit at every budget.
- **The former search** (c7754f5, 0488e2b, with 560ce44 and d47ada5) priced single-arc and star options only, on a split graph with an improved copy per star center, with melon's biobjective search and a backtrack. The measurements elsewhere in this note were taken with it.
- **The LP z_search, measured with the former search**, and the same with the new one.
  - Equal to enumerate where the budget does not bind: Aude B=15, 1,738,196.8739016054 against …6056; Quebec B=1000, 118,257,927,545.30125 against …30127 (c7754f5). A Quebec copy with 120 added single-arc options agrees to 1e-12. A 282-vertex bike crop agrees at B=50; at B=100 search is +3.3e-5 higher, from 60 over-budget columns (0488e2b).
  - Weaker but lighter at a large budget: Quebec B=10000, +0.064% above enumerate at 457 MB against 2.2 GB (c7754f5).
  - Runs where enumeration explodes: green B=5, cap 5,000, 600 s: 30 rounds of about 20 s at 0.87 GB, not converged (c7754f5).
  - One former search: 0.4 ms at zero duals, 20–300 ms under informative duals (§4.4).
  - Budget-free columns overshoot B. Measured by a Python Dijkstra over all 58,966 pairs, full bike's round-1 columns have a median of 32 options (p90 64, max 122) and a median cost of 1,815; 75.0% exceed B=1000, 50.4% B=1800, 2.6% B=5000. On green B=5, 0.7% do (study: tighter-relaxation).
- **Option shapes of the instances** (study: generic-pricing). bike: 1,642 single-arc. green: 763 stars (2 arcs and 1 vertex). Quebec: 260 stars. Aude and worst: 15 and 13 two-way segments, stars. Aix30: 12 stars and 6 general; Aix70: 42 and 23. N1: 536 general, with 5,901 arcs improved by several options; N2: 576 general, 1,801 such arcs. test: two quality options at one vertex. The former search refused Aix, N1, N2, test and min3; the new one prices them all, N1 and N2 untested.
- **Latent issues.**
  - Exact label ties can make the search return columns that buy useless options. On Aude, 33 of 1,062 round-1 columns of the former search differ from a search with a fewer-options tie-break (study: generic-pricing, analyst). The new search breaks value ties by weight only.
  - A single search cannot be interrupted: the clock is checked between searches.
### 2.6 Skipping and reuse

- **No positive purchase dual** (`path-rr-simplify`). Every rc_pS ≤ π_k V_p − σ_p, which V_p's column attains: it is the result, without a search.
- **Max-value skip** (c7754f5). If π_k V_p ≤ σ_p + margin(p), then every rc_pS ≤ π_k V_p − σ_p ≤ margin(p). The pair is not searched, and r̄_p = margin(p). This needs V_p ≥ v_pS for every S, which 560ce44 restores for star options that lower an arc.
- **Unchanged-duals reuse** (c7754f5 for unproductive searches, f0da701 for all).
  - With the graph, the qualities and V_p fixed, a pair's result depends only on κ_p = (π_k, σ_p, the duals of its existing purchase rows in option order); missing rows are 0.
  - If κ_p equals the key of its last search (exact double equality; a new row lengthens the key, so there is no false hit), the stored result is reused. Its column goes back to the pool, and r̄_p is the stored bound, or margin(p) when the search found nothing. The reused result is what a new search would return, so the round stays complete.
  - The key is built after the skip test. The reuse test comes before the time check, so a reused pair never cuts a round short.
- **Measured.**
  - Green B=5 from greedy, 600 s: searches 11.0M → 0.72M, rounds 45 → 64, bound 3.1186e13 → 3.0944e13, certifying greedy within 9.3%. The master LP becomes about 90% of a round, growing from 1 s to 10 s (f0da701).
  - Bike B=1000, cap 5,000: 94,483 searches in all, 58,966 of them in round 1 and about 4.4k per round after (study: master-acceleration).
  - tree_rr has the same skip per target (c983ada): 28–46% of pricings skipped on Aix 30x30, with the same final value.
- **Anything that moves the duals defeats it.** Wentges smoothing at α = 0.5 multiplies Quebec B=1000 searches by 13 (90k → 1.18M; mippp dual_smoothing §7, F3). Barrier duals: 90k → 209k at B=1000, 207k → 555k at B=10000. Dual completion on green: 11.4M against 568k searches (study: master-acceleration).
- On bike and green the reuse is cheap because most pairs never get rows and keep zero duals. It is a symptom of §4.3, not its cause (study: master-acceleration, verifier).

### 2.7 Column management

- **Pool.** Priced columns enter the pool of mippp's column manager. Each round, pool and master columns are re-priced at the new duals (`update_columns`), pool columns with rc ≤ 0 are purged, and columns with rc > margin(p) are activated. The activation test is the pricing test, so a pair with σ_p = 0 cannot loop on noise columns.
- **`--max-activations k`** (c7754f5, `colgen::at_most_k_best`). At most the k best improving columns enter per round. Convergence and L are unchanged: the loop stops only when no pair has a column above margin(p), and L uses each pair's r̄_p, not the activations.
  - Quebec, uncapped against cap 5,000: B=1000 15 → 19 rounds, 5.44 → 3.69 s; B=10000 15 → 34 rounds, 28.82 → 15.75 s; same values (c7754f5 runs, cap read from the file names).
  - Bike B=1000: uncapped, the second master LP did not finish in 900 s. Caps 5,000 / 1,000: 9 / 18 rounds in 870 / 786 s, master 6.276e6 / 6.263e6, below the feasible 6.419e6; bound 7.72e6 (029471d).
  - The cap limits coverage (§4.3), and k has no rule. Pool-first activation was rejected (§5).
- **`--evict-age k`** (0b3f2b2). A master column older than k rounds with rc < −1e-6 goes back to the pool.
  - A basic column has rc = 0, so only nonbasic columns leave and the RMP optimum is unchanged. Rows stay, so L is unaffected. An evicted column is re-priced at the next round and purged unless its reduced cost is then positive; later, it returns only if the pricing finds it again.
  - Eviction runs after the time-limit test, and only when the master will be solved again: removing columns after the last solve loses the solution on Gurobi and CPLEX and silently zeroes it on HiGHS (mippp column manager note, L1 and X1). path_rr calls `update_columns` once per round, so the age counts rounds (L11).
  - Backends without `has_remove_variable` warn and ignore the option.
  - Measured neutral or slower on Aude, Aix and Quebec (0b3f2b2). Never run on bike or green, where the master dominates; that was tree_rr_cm's only winning case.
  - bike_small B=200, `--evict-age 3`, single-threaded: no convergence in 1,800 s on main cf1d235. On `path-rr-feasible`, whose loop is the same, 4,946 rounds in 6 minutes, the bound stuck at 118,431 (117,226 converged without eviction, in about a minute): evicted columns come back in a cycle (2026-11-03).
- **Time limit** (0b3f2b2, c7754f5). In search mode the clock is checked before each pair search; an unsearched pair marks the round cut short, and such a round gives no bound and cannot converge. In enumerate mode it is checked only after activation, so every enumerate round is complete. After activation, if time is up, the status is "time limit reached", and the master is solved once more if columns were added, because the backends lose the solution after the model changes (as tree_rr's 33ccf0d). The master solve and the enumeration ignore the limit (§3.2).

### 2.8 Code map

In `include/gecot/solvers/path_formulation_rr.hpp` on branch `path-rr-simplify`, which uses the notation of §1.2: z_k, C_k, π_k, σ_p, λ_po, rc_pS.

| Part | Where |
|---|---|
| The LP (§1.2), the reduced cost and L (§2.3), the two pricings | the comment above `struct path_formulation_rr` |
| Pairs, C_k, base_p, V_p and start columns (§2.1, §2.2) | `pair_data`, `_pairs` |
| The label search of both pricings and the setup (§2.4, §2.5) | `label`, `_labels`; its weights, live options, pruning and dominance are the callers' |
| Enumerate pricing (§2.4) | in `_pairs`, `_labels` with costs, then `_pair_columns`; in `solve`, the `enumerate` lambda scans `pair_data::columns` |
| Search pricing (§2.5) | `option_reach`, `_option_reach`; in `solve`, the `search` lambda: skip, no-dual case, reuse, then `_labels` with the λ |
| base_p, V_p and start columns (§2.1, §2.2) | in `_pairs`, `best_columns`; `pair_data::max_column`, `pair_data::start_columns` |
| Reuse (§2.6) | `last_search`, `searches` |
| Master and lazy rows (§2.2) | in `solve`: `x`, the budget row, `z`, `contribution_rows`, `uniqueness_rows`, `purchase_rows`, `add_column` |
| Duals and reduced cost (§2.3) | in `solve`'s loop: `pi`, `sigma`, `lambda`, `reduced_cost`, `margin` |
| L and `--relative-gap` (§2.3) | `pricing_bounds` (r̄_p), `master_value` (z_RMP), `report.best_bound` (the lowest L) |
| The returned solution (§3.1) | after the loop: `options` by decreasing x, `purchased`, `score` for the start |
| Column management (§2.7) | `columns`, mippp's column manager; `max_activations`, `unattractive` |

## 3. Toward a guarantee: methods that proved useful, not landed

The study's ranking: a valid guarantee for every instance shape first, then bound strength, then implementation cost. The decision page groups the items in three tiers: 1–4 small and generic, to land first; 5–8 to land after review; 9–11 research, not ready to land. Line counts are the analysts' estimates. "Sound with fixes" means that the verifier found no flaw in the method, but that the fixes are needed before landing. Decision-page item 1, the two search fixes, is described in §2.5.

### 3.1 A budget-feasible incumbent and its gap (item 2)

- **Status.** Landed on branch `path-rr-feasible` (2026-11-03), about 25 lines, without the local search polish. The proposal had been reviewed as a fix by three verifiers (study: primal-side, master-acceleration, reduction); the change itself by three reviewers, each finding checked by a skeptic.
- **Problem.** path_rr returns {o : x*_o > 0}, and nothing ties that set to the budget: a fractional x* spreads small values over many options.
  - Simplex masters: 365–409 options at cost 16–20k on bike B=1000; costs of 1,359 / 15,687 / 42,264 on Quebec B=1000 / 10000 / 20000. Barrier: 1,504 options at 78.7k on bike. Tiny instances: 11 of 64 over budget with simplex, 36 of 64 with barrier (study: master-acceleration, reduction, verifiers).
  - On N2 B=100 the cast scores 1466.11, above the 1463.19 bound of the same run (generic exact pricing prototype; study: generic-pricing, verifier).
  - Reported by the owner on the regenerated Aix 70x70, B=10: cost 11.6. The master spends exactly 10, nine options at x = 1 (9.6) and option 123 at x = 0.2 (cost 2); the cast buys 123. Its score, 2,688,238, is above the run's own L, 2,650,855.
- **Change.**

```
X      = G(x*), or X_start if score(X_start) > score(G(x*))
G(x*)  : the options by decreasing x*_o, ties in option order; each is bought if it fits in B + feasibility_tol
report.best_bound = L_best,  only after a complete pricing round; gecot optimize prints gap = (L_best − z)/z
```

  - G skips an option that does not fit and goes on, with static_incremental's test. The options of x*_o = 0 come last and take what is left: scores are monotone in the option set (gains and qualities ≥ 0, improved arcs by max), so they never lower the score. The negative-factor caveat of §1.1 is the exception.
  - Enumerate's columns are built within B + feasibility_tol, the test of G and of the start check (§2.4). With the strict B, a set of decimal costs whose float sum exceeds B (0.2 + 0.4 > 0.6) was missing from the LP, so path_rr could return it with a score above L: Aix 30x30 B=0.6 gave a gap of −4.9e-5, and 22 of 400 random decimal-cost instances a negative gap, down to −0.28. The master's budget row stays at B, so a set of cost in (B, B + tol] can exceed L by about μ·tol.
  - Not done: local search (§3.4) and a ratio-ordered fill. The x* order is LP-driven: on a fuzz instance with x*_o1 = x*_o2 = 2/3 at B=2, the option order buys o1 (3.50) instead of o2 (6.65, the optimum).

- **Measured** against main cf1d235, single-threaded (2026-11-03): 278 configurations (real, scaled and fuzz instances; both pricings; `--max-activations`, `--evict-age`, `--relative-gap`, `--mip-start`, `--time-limit`).
  - Main's cast exceeded the budget in 74 of them, the new solution in none.
  - The LP and L are bit-identical except with enumerate on Aix 30x30 B=0.6, where the columns now reach B + tol. The lowest printed gap is −1.5e-15.
  - Where the cast fitted the budget, the new solution contains it.
  - Share of the optimal gain, against prep_mip or mip:

    | Instance | Budget | Share |
    |---|---|---|
    | Quebec | 100 / 1000 / 10000 | 0.43 / 0.95 / 0.84 |
    | bike_small | 50 / 100 / 200 / 400 | 1.00 / 0.83 / 0.71 / 0.98 |
    | Aude | 3 / 8 | 0.82 / 0.95 |
    | Aix 30x30 | 4 / 8 | 0.96 / 0.93 |
    | Aix 70x70, the owner's case | 10 | 0.999 |

    On Aix 70x70 B=10 the solution has cost 10, score 2,647,561 against 2,648,030, and a gap of 0.12%.
  - Fuzz, both pricings, default options: 162 of the 173 configurations with a positive optimal gain reach the optimum; the lowest share is 0.47.
  - With prep_mip's optimum as `--mip-start`, path_rr returns it with a certified gap: Aude B=3 1.6%, Quebec B=1000 0.011%, Aix 30x30 B=8 0.13%, bike_small B=200 0.042%.
  - The reviewers' harnesses, rerun on the final change:
    - 400 random decimal-cost instances: no bound below mip's optimum in any of the four pricing × start modes. Before the enumerate fix, 22 runs without a start and 27 with one printed a negative gap.
    - 427 adversarial instances, 15,570 runs: the only bounds below mip's optimum are on one instance built near the tolerance, where mip's solution exceeds B by 2.5e-7. That is within mip's default `-t` of 1e-6, not within path_rr's 1e-7.
- **Effect.** path_rr itself outputs a guarantee. LP rounding never beat greedy (Quebec B=10000: 1.21023e11 against greedy's 1.21052e11; study: primal-side), so the start carries the incumbent in practice. With path_rr's own bike bound the gap stays about 20%.
- **The gap is conditional:** a complete round, a shape the pricing accepts (or `--pricing enumerate`), a criterion path_rr can model, and the tolerance-sized clamp hole of §2.3. The owner's ruling leaves tree_rr without rounding for now (work in progress, do not change its cast); for path_rr, question 2 of §6 was answered yes on 2026-11-03.

### 3.2 Time limits that hold, and the LP status (item 3)

- **Status.** Small fixes (study: decision page).
- **Problem.**
  - Three places ignore `--time-limit`. The master LP solve: bike B=1000 ran 870 s at a 600 s limit (029471d). The enumeration parallel_for. A single long search: the generic budget-mode prototype on bike B=1000 ignored a 420 s `--time-limit` and was still running when a 600 s timeout killed it, about 180 s later, without finishing round 1 (study: generic-pricing).
  - The LP status is not checked. mippp's `gurobi_lp::solve` stores a non-optimal status without throwing, and path_rr reads ObjVal and Pi regardless. With BarIterLimit=1, the barrier prototype reported "converged" with a bound of 1279.25, below the optimum 1337.595 (r1_sumc B=1; study: master-acceleration, verifier).
- **Rule.** A valid L needs an optimal RMP solve and a complete round at its duals. A stopped run reports the last L that met both.
- **Change.** Pass the remaining time to the LP solver. Check the clock in the enumeration loop, and every N labels inside a search; an aborted search then returns the overestimate of §3.7. After each solve, check `mippp::is<mippp::status::optimal>`, and re-solve with dual simplex or stop without recording a bound.

### 3.3 Incremental evaluator under greedy_incr (item 4)

- **Status.** Prototype reviewed sound with fixes (study: primal-side); about 300 + 40 lines.
- **What.** One most-reliable-path table d_s(w) = P_sw(S) per source and case.

```
T_s     = Σ_w q_w(S) d_s(w),     PC_k(S) = Σ_s q_s(S) T_s
ADD A:  seeds at v with d_s(u) p'_A(a) > d_s(v) on the improved arcs a = (u, v);
        a max-product Dijkstra from the seeds keeps a label only if it beats d_s
DROP o: recompute s by a full Dijkstra iff an arc a = (u, v) that o improves is tight:
        d_s(u) p_S(a) ≥ d_s(v) (1 − 1e-12)
```

- ADD is exact because p ≤ 1 and max semantics make labels only grow, so the seeded closure is the new fixpoint. DROP is exact because a non-tight arc lies on no optimal path (prefix optimality). Exact per-case values make every criterion exact.
- **Effect.** The same choices as greedy_incr on 13 instance/budget pairs. Parity fails off-sample: on dupq (an all-zero row of option X before a second row at the same vertex) the prototype buys X = 1.0, while greedy_incr and prep_mip buy Y = 1.25; the touched flag below restores it (verifier). Bike B=1000 / 5000 / 10000: 74.2 / 608 / 1143 s → 1.769 / 12.513 / 22.664 s (16 threads). Quebec: 0.27–0.83 → 0.08–0.11 s. Green: 38.4 → 5.4 s (4 threads). One full evaluation takes 15 ms on bike (16 threads).
- **Fuzz** (600 random instances with cycles, probability-1 arcs, multi-arc options, several options per arc, duplicate rows): committed scores are exact. Predicted gains are wrong when an all-zero quality row precedes another row of the same vertex, by up to 300% on 222 of 300 instances; green and bike, the instances checked, have none (study: primal-side, verifier).
- **Fixes.** A per-vertex touched flag instead of the zero-gain test; test every predicted gain against a full evaluation; run after `trivial_reformulate_instance`; costs from `compute_solution_cost` with the tolerance; a memory cap with a fallback to `parallel_pc_num` (|sources| × |V| doubles per case: 11.7 MB on bike, 37 MB on green, 236 MB on Aix 70x70); undo logs instead of state copies.

### 3.4 Local search, bundle greedy and a criterion-aware key (item 5)

- **Status.** Prototype reviewed sound with fixes (study: primal-side); about 200 lines. The criterion-aware key is research.
- **Moves**, from S with residual R = B − c(S). N1: drop one option, refill greedily with positive gains. N2: drop a pair among the m options with the least loss per cost (m = 12–20), refill. N3: insert one of the k options of largest gain (k = 20–40) beyond the budget, drop by least loss per cost until c(S) ≤ B, refill. The first improvement above 1e-10 relative is taken. Every move ends budget-feasible and is scored exactly, so the result is a valid incumbent, with no approximation guarantee.
- **Bundle greedy.** Synthetic purchase duals λ_po := θ c_o, with θ a multiple of the median log-gain per cost. Per source, a shortest-path tree on ℓ(a) = min(−log p_S(a), min_{o∉S, c_o≤R} [−log p'_oa + θ c_o]); the options on the tree path to t form a bundle. Greedy then picks among single options and bundles by exact gain per cost. This is path_rr's pricing with λ = θ c collapsed to one objective: it buys complementary chains whose links alone gain little.
- **Non-submodularity.** 616 of the 1,620 options in primal-side's gain sweep (bike has 1,642) gain more at the B=1000 local-search state than at the base state (study: primal-side). So lazy (CELF) greedy is invalid, greedy carries no guarantee, and the incumbent's quality can only come from a bound.
- **Effect.**
  - Bike B=1000: greedy 6.39784e6 → 6.418970948e6 (+0.330%) in 88.2 s, the same 22-option set as prep_mip's 600 s incumbent. Bundle greedy alone: 6.418949702e6 in 30.2 s.
  - Aix: greedy's gaps of 0.58% / 2.74% / 0.92% (aix30 B=4, B=8, aix40 B=2) closed to the optimum in 0.6–4.5 s.
  - Share of greedy's gap closed: bike_small 16% / 100% / 100% at B=200 / 400 / 800; Quebec 76% / 9% / 0% at B=1000 / 5000 / 20000.
  - Bike B=5000: +0.0006% in 474 s. Bike B=10000: +0.045%, interrupted at 481 s. Bundle greedy reached only spend 1537 in 560 s at B=5000 and made no step at B=10000; against greedy at equal spend its edge is +0.38% / +0.02% interpolated (not the analyst's +0.69% / +0.20%), and −0.07% against the next greedy point. It needs a pre-filter (verifier). Green: no move in 459 s; bundle greedy +0.040% (gap 9.283% → 9.240%). In the verifier's words, "no improvement found in the time allowed": the neighbourhoods were truncated (study: primal-side, verifier).
- **Criterion-aware key.** Under φ = min_k, an option that improves a non-binding case gains 0. On min3 (three one-arc cases, unit costs, B=3), greedy_incr and static_incr buy {a1, a2, a3} = 2.5, while the optimum {a1, b1, c1} = 3.0 (study: primal-side, verifier). Proposed: order by (φ, then Σ_k PC_k), and allow zero-gain refills and inserts that improve the second key. Not prototyped.
- **Fixes.** Undo logs; cached drop losses; explicit time checks in the repair loops; pre-filtered bundles; run N3 at B=10000 before concluding anything there.

### 3.5 Pruned search pricing (item 6)

- **Status.** Prototype reviewed sound with fixes (study: master-acceleration; an A* variant in study: lagrangian-dual); about 50 lines, plus a 15-line label-filter hook in melon's biobjective_dijkstra.
- **What.** One potential per case and target, from one reverse max-product Dijkstra on the split graph with every option free:

```
P̄_t(v) = max over paths v → e ∈ E_t of (Π_a p_a) q_e / q̄_t
drop label (P, D) at v  if  π_k q̄_t P P̄_t(v) (1 + 1e-12) − D < σ_p + max(margin(p), best rc so far)
```

- It is valid because P̄ bounds the remaining probability times the end quality, λ̃ ≥ 0 so D never decreases, and the cut is strict. The search returns the exact max rc when it exceeds margin(p), else margin(p). It generalises the early stop of §2.5.
- **Effect.**
  - Same duals, bike B=1000 barrier rounds 2–3: pricing 110.7 → 23.4 s and 161.4 → 28.6 s (4.7–5.6×; study: master-acceleration, verifier).
  - Exact, center copies included: Quebec B=1000 gives the identical converged LP 118257927545.30 and bound 118257931979.41; bike_small identical 349-round sequences; 64 tiny instance-budgets without a bound below the optimum.
  - Nothing on Quebec, where pricing is about 1% of the time. Under today's capped master, pricing is 0.2–1.3 s per round on bike against 5.7–155 s master solves, and about 10% of a green round, so pruning matters only with dense duals (barrier, Lagrangian, completion; study: lagrangian-dual, verifier).
  - The A* variant, a label vector with predecessor pointers that also removes the backtrack: 0 mismatches against melon over 87,163 bike_small pricings, all single-arc (A* on center copies is checked only indirectly: Aude to 9 digits, 40 random instances); 0.49 / 4.6 / 8.2 / 25 s per iteration against melon's 0.88 / 22 / 56 / 224 s on the same Lagrangian trajectory (study: lagrangian-dual).
- **Fixes.** Cap the potentials' memory (#targets × #split vertices doubles per case: about 5 MB on bike, about 20 MB on green) and fall back to no pruning above the cap. Search mode only. It needs 560ce44, since the potentials inherit the split graph. §3.6a, §3.8 and §3.9 reuse the same potentials: implement them once. Since `path-rr-simplify` there is no split graph and the search runs backwards from t: the potential of a label at u is the most reliable u ← s probability with every option free, one Dijkstra per source on G_k, and it would replace the q̄_s bound of §2.5's pruning.

### 3.6 Exactness tools for instances that converge (item 7)

- **Status.** Analyst write-up, no separate review (study: tighter-relaxation); about 150 + 80 + 120 lines. These tools turn the 0.01–0.2% LP gaps of §1.4 into proofs. prep_mip already solves most such instances fast (Quebec, Aude and small Aix in 0.02–8.4 s).

**(a) Lazy budget pricing.** Labels (P, D, W) on the split graph, with W = Σ c_o over the options paid.

```
extension: (P p_a, D + λ̃_{p,o(a)}, W + c_{o(a)});   prune W > B (1 + 1e-12)
dominance: P1 ≥ P2, D1 ≤ D2, W1 ≤ W2.  Labels pop by decreasing P, so this is a 2-D staircase query
           over the settled labels, O(log k), and exact
A* prune:  π_k P h_t(v) − D − σ_p ≤ max(margin(p), best rc),   h_t with every option free
lazy rule: run the 2-D (P, D) search; run the 3-D search only if the 2-D argmax costs more than B
```

- When the 2-D argmax fits the budget, it attains a maximum at least the budgeted one. On a label cap, the 2-D value is a valid overestimate. The LP becomes z_budget.
- bike_small B=200: 7.58% → 0.59% of the gain, 15.5 → 3.8 s. B=100: 4.40% → 3.18%. B=400: 0.78% → 0.45%, but 15.1 → 35.0 s. bike_mid B=200, bound after 300 s: +4.84% → +0.815%, at 895 MB against 205 MB.
- The prototype ran the 3-D search on every priced pair, so the lazy rule's speed is unmeasured, and it pruned on a strict cost > B. Its dominance is the T = ∅ case of the generic budget mode (§3.7), which the generic-pricing verifier fuzzed: equal to enumerate 3,095 times.
- Intractable on full bike at B=1000, and a no-op at B ≥ 5000 and on green (§5).
- Fixes: implement the lazy rule itself; the B(1 + 1e-12) tolerance; a per-pair label cap of 2e6 with the 2-D fallback; shared potentials (§3.5).

**(b) Knapsack-hull end phase.** Dantzig–Wolfe columns over the 0/1 knapsack:

```
x_o − Σ_j z^j_o ν_j ≤ 0    [α_o]                 Σ_j ν_j ≤ 1    [η]
K(α) = max{α·z : c·z ≤ B, z ∈ {0,1}^O};   add z = argmax when K(α) > η
L = z_RMP + Σ_p r̄_p + max(0, K̄ − η),   K̄ ≥ K(α)
```

- Together with budget pricing (a) or enumerate, the master reaches z_hull at convergence; on search columns alone it reaches z_hull^S (§3.8). The DP floors costs on a grid of R = 1e5 steps. Every set with c ≤ B fits, so the DP value bounds K; the generated columns may slightly exceed B, which only enlarges the hull. A post-hoc K̄ must be a DP or Dantzig bound, or a MIP's dual bound, never a MIP incumbent.
- Proved optimal within 4–8e-8 relative, with lazy budget pricing (a) underneath on Quebec and bike_small, and enumerate on Aix: Quebec B=1000 in 5.3 s and B=10000 in 45 s (budget pricing alone: 4.0 s and 25 s, at +0.0105% / +0.0271%; search alone: 5.25 s and 28.8 s, at +0.0105% / +0.091%); bike_small B=50–400; Aix30 B=4 and B=8 (0.5 s, 1.4 s); Aix40 B=2 and B=4. Aix70 B=2 stays at +0.173%, its costs being unit.
- Run from round 1, it tails: one knapsack column per round, with oscillating bounds. bike_small B=800: 853 rounds and 178 s, with 1.14% of the gain left; B=1600 and B=3200 time out. Gating it to after the path LP converges is untested.
- Fixes: start it after convergence or when the gap target is unmet; stop on `--relative-gap` or `--time-limit`; a valid K̄; report the relaxation level reached in algorithm_report.

**(c) Column-cover cuts, enumerate mode.** For a pair p and an option set T, let 𝒮_p(T) = {columns S of p : S ∩ T = ∅, c(S) + c(T) > B}.

```
Σ_{S∈𝒮_p(T)} y_pS + Σ_{o∈T} x_o ≤ |T|
```

- Valid: y_pS = 1 forces x_S = 1, and S ∪ T does not fit; at most one column per pair is used. T = ∅ gives y_pS = 0 for c(S) > B. It is not implied by z_hull when T alone fits.
- Separation, per pair and active column S: a covering knapsack over T disjoint from S, min Σ_{o∈T} (1 − x_o) s.t. c(T) > B − c(S), filled greedily by (1 − x_o)/c_o.
- Pricing subtracts from each column the duals of the cuts that contain it: a per-column check, exact in enumerate mode. In search mode it would need the cost and T as label resources, intractable at bike scale.
- The pool must stay minimal: a column S ∪ {u} with a useless u escapes every cut whose T contains u.
- Aude B=2: +0.4918% → optimum (12 + 36 cuts). B=3: +0.8535% → optimum (48 cuts). B=5: +2.2582% → +1.4412% in 3 rounds (132 cuts, 36% of the gap). bike_small B=800: at the hull solution, 81 of 352 pairs have a violated cut (none at B=200 and B=400). These ran as a Python/Gurobi LP outside gecot.
- **Ceiling: z_PPC.** On Aude, z_PPC equals the optimum at B=1, 2 and 3, and is +1.4076% at B=5 (full enumeration, 34 s, 6.5 GB). What remains beyond it is a multi-pair effect that no per-pair cut can close. z_PPC is a research diagnostic, not a solver component.
- Fixes: implement inside enumerate mode, with a minimal pool; measure on Aix70 B=2 and bike_small B=800.

### 3.7 Generic exact pricing for every option shape (item 8)

- **Status.** Prototype reviewed sound with fixes (study: generic-pricing); about +150 lines net. Its dominance is §2.5's since `path-rr-simplify`, on G_k without H_k: options stay live by the shape of their arcs, so no center matching, and its two quality-list defects are fixed there.
- **Graph H_k.** A single-arc option gets one parallel arc per (arc, option), even when several options improve the arc. A star option gets an improved copy, with centers assigned by bipartite matching (Kuhn). Every other option is *general*: its parallel arcs, plus a per-vertex list of its gains. A general option is tracked when λ̃_po > 0 (always, in budget mode); an untracked one is free.
- **Labels** (P, D, T), with T the sorted set of tracked general options already paid.

```
extension: single-arc arc, star copy:          D += λ̃_po
           arc of a tracked general o ∉ T:     D += λ̃_po, T ∪= {o};     o ∈ T: nothing charged
starts:    each copy of s × each subset A of the general options with a source gain at s (≤ 2^16)
end at e:  max over U ⊆ target options of  π_k P (q_e + Σ_{o∈U} g_ot) − D − Σ_{o∈U∖T} λ̃_po − σ_p
           separable: take o iff o ∈ T or π_k P g_ot > λ̃_po
dominance: ℓ1 ≽ ℓ2  iff  P1 ≥ P2  and  D1 + Σ_{o∈T2∖T1} λ̃_po ≤ D2
```

- Dominance keeps the maximum: for any later purchase set U, λ(U∖T1) − λ(U∖T2) ≤ λ(T2∖T1). It needs λ̃ ≥ 0, hence the clamp. Exact ties go to the label with fewer option uses. The early stop and the pruning work as in §2.5 and §3.5.
- **Budget mode.** A third resource W with the dominance W1 + c(T2∖T1) ≤ W2, and a knapsack end over the target's general options with the remaining budget. The LP is z_budget.
- **Effect.**
  - Equal to enumerate's LP on worst B=3 (8.06), test B=2 (5.5), Aix30 B=4 and B=8, Aix40 B=2 and B=4, Aix70 B=2 (2412298.1986), the bike crop at B=100, a 2-case min instance and biobj_aude B=5. Equal to search's LP on Quebec B=1000, Aude B=3 and B=8 and the bike crop at B=100.
  - Quebec B=1000 at parity: 5.48 s and 14 rounds, against 5.64 s and 16 rounds. Slower elsewhere: bike crop pricing 5.4–6.7 s against 3.6 s for melon's search; Aix70 B=2, 104 s against 20 s for enumerate.
  - **Aix N2 B=100** (576 general options, 1,801 arcs improved by several options): converged in 87.6 s and 134 rounds, LP 1463.0565, bound 1463.1893, about 3.8k labels per search. With greedy_incr's 1457.6057 the gap is ≤ 0.383%, 87.3% of the gain. Nothing else runs on N2: enumerate stays in setup over 200 s, and prep_mip runs out of memory at 16 GB. Budget mode changes N2's LP by ≤ 0.002%.
  - Budget mode: worst B=3 gives 8.06 against 11.13 unbudgeted (optimum 7.46). Its cost grows with the number of affordable options per path: bike crop B=100 1.08 s against 3.6 s for search, B=400 over 120 s against 14.4 s, full bike B=1000 round 1 unfinished in 600 s.
  - The fewer-options tie-break matters: without it Quebec B=1000 takes over 209 rounds and 120 s, instead of 14 rounds and 5.5 s. It is a heuristic (it counts option uses), valid because it only chooses among labels that dominate each other.
  - **`--probe`**: price every pair once at enumerate's converged duals. The slack Σ_p max(0, r̄_p) is 0 for an exact pricing of the same family, and otherwise measures what the pricing adds to L at the optimum. Exact: 0 on Aix70 B=2 over 43,180 pairs. It is the cheapest test of a pricing's exactness.
- **Defects found by the verifier.** Each reported an invalid bound with status "converged". None occurs on the measured instances.
  - **Duplicate quality entries.** trivial_reformulate_case contracts probability-1 components and appends each merged vertex's (source gain, target gain, o) tuples, so one option can carry several entries at one vertex; the input can too. A pricing that charges or chooses per entry underprices. Fuzz: 56 + 87 violations in 800 instances; on r253 the optimum is 10 and the bound 9.333. Fix: aggregate the entries per (option, vertex); 0 violations in about 25k runs after that. This applies to any pricing structure built on the case's quality lists.
  - **Double abort under the label cap.** The pair's bound fell to margin(p), and the empty result was cached: bike_small B=100 with `--max-labels 200` gave a bound of 116,702.17, below greedy's feasible 116,843.65. The rule for any capped or timed-out search: return r̄_p = max(best rc, π_k q̄_t P_popped − σ_p), or π_k V_p − σ_p; never cache it; count it; report a status such as "label cap reached". Labels pop by decreasing P and charges are ≥ 0, so no unexplored completion beats π_k q̄_t P_popped − σ_p.
  - **Uncapped target-subset loop** in budget mode: it hangs at 33 target options and returns 1.5 against an optimum of 2.0 at 65. Fix: the separable unbudgeted choice above 16 options. Source side: beyond 16 entries, start one label with every positive general gain at charge 0 (a valid overestimate) instead of throwing; the cap counts entries, so aggregate first.
- **Other fixes.** A time check every N labels; `--mip-start` support (best column within the start's options); keep the tie-break.
- **Scope.** Bike and green are unchanged, since their shapes are already priced exactly; their problem is convergence. N1 (up to 7.1M pairs, options of 2–240 arcs) is untested.

### 3.8 The direct Lagrangian dual (item 9)

- **Status.** Prototype reviewed sound with fixes (study: lagrangian-dual). Its density control is open research.
- **What.** Relax the purchase rows with free multipliers λ_po ≥ 0, and drop the master LP.

```
L(λ)     = Σ_k π_k C_k + Σ_p h_p(λ_p) + K(Λ)                              single case: π = 1
h_p(λ_p) = max( π_k v_p∅ ,  max_S [π_k v_pS − Σ_{o∈S} λ_po] )
Λ_o      = Σ_p λ_po,    K(Λ) = max{Λ·x : c·x ≤ B, x ∈ {0,1}^O}
subgradient:  g_po = x_o − [o ∈ S*_p]       (x the knapsack argmax, S*_p the pair's argmax)
step:         λ ← clip(λ − t s_p g),  s_p = gain_p = V_p − base_p,  t = θ (L − z_inc) / Σ s_p g²,
              θ = 2, halved after 10 non-improving iterations
```

- L(λ) ≥ z_IP for every λ ≥ 0: for a feasible X, each relaxed term λ_po (x_o − Σ_{S∋o} y_pS) is ≥ 0. By Geoffrion, min_λ L is the knapsack-hull LP of h_p's column family: z_hull when h_p ranges over c(S) ≤ B (colbudget, or enumerate's lists); with h_p as written (budget-free) it is z_hull^S, between z_hull and z_search and not comparable with z_budget. Aude: z_hull^S = z_search = 1,138,555.66 > z_budget = 1,129,774.92; bike_small B=100: z_hull^S ≤ 116,845.71 < z_budget = 116,853.65. Within one family, z_LP − z_LD ≤ K_LP(Λ*) − K(Λ*), with K_LP(Λ) = max{Λ·x : c·x ≤ B, 0 ≤ x ≤ 1} (§5's μB is the x-unbounded version). The difference is 0 with unit costs and an integer B, and largest when B holds few options of mixed costs.
- **Conditions.** λ ≥ 0, a missing entry counting as 0. h_p computed exactly or overestimated, with S = ∅ included. π the criterion's exact weights, never a solver's contribution dual (which can be 1 − ε). K replaced by an upper bound K̄: min(floor-grid DP, Dantzig), or a MIP's dual bound at MIPGap 0, never its incumbent. 560ce44 applied. A coarse grid hides the gain: at R = 20000 the floor DP exceeded K_LP at every iteration on bike.
- **Box.** λ_po ∈ [0, g_po] is a valid projection, with g_po = min(gain_p, π (p'_oa − p_a)⁺ F*_s(u) P*_t(v)) for a single-arc option, F* and P* the all-options-free forward and backward values, endpoint qualities q_s(O) and q_t(O) included. Above the cap, every column containing o is dominated by S∖{o}, so o's arc can be deleted from p's search. Snapping λ to the cap is not free (§5).
- **Effect.**
  - L(0) equals path_rr's round-1 bound (bike: 7,816,736.299526). On Aude it converges to the search LP, and with budget-limited columns to the enumerate LP.
  - bike_small: proved optimal (L − z ≤ 1e-9 z) at B=50 in 0.12 s, B=400 in 0.96 s and B=1600 in 1.7 s. At B=100 the best L is 116845.71, against the optimum 116843.65 and the converged path LP 116857.50; other passages of the same write-up give 116846.06. L falls below the converged path LP after about 60 iterations (0.3 s), while path_rr needs 289 rounds (4.1 s) to converge. The tail is slow: after 500 iterations B=200 stays at +0.033% and B=800 at +0.023% (verifier).
  - 40 random search-shaped instances: L reaches the optimum on 34 (36 with budget columns), against 5 for the converged path LP (verifier).
  - bike_mid: +0.0095% / +0.0084% / +0.020% above the optimum at B=200 / 900 / 1800, in 163–200 s, where path_rr's `lagrangian_bound` is still +4.84% / +3.65% / +2.88% above the optimum after 300 s, its master 0.34–1.89% below it (study: tighter-relaxation, analyst and its run logs).
  - **Bike B=1000:** 7.817e6 → 6.667e6 in 8 iterations and 561 s, at 806 MB. That is about 5× better in gap than path_rr's column-generation bound (7.72e6, reached only after 786–870 s; verifier; the analyst wrote 20×), and 3.2% looser than prep_mip's 6.459e6 at 8.1 GB. Its knapsack was in effect the LP knapsack (R = 20000, see Conditions), so integrality on full bike is unmeasured.
  - **λ′ passes** (study: master-acceleration): raise the master's budget dual μ to μ′, spread the slack c_o (μ′ − μ) as virtual duals, and re-price: the LP-knapsack version of L(λ). Bike B=1000, round 1, pruned, μ′ = 60: 7.817 → 6.830e6 in 5 passes and 340 s, which the direct Lagrangian's 6.667e6 beats; since that run's knapsack was in effect the LP knapsack, the edge comes from per-pair multipliers, not integrality. Green: 3.247e13 after 6 passes of 7–26 s, worse than the prototype column generation's 3.068e13. bike_small: 3.7e-4 relative above the LP in round 1 after 20 passes (0.3 s), then a stall on degenerate duals. They belong in this item, not as their own option.
- **Blocker.** Pricing per iteration grows from 0.49 to 138 s, and labels from 6.87e7 to 2.62e9 per iteration. The argmax columns shrink to about 5 options per pair, but every option a pair ever used keeps an intermediate λ, so its front stays biobjective. The analyst's projection of 50–100 iterations is unsupported: 2–4 hours at that rate. Green was never run.
- **Density controls to try.**
  - **Stale bound.** For any λ and λ′, h_p(λ′) ≤ h_p(λ) + Σ_o (λ_po − λ′_po)⁺, because each column's charge drops by at most that much. A pair that is not re-searched enters L(λ′) with this overestimate, and L stays valid. Re-search a pair only when its stale bound can still change the total. This extends f0da701's reuse. Untested.
  - A bundle method, or dropping the multipliers of options a pair has not used recently. Plain gain-scaled Polyak was equal to or better than unit scaling and a naive volume method on bike_small (§5).
  - Success criterion: bike B=1000 at or below 6.46e6 within 10 min, with pricing under 5 s per iteration for 50 or more iterations. The run would also measure full bike's converged LP.
- **Extensions, analytical only.**
  - **Multi-case,** through the criterion's conjugate: L(λ, π) = φ*(π) + Σ_k π_k C_k + Σ_p max_S [π_{k(p)} v_pS − Σ_{o∈S} λ_po] + K(Λ), with φ*(π) = sup_{z≥0} φ(z) − π·z. For a sum, π is fixed and φ* = 0. For a min, π lies on the simplex, φ* = 0, and π takes its own projected subgradient step. L(λ, π) ≥ z_IP holds for any φ; computing φ* by a small LP needs φ concave, so negative factors on min terms must be refused. A cheap valid fallback for a min: min_k UB_k, weak. The prototype refuses multi-case criteria.
  - **Disaggregated rows** for the shapes search rejects: one row per (p, o, element a) with multiplier λ_poa, and Λ_o = Σ_{p,a} λ_poa. Each (o, a) becomes its own parallel arc. It is a valid relaxation for every shape, equal to the split graph for single-arc options; relaxed charging (§5) is one fixed choice of its multipliers. Untested (verifier's proposal).
  - **Lagrangian fixing:** L(λ | x_o = 1) = Σ_k π_k C_k + Σ_p h_p + Λ_o + K_{−o}(B − c_o) below z_inc fixes x_o = 0, and L(λ | x_o = 0) below z_inc fixes x_o = 1. Untested; §5's fixing row measures the master-dual version.
- **Fixes.** 560ce44; the exact π; a valid K̄, reporting which one; check compute_solution_cost(x) ≤ B before promoting a knapsack x (ceil weights on a grid can exceed B by up to n·1e-9·B/R); state the 1e-9 tolerance with every optimality claim.

### 3.9 A faster master (item 10)

- **Status.** Prototypes reviewed sound with fixes, small effect (study: master-acceleration). Every barrier number below comes from a prototype without the LP status check.
- **Barrier without crossover** (Gurobi Method=2, Crossover=0). Same RMP. Barrier stops at an interior point of the optimal face, so its duals are central: a stabilisation at no extra cost.
  - Quebec: 1.75–2.7× fewer rounds (B=1000 16 → 6 rounds and 5.58 → 3.79 s; B=10000 14 → 8 rounds and 28.3 → 13.9 s; the round-2 master 17.04 → 1.46 s). bike_small B=200: 449 → 56 rounds, 20.4 → 3.27 s, with timings noisy at the 2× level.
  - Bike B=1000: each solve 5–7× faster, but interior duals defeat the reuse (§2.6), and pricing rises to 110–161 s per round unpruned. With pruning: 10 rounds in 430 s, bound 7.454e6.
  - Green: slower per solve (230 s over 21 rounds, against 265 s over 45), and it needs value scaling.
  - Validity: check the status (§3.2), and take z_RMP from the dual objective D, not ObjVal. With BarConvTol = 0.5 and status optimal, primal 1329.15 and dual 1340.57 gave a reported bound of 1329.15, below the optimum 1337.595 (r1_sumc B=1; verifier). At the default tolerance, primal and dual agree to about 1e-9 relative on bike and green.
  - Gurobi only (`native_api`) until mippp has an lp_algorithm selector.
- **Value scaling** (open item C5). Divide the contribution coefficients and constants by S (1e8 on green), and π by S in pricing.
  - The scaled LP equals the original divided by S iff φ is positively homogeneous: the criterion's constants must be divided too. The prototype did not divide them. With S = 1e3, sum(k0, k1, 1000) gave 1,000,337.6 instead of 1,337.6; min(k0, 120) gave a bound of 169.12 against an optimum of 120; sum(k0, −50) crashed (verifier).
  - Green's matrix range is 2e-4..5e11 with a right-hand side of 1e13; unscaled barrier hits the 1000-iteration limit. Scaled barrier with pruning reached 3.0767e13 in 300 s, against 3.1177e13 for unscaled simplex.
  - The pricing margin is then in scaled units: state it in unscaled ones.
- **Dual completion.** Virtual duals on missing rows, within the slack of x_o's dual row:

```
s_o = c_o μ − Σ_{existing rows (p,o)} max(0, λ_po),      λ̂_po ≥ 0 on missing rows,   Σ_p λ̂_po ≤ s_o
```

  - With the RMP's μ, (π, σ, λ + λ̂, μ) is an alternative optimal RMP dual: λ̂ enters only x_o's row, within its slack, and columns that are not in the RMP. So pricing with it is exact column generation, and L stays valid. It attacks the coverage problem of §4.3 directly.
  - Update, per improving pair: add rc_p/|M_p| to the options M_p of its best column that have no row and some slack, then project each option's vector onto {λ̂ ≥ 0, Σ_p λ̂_po ≤ s_o}.
  - bike_small B=200: 449 → 342 rounds, 20.4 → 13.1 s. Quebec B=1000: 16 → 15 rounds, negligible. Green: a per-round effect only (3.411 → 3.343e13 at round 2), and it invalidates the reuse key at every pass: 11.4M against 568k searches.
  - A no-op under barrier, whose interior duals already use the slack: barrier and completion are alternatives, not a stack. Its effect is modest on bike_small and unproven alone on green, where the 10.1% → 8.3% gap changed scaling, pruning and completion at once (verifier).
  - Fixes: slack max(0, raw + c_o δ) with raw unclamped; a separator before the virtual entries in the reuse key; enumerate still reads the raw purchase duals (a 2-line change).
- **Tolerance-free bound** (the verifier's proposal, not implemented):

```
L = D + B (μ′ − μ) + Σ_o max(0, Σ_p λ̃_po − c_o μ′) + Σ_p max(0, max_S [π_k v_pS − σ_p − Σ_{o∈S} λ̃_po])
```

  - D is the RMP's dual objective, λ̃ the clamped (and completed) duals, μ′ ≥ 0 any budget price (μ′ = μ in plain column generation). The bound holds for z_IP, using x_o ≤ 1 in the integer problem. It also needs σ_p ≥ 0 (derived for this note, not in the study), and π, with the duals of the criterion rows, must satisfy the dual constraints of the z_k and criterion columns exactly: take π as the criterion's exact weights (renormalised for min) and use that π in D and in pricing, or add Σ_k max(0, w_k − π_k) Ū_k, with w_k case k's criterion weight and Ū_k an upper bound on its score. A solver π of 1 − ε for a single case otherwise understates the bound by up to ε Ū (study: lagrangian-dual, verifier). Under simplex the basic z_k makes this term vanish to rounding.
  - It absorbs the barrier residuals on the x and y columns, the clamp of §2.3 and the slack of the λ′ passes. Under simplex it is expected, not measured, to agree with `lagrangian_bound` to about 1e-8.

### 3.10 Pair tolerance (item 11)

- **Status.** Prototype reviewed sound with fixes (study: reduction); about 25 lines. Useful on instances whose gain is concentrated in few pairs.
- **What.** For any set R of pairs and any V_p ≥ max_{c(S)≤B} v_pS:

```
score(x) ≤ Σ_k C_k + Σ_{p∈R} V_p + Σ_{p∉R} v_p(x)        for every budget-feasible x
```

- So moving R into the constant at V_p keeps a valid bound, looser by at most Σ_{p∈R} (V_p − base_p). Rule: per case, move the pairs with the smallest V_p − base_p while the cumulative slack stays ≤ ε · (case base score). It needs a true V_p (560ce44), φ nondecreasing, and base_p stored per pair rather than derived from the start column.
- **Effect.** Quebec at ε = 1e-6: pairs 146,987 → 9,168 / 181,521 → 19,215 / 183,381 → 19,672 and times 5.1 → 0.6 s / 27.9 → 2.0 s / 34.1 → 2.2 s at B=1000 / 10000 / 20000; memory 463 → 181 MB; bound +9.5e-7 relative. The top 1% of Quebec's pairs carry 98.2% of the gain. At ε = 1e-4 the bound rises by 9.5e-5 relative, as much as Quebec's whole LP gap. prep_mip already solves Quebec exactly in 8.4 s. Bike and green barely gain (§5).
- **Fixes.** ε = 0 by default, and ε ≤ relative_gap/10 when a gap target is given; report pairs_dropped and pair_bound_slack; keep base_value per pair. Enumerate mode drops pairs after enumerating them, so it saves no enumeration time.

## 4. Why the bike and green bounds stall

### 4.1 The relaxation is not the problem

- Where the optimum is known, the converged path LP is close to it (§1.4), and greedy is close too: bike_small 0 / 0 / 0.0193% / 0.0054% / 0.0006% below the optimum at B=50 / 100 / 200 / 400 / 800; Quebec 0.0109% / 0.0206% / 0 / 0.0182% at B=1000 / 5000 / 10000 / 20000 (study: primal-side).
- On bike B ≥ 5000 and on green, polishing barely moves the certified gap: bike B=5000 2.077% → 2.076%, B=10000 3.242% → 3.195%, green 9.283% → 9.240%. So the gap must close through the bound.
- The verifier's correction: the evidence that these incumbents are near-optimal is weak. prep_mip never left its root node there (nodes: 1), and the local search was truncated. The supporting evidence is the crops, and extending it to full bike is the extrapolation of §1.4. Green's 9.3% comes from an unconverged bound (study: primal-side, verifier).

### 4.2 Round 1 buys whole paths

- Round 1 has no purchase rows, so every λ_po = 0 and each pair's column buys every improvement on its all-free best path. The round-1 bound is the all-free value, Σ_k C_k + Σ_p V_p for a single case: L(0) of §3.8, independent of B in search mode.
- Full bike: a median of 32 options per column, at a median cost of 1,815; 75.0% of the columns exceed B=1000 and carry 88% of the free-options gain (study: tighter-relaxation, a Python Dijkstra over all 58,966 pairs; the reduction analyst's mean is 34.4 options per path). The owner's log excerpt shows 14–53 options per column (029471d).
- Activating them creates rows for every option on the path. Uncapped, bike's second master LP did not finish in 900 s (029471d).

### 4.3 Missing rows price options as free

- Under `--max-activations` a pair gets rows only when one of its columns enters the master. Until then its λ_po are 0, its rc keeps its all-free value π_k V_p − σ_p, and its term in L stays near its round-1 size.
- Bike B=1000, cap 5,000: about 54k of the 58,966 pairs never get rows; 575k rows by round 9; the bound stays at 7.72e6 against L(0) = 7.817e6 (study: master-acceleration). Green: only 5,000 of 268k pairs gain a column per round (mippp dual_smoothing §7).
- The verifier's correction: the missing rows are the cause. The unchanged-duals reuse of §2.6 is a symptom, cheap precisely because nothing is priced.
- Removing the cap is not an option on bike (§4.2). Dual completion (§3.9) gives missing rows prices without creating them; its effect is modest.

### 4.4 Informative duals make pricing the wall (bike)

- The search front is biobjective in (P, D). With every λ_po = 0, D = 0 and the front is one label per vertex: a Dijkstra. With positive prices on k options along a path, labels trading probability against paid duals stay non-dominated, up to one per subset of priced options.
- Measured: 20–300 ms per pair search under informative duals, against 0.4 ms at zero duals. A full pass over bike's 58,966 pairs takes 20–100 s even with pruning, about 27 ms per search on 16 threads. The crops needed 100–450 passes to converge (bike_small B=200: 449; study: master-acceleration; decision page).
- Every route that produces informative duals hits the same wall:
  - the barrier master: pricing 110–161 s per round unpruned, 23–49 s pruned (§3.9);
  - the λ′ passes: 21 → 99 s per pass (§3.8);
  - the direct Lagrangian: 0.49 → 138 s per iteration (§3.8).
- Pruning (§3.5) divides the cost by about 5, but does not remove it. Smoothing does not help: it moves the duals every round, defeats the reuse, and its best fixed α is 0 (§5).

### 4.5 Green: coverage and the master

- Green stalls differently. After f0da701 the master LP is about 90% of a round, growing from 1 s to 10 s. Barrier is slower per solve there, and needs value scaling (§3.9).
- The bound 3.0944e13 (f0da701, 600 s) is from an unconverged run. A prototype master with scaling, pruning and dual completion reached 3.0676e13 in 300 s, three changes at once (study: master-acceleration).
- Under λ′ virtual duals a green pass took 7–26 s (6 passes, bound 3.247e13, worse than the prototype column generation's 3.068e13). Under barrier duals green pricing rose from 0.2–0.8 s to 2.5–4 s per round. The direct Lagrangian dual was never run on green: its queued job never got the machine lock (study: master-acceleration, lagrangian-dual).

### 4.6 A good start gives no prices

- x_o's dual row reads Σ_p λ_po ≤ c_o μ, because x has no upper bound in the master (§1.2). If the RMP optimum leaves budget slack, then μ = 0 by complementary slackness, and every λ_po = 0.
- Greedy leaves slack: 999.7 of 1000 on bike, 198.5 of 200 on bike_small. With a greedy start, the round-1 bound on bike B=1000 is the cold start's 7.816736e6 (study: master-acceleration). On green from greedy, the master starts at greedy's 2.83154e13 and gains about 1e-5 relative in 45 rounds (c7754f5).
- Untested idea, not in the study: x_o ≤ 1 rows would add a dual ρ_o to x_o's row, but it is positive only where x_o = 1. The start still matters for the incumbent (§3.1).

### 4.7 Best certified gaps so far

| Instance | Incumbent | Bound | Gap | Gain share ≥ | Sources |
|---|---|---|---|---|---|
| bike B=1000 | 6.41897e6 | 6.45909e6 | 0.62% | 81.5% | both from prep_mip (600 s, 8.1 GB); local search reaches the incumbent in 88 s. path_rr's own bound: 20.3% (7.72e6); Lagrangian prototype: 3.9% (6.667e6 in 561 s) |
| bike B=5000 | 6.80864e6 | 6.95043e6 | 2.08% | 80.0% | greedy and local search, prep_mip's bound |
| bike B=10000 | 7.05972e6 | 7.28527e6 | 3.20% | 78.4% | greedy and local search (cut by its time limit), prep_mip's bound; prep_mip's own incumbent is empty |
| green B=5 | 2.83266e13 | 3.0944e13 | 9.24% | 36.0% | bundle greedy; path_rr's bound (f0da701, 600 s); prep_mip runs out of memory |
| Aix N2 B=100 | 1457.61 | 1463.19 | 0.38% | 87.3% | greedy_incr; generic exact pricing (prototype, 88 s) |

"Gain share" is the share of the possible gain over the base score that the incumbent is guaranteed to reach (study: decision page).

## 5. Measured and set aside

| Idea | Why not | Source |
|---|---|---|
| Wentges smoothing, fixed α (branch `path-rr-smoothing`) | Loses on Quebec at every α. At α = 0.5, B=1000: 15 → 32 master solves, 90k → 1.18M searches, 5.4 → 9.7 s, 7 → 10 rounds to the bound. Green α = 0.5: bound 3.1108e13 against 3.1186e13, worse in early rounds (3.59e13 against 3.41e13 at round 2); f0da701's reuse later reached 3.0944e13. It defeats the reuse. Adaptive α was not tried: the best fixed α is 0. Adaptive α and in-out separation do not address a coverage-limited master | a99eb98; mippp dual_smoothing §7; study: master-acceleration |
| Pool-first activation | On green the stale all-free columns bought many options, the master grew to 2.85M rows, and progress was worse. A pool-only round gives no bound | c7754f5 |
| Gurobi Method=0 for the master | Slower on green, where the master is about 90% of a round | f0da701 |
| `--evict-age` on path_rr (landed, §2.7) | Neutral or slower on Aude, Aix and Quebec; never run on bike or green. Quebec's 2.2 GB are stored columns, which eviction keeps | 0b3f2b2 |
| Enumerate at scale (landed, §2.4) | Green explodes at B=0.5; bike never finishes at B=50 (1.6e15 labels per pair at B=1000); N2 stays in setup over 200 s; Quebec B=10000 needs 2.2 GB against 457 MB for search | 8429409; c7754f5; study: generic-pricing |
| Budget pricing on full bike and green | Bike B=1000: the 3-resource search runs out of memory at 13.8 GB uncapped, and round 1 takes 240 s with a 2M-label cap (8,544 pairs capped); the generic budget mode does not finish round 1 in 600 s. A no-op at B ≥ 5000 and on green B=5, where 2.6% and 0.7% of the round-1 columns exceed B. It stays useful on converging instances (§3.6a) | study: tighter-relaxation, generic-pricing |
| Relaxed charging (λ_po/m_o per element of a general option) | Valid only after aggregating quality entries (instance 253: 27.57 against an optimum of 36 before). Weak: slack 4.9% on Aix40 B=2 and 7.9% on Aix70 B=2 against integrality gaps of 0.034–0.17%, because paths use 0.085–0.54 of an option's elements; column generation stalls (N2 bound +3.65%) | study: generic-pricing |
| Per-pair budget bound (resource-constrained Dijkstra, multiplier ν on cost) | Bike B=1000: 7.225e6 in seconds, 12.9% above greedy, against prep_mip's 0.62%; round-7 duals add nothing (7.2252e6). 99.9–100% of the budget-free bound at B ≥ 5000; green's gain ratio is 1.000 at B=5. Its cap can push L below z_RMP, so it would need its own stat | study: reduction |
| Integral knapsack at the master's own duals | At an optimal RMP dual with a tight budget, every option the RMP buys ties at ratio μ, so K_LP − K ≤ μ (B − the best tied subset sum ≤ B). bike_small: 0.0134 at B=100, ≤ 0.0036 at B=200–3200; only B=50 (0.707) closed the gap. Needs re-optimised duals (§3.6b, §3.8). The lagrangian-dual verifier recommended it, with the exact knapsack, for converging instances; this measurement overrides that | study: tighter-relaxation; lagrangian-dual, verifier |
| DW knapsack hull from round 1 | Tails: bike_small B=800, 853 rounds and 178 s with 1.14% of the gain left; B=1600 and B=3200 time out. Kept as an end phase (§3.6b) | study: tighter-relaxation |
| λ′ passes as their own option | The LP-knapsack version of §3.8: 6.830e6 on bike B=1000, which the direct Lagrangian's 6.667e6 beats (its knapsack was in effect the LP knapsack too, so the edge is the per-pair multipliers, not integrality); passes grow from 21 to 99 s; green 3.247e13 after 6 passes, worse than the prototype column generation's 3.068e13; they stall on bike_small after round 1 (degenerate duals) | study: master-acceleration, lagrangian-dual, verifiers |
| Snap-to-cap multipliers | Refuted: K(Λ) grows with Λ. One pair, options o1 and o2 of cost 1, B=1, values 10 and 9: L = 10 at λ = (5, 5), and 14 after snapping o2 to its cap 9 | study: lagrangian-dual, verifier |
| Unit step scaling, naive volume method | Equal to or worse than gain-scaled Polyak on bike_small | study: lagrangian-dual |
| Reduced-cost option fixing, then prep_mip | Bike: 0 options fixed at the 20.8% or 0.62% gaps. At the round-7 duals, at least about 736 of 1,642 options survive at any realistic gap (906 fixable at 0.001%, 2 at 0.1%). bike_small with converged duals: 58–66% at a 1e-5 gap, not the analyst's 85%. The fast prep_mip runs on 140 and 256 options used sets built from known solutions. The analyst's K form is invalid with criterion constants (cx2: L = 0.9 against 10.9). The primal-side argument from single-option gains omits the −μ c_o term of the reduced cost; its conclusion holds | study: reduction, verifier; primal-side |
| LP probing for fixing | Fixes 95 of 98 and 102 of 112 options on bike_small, but one column-generation run per option; 0 at a 0.3% gap | study: reduction |
| Pendant pair aggregation | −8.1% pairs on bike, −4.4% on Quebec, 0% on green | study: reduction |
| Useless-option and dominance tests | 0 of 1,642 bike options useless: each can improve 22,475 of 63,504 pairs on average. Dominance: no figure reported | study: reduction |
| Pair tolerance on bike and green (§3.10) | The gain is spread: bike 4.9% / 14.7% / 37.6% of pairs moved at slack 1e-4 / 1e-3 / 1e-2; green 0.7% to 25.7% at 1e-5 to 1e-2, where the 1e-2 slack (2.7e11) compares with a gap of about 2.6e12 | study: reduction, verifier |
| Aggregated purchase rows, pair-proportional prices | Analytical, no measurement. An aggregate of the per-pair rows is a relaxation: in dual terms λ_po = κ_o w_p, a far weaker LP. The equivalent per-source decomposition is tree_rr's pricing | study: reduction, master-acceleration |
| Rounding x* as the incumbent | Never beat greedy: Quebec B=10000 1.21023e11 against 1.21052e11. Bike's x* is 0.05–0.06 on 340–409 options. Kept only as one candidate in §3.1 | study: primal-side |
| Knapsack primal of the Lagrangian iterations | Bike B=1000: 6.275e6 against greedy's 6.398e6; Aude B=3: about 5% below the optimum. It helps only on small instances: it found the optimum on bike_small B=200, 400, 3200 and bike_mid B=200, 900 | study: lagrangian-dual, tighter-relaxation |
| Final crossover for the returned solution | Refuted: the cast exceeds B with or without it (bike: 1,504 options with barrier, 365–409 with simplex). The bound does not need it once D is used | study: master-acceleration, verifier |
| Bundles taken from path_rr's column pool | Untested and doubtful: pool columns are priced at per-pair duals (missing rows at 0, no budget), round-1 columns exceed R, and 65k–96k columns would need exact evaluation | study: primal-side, verifier |
| Column-cover cuts in search mode | Pricing would need the cost and T as label resources, intractable at bike scale; ignoring the cut duals is valid but loses most of their effect (§3.6c) | study: tighter-relaxation |
| Static and enumerate-list fallbacks of the Lagrangian | For shapes search rejects: h_p ≤ max_p makes λ only raise K, so it is minimised at λ = 0 and gives L(0), the trivial bound (22% over greedy on bike); enumerate's lists inherit its exponential enumeration. The disaggregated rows of §3.8, untested, would replace them | study: lagrangian-dual, verifier |

## 6. Open questions

The owner's six questions from the study's decision page, answered where marked:

1. **The fixes branch.** Merge `path-rr-search-fixes` as is? Should the parser also reject an improved probability below the base?
2. **Item 2.** May path_rr's returned solution change from the over-budget cast to a feasible incumbent, with best_bound and a gap (§3.1)? Answered 2026-11-03: yes, for path_rr only; tree_rr keeps its cast.
3. **Items 4 and 5.** Rebuild greedy_incr on the incremental evaluator? Should local search be `--polish` on the heuristics, or a solver of its own (§3.3, §3.4)?
4. **The certificate for bike-like instances.** Is prep_mip's bound (8 GB, 600 s) acceptable for now? path_rr would then cover the instances prep_mip cannot hold, like green and N2.
5. **Item 9.** Run the density-control experiment next (§3.8)? It also answers whether full bike's LP is as tight as its crops.
6. **Item 10.** Land the Gurobi-only barrier now, or wait for mippp's lp_algorithm? Value scaling (open item C5) is needed either way (§3.9).

## References

**gecot commits.**
- 0b3f2b2: `path_rr`, the path formulation relaxation, with columns enumerated once per pair, on mippp's column manager; `--evict-age`.
- 8ef534c: a pair's rows are created with its first columns in the master.
- c7754f5: `--pricing search` (biobjective search on the split graph, center copies, early stop, backtrack), the max-value skip, the reuse of unproductive searches, `--max-activations`.
- 1af6651: the Lagrangian bound of each complete pricing round, as `lagrangian_bound`, and `--relative-gap`.
- f0da701: a pair whose duals are unchanged reuses its last search, the column found included.
- 8429409: purchase duals clamped at 0 in the search pricing.
- 0488e2b: parallel arcs for options that improve a single arc.
- 029471d: search pricing by default, `--pricing enumerate` as the opt-in.
- 560ce44 (branch `path-rr-search-fixes`): a star option's copy arc keeps max(base, improved).
- d47ada5 (branch `path-rr-search-fixes`): depth-first backtrack over (vertex, label) states.
- a99eb98 (branch `path-rr-smoothing`, not merged): `--smoothing α`, Wentges smoothing with the mispricing schedule of Pessoa et al.
- Related tree_rr commits: 2089fd9 (relaxation_value is not a bound under inexact pricing), 33ccf0d (re-solve the master when the time limit stops a round that added columns), c983ada (skip a target at the duals of an unproductive pricing).
- Branch `path-rr-rewrite`: the rewrite that follows this note, on top of d47ada5, with the same results.
- Branch `path-rr-feasible` (2026-11-03): the returned solution fits the budget, the lowest L is the report's best_bound, and enumerate's columns are built within B + feasibility_tol (§3.1).

**mippp design notes.**
- `~/Projects/mippp/misc/design/dual_smoothing/dual_smoothing.md`: the Lagrangian value under smoothing and validity with rows added at dual 0; §7 is the path_rr field test.
- `~/Projects/mippp/misc/design/column_manager/column_manager_tree_rr.md`: the column manager path_rr uses; L1 (eviction before activation), L11 (age counts events) and X1 (solution reads after a model change).

**The study.** Its decision page is published privately at https://claude.ai/artifact/6CQTheQzG91s47ctU7Euvs. Its prototypes, run logs, the analysts' write-ups with the verifiers' verdicts, and the verifiers' counterexamples and fuzzers live in a session scratchpad, not in the repository, and disappear on reboot: `/tmp/claude-1000/-home-plaiseek-Projects-gecot/094cc502-712b-4c26-b6be-3b28e9a86c0c/scratchpad/design/` (`results/*.md`, one prototype folder per direction, `verify-*/`).
