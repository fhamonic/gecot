# Constrained (Lagrangian) preprocessing: design note

Design note on gecot's constrained preprocessing, `compute_constrained_strong_and_useless_arcs`, which `--nb-mus` turns on in `flow_mip`, `tree_lp` and, since branch `benders-preprocessing`, the Benders decompositions (today `multicut_benders`, `flow_benders` and `multiflow_benders`). It is written for the owner and future maintainers; §2 is the part gecot's user documentation can draw on, and it ends with proposed text. Dated 2026-11-06, it records everything that was tested about this preprocessing on 2026-11-05 and 2026-11-06: a review against the thesis, three validity fixes, budget and grid sweeps, a study of the multiplier order, end-to-end timings, and the implementation that landed. The code described is commit 4d4c14c on branch `prep-mip-tight-big-m`, squashed into main as 68e7391; the header is the same file in both (md5 d7af8040).

- **Sources.**
  - [R] The review of 2026-11-05, `misc/design/lagrangian_prep_review.md` (untracked), on 6c59214. It compares the code with the thesis (`~/Projects/these/tex/chap_preprocess.tex`, lines 319-567) and lists code defects A1-A7, thesis errors, the M3 refinements and numerics. 50 agents: five lenses, 44 adversarial verifiers, one critic.
  - [S1] Verification and sweep study, 2026-11-05/06: the three fixes verified (fuzz, assertion and UBSan builds), budget sweeps, multiplier-grid sweeps, the proposal of the 3-decade grid and its skeptic, end-to-end timings with Gurobi seeds, blue ties, a critic.
  - [S2] Order study, 2026-11-06: a new harness (osweep), theory with proofs, order sweeps on four instance groups, a judge, a skeptic, a CLI design built in a scratch copy, serial timings, a critic.
  - [S3] Verification of the implementation, 2026-11-06: regression test and build, a gate of the real binary against the harness, brute-force fuzz, code review, critic.
  - [S4] Verification of the final change to the cap, 2026-11-06.
  - [S5] Blue ties, 2026-11-06: proof with independent reviews, brute-force fuzz and exhaustive search, end-to-end timings, then the implementation on branch `blue-ties` and its verification (fuzz, real and raw instances, mutants, reviews). See §5.4.
- **Citations.** The tags above; a word after a tag names the agent inside a study, e.g. [S2 skeptic] or [S3 gate]. A commit sha or a header md5 marks a code state. Where an earlier agent and a later skeptic, critic or verifier disagree, the later figure is given; where the earlier one circulated, the text says it was corrected.
- **Header states.** 6c59214: reviewed by [R]. md5 df9a8581: 6c59214 plus the three fixes of §3.2; verified by [S1], and the header every [S2] harness copied. md5 80bb8a4e: `--mus-sweep`, the nested grid, the final μ = 0 and frontier; verified by [S3]. md5 26de0dac: the cap on the largest cost per arc and the two seed-order comments; verified by [S4], which also showed its labels identical to 80bb8a4e's under the CLI predicate. md5 d7af8040: 26de0dac with one comment reworded, committed as 4d4c14c. Branch `blue-ties` adds §5.4; every figure outside §5.4 was measured with red ties, unless marked.
- **Instances.** Budgets are in cost units; a percentage is a fraction of the raw total option cost.
  - **aix30, aix40, aix70** (Aix 30x30, 40x40, 70x70): raw total option cost 17.2, 28.4 and 61.4 [S1]. aix30 has 18 options; after reformulation 16 at B=1 and 17 at B=1.72 [S3 gate]. aix70 at B=2 has 19,880 arcs after reformulation, 213 targets and 65 options [S2]. The Aix CSVs carry the owner's uncommitted edits of 2026-11-02 (§11).
  - **aix_N2, biorevaix_N2** (Aix N2, BioRevAix N2): total option cost 10,743 and 10,877 [S1]; aix_N2 has 68,756 arcs [S2]; their flow_mip models have about 10.2M and 12.2M columns at budgets of 1% and above [S1]. **Aix N1, BioRevAix N1**: 478k arcs [S1], never swept.
  - **quebec** (Quebec_438_RASY): raw total option cost about 426,655 [S1]; 260 options; after reformulation 167 / 207 / 255 at B = 1000 / 2133 / 10000 [S3 gate].
  - **bike_mid, bike_small**, crops of bike: total option cost about 15,536 and 6,204 [S1]; bike_mid has 358 options (322 after reformulation at B=110, 346 at B=155) and 37 targets [S2 design, S3 gate]; 2,167 and 352 pairs (`misc/design/path_lp.md`, Instances).
  - **aude**: 15 options of cost 1 [S1].

## Summary

- **What it does.** For each arc uv, red/blue competing Dijkstras run once per Lagrange multiplier μ: option lengths carry a penalty floor(μ·c_o/|A_o|) and the blue side an offset floor(μ·B). A vertex t that blue wins is a target for which uv is on a shortest path (strong loop) or on none (useless loop), under every option set that fits the budget. μ = 0 is the unconstrained preprocessing restricted to affordable options. The positive multipliers add labels, which shrink the contracted graphs G_t and the flow_mip model. No invalid label was found in any brute-force phase (§4); the validity argument holds pass by pass, for any order (§1.3).
- **When it pays.** Only at small absolute budgets, where few options fit together. Contracted arcs removed over `--nb-mus 0`, best grid: up to 25% on Aix at B=1, 21% on bike_mid at 0.5% of the total cost, 12.5% on quebec at 0.5% (B=2133; 12.1% at B=1000, both with the shipped `--nb-mus 20`), 7.4% on aix_N2 at 0.5% (top:3:21; 7.2% for the positive points of `--nb-mus 9`) (§2.2, §5.1). Nothing from 10% of the total cost on quebec and bike_mid, from 20% on bike_small, from 2% on N2 (aix_N2 still 0.0026% there), from B ≈ 5.7-10 on Aix, and nothing at any budget on aude. Where it gains, flow_mip ran up to 2.5x faster end to end (bike_mid, median of 10 seeds, §9.3); where it gains nothing it only adds labelling time; and where the gain is small or the unconstrained solve is short, it was slower even with a smaller model (aix70 B=1: 17.0 s against 11.9 s for a 24% smaller model, with [S1]'s rule build at 16 threads and without frontier; the landed build was not timed there) (§9.1).
- **Recommended settings.** Turn it on only at small budgets. `--nb-mus 20` on small graphs (quebec, bike: labelling takes milliseconds), `--nb-mus 9` on large ones (aix70, N2: at aix70 B=2 and 4 threads, labelling is 53% (`--nb-mus 9`) to 69% (`--nb-mus 20`) of the run). Keep the default `--mus-sweep decades` (§2).
- **In the code** (4d4c14c, §10): `--mus-sweep decades|full`, default decades, which tests μ = 0, then N+1 multipliers geometric over [max(min_mu, max_mu/1000), max_mu] with the last exactly max_mu, then μ = 0 again; the grid formula lo·(max_mu/lo)^(k/N), which makes the grid of N a bitwise subset of the grid of 2N; frontier, which drops fiber seeds that relax nothing (labels unchanged; with the final μ = 0 included, `--nb-mus 9` preprocesses in 0.23-0.78x the time of the same grid without frontier and without the final 0); max_mu capped at 2^62/max(B, largest cost per arc, 1); the three fixes of [R] (grid by index, no heap seed pushed twice, options without arcs skipped); `--nb-mus` validated non-negative; the test `preprocessing.constrained_labels`.
- **Blue ties** (§5.4, branch `blue-ties`). The strong loop lets blue win ties at every μ, except on zero-length cycles, and the per-arc shares round down so that they cannot exceed the budget penalty. Contracted arcs: −27 to −48% on Aix, −0.2 to −0.4% on quebec, nothing on bike_mid. Blue `--nb-mus 0` runs aix70 in 0.39-0.94x the time of red and beats red `--nb-mus 9` at every budget; aix40 B=4 is slower through branching (1.55x), with identical LP bounds. The settings above were chosen with red ties.
- **Open** (§11). The decades window is anchored on max_mu, which a single cheap option on a near-zero-probability arc can move; `full` is better there. No N1 instance was run, no N2 instance timed end to end, no hard MIP timed. The 3 decades and the point counts were chosen on the instances that score them. Not implemented: the fixpoint with the idempotence skip (order-independent, 1.07-1.33x dec20's labelling counters, about 10 lines). Thesis errors to fix; small review leftovers.

## 1. What the constrained preprocessing computes

### 1.1 Lengths, options and multipliers

```
r              = --resolution (default 1e-8); one length unit = log1p(r) ≈ r nats
l^+_a          = floor(−ln max(p_a, r) / log1p(r))          base length, uint64 (prob_to_length)
l^o_a          = floor(−ln max(p'_oa, r) / log1p(r))        length of a once option o improves it
A_o            = arcs of option o in the case;   s_o = c_o / |A_o|       cost share per arc
admissible o   : predicate(o) and A_o ≠ ∅; the CLI predicate is c_o ≤ B
λ^μ(a)         = min( l^+_a , min over admissible o ∋ a of  l^o_a + floor(μ·s_o) )
P(μ)           = floor(μ·B)                                  budget penalty
θ(μ)           = μ·B·log1p(r)                                P(μ) in nats, for reporting only
min_mu         = min_o 1/s_o                                 a one-unit penalty on the dearest share
max_mu         = max over (o, a) with l^+_a > l^o_a of (l^+_a − l^o_a)/s_o
                 then max_mu = min(max_mu, 2^62 / max(B, max_o s_o, 1))
θ_max          = θ(max_mu)
```

- In the code, floor(μ·s_o) above is `trunc(nextafter(μ · nextafter(s_o, 0), 0))`, at most floor(μ·s_o) and one unit less only when μ·s_o is within rounding above an integer; P(μ) is `trunc(μ·B)` in doubles (§5.4).
- max_mu saturates every arc: from there on λ^μ = l^+ everywhere while P(μ) keeps growing, so a larger μ adds nothing [R §C6]. min_mu never adds a label (§5.3).
- A feasible option set X costs at most B. A label (t, uv) is **strong** if uv is on a shortest u-t path under the lengths of every feasible X, and **useless** if it is on none. `compute_contracted_generalized_flow_graph` contracts the strong arcs of t and removes its useless arcs to build G_t. Vertices with no target quality and no vertex option join the fiber but get no label.

### 1.2 One pass

The two loops run independently per arc uv, each over the multiplier sequence in test order. A pass is one melon `competing_dijkstras` run, with red lengths λ^μ and blue lengths l^+, on H_F = G minus uv minus every arc into the fiber F.

```
strong loop, pass μ    red  u at 0
                       blue v at l^+(uv) + P(μ)
                       blue w at D(w) + P(μ), for every live fiber vertex w ≠ v
useless loop, pass μ   blue u at P(μ)
                       red  v at λ^μ(uv)
                       blue w at D(w) + P(μ), for every live fiber vertex w ≠ u
                       the loop stops once v is in the fiber
every vertex blue wins joins F, stored as D(w) = key − P(μ)
ties: blue wins in the strong loop (strong_arc_default_traits), except that a zero-cyclic arc seeds v one
      unit higher (§5.4); red wins in the useless loop (useless_arc_default_traits)
labels: the fiber of the strong loop (strong) and of the useless loop (useless), over all passes
```

- **Canonical distances.** A stored D(w) does not depend on the multiplier, the order or the fiber in force: D(w) = l^+(uv) + d^+_{G−uv}(v, w) in the strong loop (one unit more for a zero-cyclic arc, §5.4) and d^+_{G−uv}(u, w) in the useless loop [S2 theory, Lemma 0, proved]. Checked on real instances: 0 mismatches over 12,850,174 fiber vertices (aix30 B=1 and aix40 B=4 on 25% arc samples, quebec B=2133 on the whole graph; orders asc, desc0last, no0desc, perm0:3 and REC; the checked counts equal fiber_S and fiber_U, so every fiber vertex was checked), and a +1 perturbation control flagged 733,779 of 733,779 [S2 critic]. The fiber therefore acts as a set (§7.6).
- **Cut characterization.** A vertex x outside F is claimed iff D(x) is finite and every red path in H_F to x is cut at some vertex q, meaning its red prefix length exceeds D(q) + P(μ) (or reaches it, in the strong loop under blue ties) [S2 theory, Lemma 1, proved]. The heap's tie order and melon's termination rule do not change the claimed set.
- **Seed order is load-bearing.** Each `add_*_source` call settles the red vertices closer than every blue seed pushed so far (`settle_red_prefix`). The explicit seed (v in the strong loop, u in the useless loop) has the smallest blue key of the pass, so it must be pushed first. Pushed after the fiber seeds, it loses labels, which stay valid; the order of the fiber seeds among themselves does not matter [S3 fuzz, controls p3 and p3r]. The landed header states this in a comment.
- **The useless-loop stop.** Once v is blue, red has lost its only source; the pass then claims everything u reaches, and every later pass claims nothing [S1 verify, proved]. The stop is exact.

### 1.3 Why the labels are valid

For a feasible X and a simple path Q: if X improves an arc a of Q through o ∈ X, then λ^μ(a) ≤ l^X_a + floor(μ·s_o); otherwise λ^μ(a) ≤ l^+_a = l^X_a. Option o covers at most |A_o| arcs of Q, so

```
λ^μ(Q) − l^X(Q)  ≤  Σ_{o∈X} |A_o|·floor(μ·c_o/|A_o|)  ≤  Σ_{o∈X} floor(μ·c_o)  ≤  floor(μ·Σ_{o∈X} c_o)  ≤  P(μ)
hence  l^X(Q)  ≥  λ^μ(Q) − P(μ)
```

- **Strong loop.** Invariant: for every w ∈ F and every feasible X, every u-w path avoiding uv is at least D(w) long, which is at least the X-length of the walk through uv. A new claim x: on any u-x path Q avoiding uv, take the first vertex y that is in F or claimed in this pass. If y ∈ F, the invariant extends to x. Otherwise the red prefix to y is cut, λ^μ(Q[u..y]) ≥ D(y) + P(μ), so l^X(Q[u..y]) ≥ D(y) ≥ d^X(u, y). Hence uv lies on a shortest u-x path. That is enough for each label, not for the contraction of several at once: see §5.4 for the guard and the proof. A zero-cyclic arc keeps the strict inequalities, and with them uv on every shortest path.
- **Useless loop.** Red wins ties, so the inequalities are strict; the same argument runs on the walk uv·Q[v..y], whose arcs are distinct because Q is simple from v [S2 theory (e), proved; R §B for the generalisation of λ^μ to several options per arc and to options covering several arcs].
- **No order assumption.** Only the current pass's μ and the invariant on F are used: neither the order nor μ = 0 coming first matters for validity [S2 theory]. Order matters only for strength (§7).
- **Floating point.** With products rounded to nearest, the shares along a path can exceed P(μ) by one unit. Red ties absorbed that for strong labels, not in the useless loop, and blue ties cannot. The shares now round down twice, which restores S ≤ P exactly (§5.4).
- **Integer model.** Labels are exact for the integer lengths; the end-to-end loss is relative, about P*·(h−1)·r for an h-hop path (§3.5).
- **Slope lemma** [S2 theory, proved]. If the options that improve arcs reachable from the red source in G−uv (plus uv itself in the useless loop) cost at most B in total, then P_μ(F) ⊆ P_0(F) for every μ and F: after μ = 0, no positive multiplier adds a label for that arc and loop. A gain needs a red path whose improving options cost more than B together, which is why gains appear only at small budgets. On fuzz, 188,013 of 832,304 (arc, loop) pairs were eligible on 20,000 reformulated instances and 93,587 of 413,124 on 10,000 raw ones; none grew after μ = 0.

### 1.4 Mapping to the thesis

Thesis lines 319-420 were read for this note; algo:beta_strong (line 470) and algo:beta_useless (line 533) are described here through [R].

| Thesis (chap_preprocess.tex) | Code |
|---|---|
| l^+_a, l^-_a | `base_length_map`, `prob_to_length(enhanced_prob)` |
| c_a, a cost per arc | the share s_o = c_o/\|A_o\|; several options per arc take the min |
| λ^μ_a = min(l^-_a + μ·c_a, l^+_a) | λ^μ(a) = min(l^+_a, min_o l^o_a + floor(μ·s_o)) |
| λ^+_uv = l^+_uv + μβ | the blue offset P(μ) = floor(μ·B) on every blue seed |
| T ⊆ 𝔽^β(u,v), grown pass after pass from T = F(u,v) (line 508 fixes no order of μ) | `fiber_map` / `mu_fiber`, carried from pass to pass |
| H: G minus the arcs into T except uv | subgraph filter `!fiber_map[target(a)] && a != uv`, for both colours |
| algo:beta_strong, algo:beta_useless | the strong and the useless loop |
| (no grid of μ) | min_mu, max_mu and the grid; min_mu is an addition [R] |

- **T handling.** The thesis states two versions (the graph H, and the printed algorithm, which bars only red from T); the code removes arcs into T for both colours. All three give identical output on every random and real instance [R §B].
- **Source u in the useless loop.** The code makes u a contestable blue source at floor(μβ); the thesis fixes u blue at 0 and puts μβ on u's out-arcs. They agree on positive lengths. With zero-length cycles the code's version is the valid one: the thesis version produced 1,547 invalid labels in 3,000 random instances [R §B].
- **Ties.** The strong loop lets blue win ties, as the thesis does (lines 126, 486-488), plus a guard on zero-length cycles that the thesis lacks and needs (§5.4). From 83729a6 (2023-06-27) until branch `blue-ties` it let red win them, in both headers [R §B].
- The thesis's own errors are listed in §3.3.

### 1.5 Names used in this note

- **Grid specs** (the harnesses' names). `unc`: the unconstrained header, i.e. `--nb-mus 0`. `cur:N`: the df9a8581 grid, μ = 0 then N+1 points over [min_mu, max_mu] built by index, the last exactly max_mu; never committed on its own, its labels equal those of today's `--mus-sweep full --nb-mus N` in every comparison made [S3 fuzz, S3 gate], though nothing guarantees it (§6.4). `old:N`: the product loop committed at 6c59214, which drops max_mu on Aix at N = 5, 8, 10, 11, 30 and 40 (§3.2, A1). `top:D:N`: N points geometric over [max(min_mu, max_mu/10^D), max_mu], the last exactly max_mu ([S2]'s osweep definition; [S1]'s sweep harness used [max_mu/10^D, max_mu], without the max, and the two differ only where min_mu > max_mu/10^D); the positive points of `--mus-sweep decades --nb-mus N` are `top:3:(N+1)`. `dense:N`: N points over [min_mu, max_mu]. `halve:N`: max_mu·2^−k for k = 0..N−1. `theta:` lists: μ = θ/(B·log1p(r)), clamped to max_mu.
- **Orders.** `asc` = 0, p1..pn; `desc` = 0, pn..p1; `asc0last` = p1..pn, 0; `desc0last` = pn..p1, 0; `no0asc` and `no0desc` drop μ = 0; `bisect`, `zigzag`, `perm:SEED`; `oracle` = decreasing singleton gain. "+0" appends a final μ = 0.
- **Strategies.** `dec9`, `dec10`, `dec20`: `--mus-sweep decades --nb-mus 9/10/20`, i.e. top:3:10, top:3:11 and top:3:21, run as 0 / ascending / 0 with frontier. `REC` = dec20, the recommendation of [S2]. `rule`: top:3:10 ascending, without the final 0 and without frontier, a scratch build run as `--nb-mus 9` (the timed variant of [S1] and [S2]). `nm0`, `nm1`, `nm3`, `nm10`: the CLI at df9a8581 with `--nb-mus` 0, 1, 3, 10 (cur:N grid, no frontier).
- **Metrics.** Labels = strong + useless (t, uv) labels, summed over cases; the trace prints them per target. gt_arcs = Σ_t arcs of G_t. A gain is counted over unc. In [S1], a "share" is a grid's gt_arcs gain as a fraction of dense:40's; it can exceed 100% because dense:40 does not contain the best points. In [S2], "% of the gain" is a difference against top:3:10@asc as a fraction of top:3:10@asc's label gain, and "pp" a difference of two such fractions. [S2] costs are deterministic counters: seeds + relaxation pushes + pops, both loops.

## 2. Options and recommended use

This section is the seed for the user documentation.

### 2.1 The options

| Option | Values | Meaning |
|---|---|---|
| `--nb-mus N` | integer ≥ 0, default 0 | 0: the unconstrained preprocessing. N > 0: the constrained preprocessing, with N geometric steps, i.e. N+1 positive multipliers tested after μ = 0. Negative values are rejected. |
| `--mus-sweep` | `decades` (default) or `full` | `decades`: μ = 0, then N+1 multipliers over [max(min_mu, max_mu/1000), max_mu] ascending, the last exactly max_mu, then μ = 0 again; at most N+3 passes per arc and loop (fewer when points coincide, §10.5, or the useless loop stops, §1.2). `full`: μ = 0, then N+1 multipliers over [min_mu, max_mu], no final μ = 0; at most N+2 passes; the labels of the df9a8581 build (the three fixes, before the sweep change) in every comparison made (fuzz at N = 1, 2, 9, 10 and 20; 16 real cells at N=10), not guaranteed, since the new formula moves the grid points by a few ulps (§6.4). Ignored when `--nb-mus` is 0. |
| `--resolution r` | default 1e-8 | The length unit. It also sets the clamp p < r, hence max_mu on instances with near-zero probabilities (quebec, §5.2). |

- **Who exposes them.** `flow_mip` and `tree_lp`, through `add_preprocessing_options`; not `mip` (removed since) or the three Benders commands [S3 testBuild, S3 review, S3 critic], nor `path_lp`: at 4d4c14c `add_preprocessing_options` is called only from `preprocessed_mip_interface.hpp` and `tree_formulation_lp_interface.hpp` (code). tree_lp on bike_small B=31 with `--nb-mus 9` runs 0, 10 points, 0 under decades and 11 entries without the final 0 under full [S3 critic].
- **Spread under 3 decades.** Then decades is full plus the final μ = 0: min 1, max 50 and N=1 give the sequence 0 1 50 0 [S3 review].
- **Trace** (`-v`). "Min µ" is the grid's lower end, "µ sequence" lists the multipliers in test order. The table prints the μ = 0 row, then one row per multiplier that grew a strong or useless fiber, the final μ = 0 included; vertices without a label count as growth. Its values are cumulative average fiber sizes per arc, not label counts.
- **Library.** The function takes `(num_mus = 10, decades_sweep = true)`. Its default predicate cannot be used (§10.4).

### 2.2 When it pays

The gain depends on the absolute budget, not on its fraction of the total: at 10% of the total cost, cur:10 removes 9.4% of the contracted arcs on aix30 but 0.29% on aix70, while at the same B=1 the two agree, 22.8% and 24.8% [S1]. Contracted arcs removed over `--nb-mus 0` [S1, S2]; figures are [S1]'s unless marked [S2], and on the unmarked Aix and bike_mid rows [S2]'s maxima are within 0.1 pp of [S1]'s:

| Family | Largest reduction measured (best grid) | Nil from |
|---|---|---|
| Aix 30/40/70 | aix30 23.0% and aix70 24.9% at B=1; aix40 (not run at B=1) 11.8% at B=1.42; in each case the first budget where the cost-1 options fit | B ≈ 5.7-10 (θ_max ≳ 1 nat), whatever the fraction |
| quebec | 12.5% at B=2133 (0.5%) and 12.1% at B=1000 (0.23%) with `--nb-mus 20` [S2]; [S1]'s best, dense:40, 11.7% at B=1000 | 10% (B=42,665); 0.06% at 5% |
| bike_mid | 21.3% at B=78 (0.5%) | 10% (B=1,554); 0.34% at 5% |
| bike_small | 2.3% at B=62 (1%) with `--nb-mus 20` [S2]; [S1] 2.2% at B=31 (0.5%) | 20% (B=1,241) |
| aix_N2 / biorevaix_N2 | 7.4% (top:3:21 [S2]) / 3.75% (top:2:5) at 0.5% | 2% (aix_N2 0.0026%); 5-20% not run |
| aude | 0 at every budget | – |

- A rule "skip when θ_max ≳ 1 nat", which fits Aix, fails elsewhere: quebec still gains at θ_max = 227 (B=1000) and aix_N2 at θ_max = 122.6 (0.5%) [S1].
- Where the model shrinks, flow_mip can run much faster: on bike_mid the 3-decade variants (rule, dec9, dec10, dec20) ran 1.7-2.5x faster than `--nb-mus 0` (medians of 10 seeds; the old `--nb-mus 10`: 1.3x at B=155, 1.7x at B=110) [S2 timing], and at aix70 B=2 the 3-pass `--nb-mus 1` of df9a8581 (no frontier, 16 threads) went from 30.5 s to 13.5 s [S1 timing]; no source compares `--nb-mus 0` with the shipped `--nb-mus 9` on aix70 at the same thread count. The model size does not predict the time on small graphs (§9.4).

### 2.3 How many multipliers

- **`--nb-mus 20` on small graphs.** Against 10 points (`--nb-mus 9`, both run as 0 / ascending / 0), 21 points add +6.7 / +10.2 / +5.8 / +2.3 / +8.0 pp of the label gain on quebec at B = 1000 / 2133 / 4267 / 8533 / 21333, and +1.7 to +12.4 pp on bike (bike_mid B=155 +6.05, bike_small B=100 +12.4) [S2 judge]; at quebec B=10000 `--nb-mus 9` finds more labels (Nesting, below). On bike, 11 points already give 21's labels, because both grids contain 0.501·max_mu [S2 skeptic]. On small graphs the extra points cost 8-64 ms over `--nb-mus 9` (gecot's own times: bike_mid B=155 22 → 30 ms, quebec B=2133 118 → 182 ms) and remove up to about 2 pp more of the contracted arcs: bike_mid B=155 14,307 against 14,673 of unc's 17,879, quebec B=2133 193,675 against 196,279 of 221,314 [S2 judge, S2 timing]. The advantage is in labels and model size only: no solve-time gain could be credited to the extra points, since dec20 beat dec9 at quebec B=2133 but dec10 was as fast there with 1,887 more variables, dec20's range overlaps dec9's at bike_mid B=155, and dec9 was as fast or faster at quebec B=1000 and bike_mid B=110 (1.76 s against dec20's 1.83 s) [S2 critic, gap 1] (§9.3, §9.4).
- **`--nb-mus 9` on large graphs.** On aix70 and N2, 21 points remove under 0.2% more contracted arcs than top:3:10 ascending (aix70 B=2: 0.08%; aix_N2, whole graph, 21 points without the final 0: 0.17%) [S2 judge], for 1.73-1.77x the labelling time of dec9 (1.65x on aix40 B=2) [S2 timing]. End to end at aix70 B=2, 4 threads, 2 seeds: dec9 23.0 s, dec10 23.5 s, dec20 31.3 s [S2 timing]. 10 is no better than 9 there: never better on the large cells, 1.06-1.07x slower to label, and on the aix_N2 sample it gains 66,609 labels against 67,296, fewer even than plain top:3:10 [S2 timing, S2 critic].
- **What "large" means.** Graphs where one labelling pass costs more than about 0.5 s [S2 skeptic]: aix70 (about 0.93 s per pass at 16 threads without frontier [S1 timing]) and N2.
- **Nesting.** The grid of `--nb-mus N` is a bitwise subset of the grid of `--nb-mus kN`, and inserting multipliers never loses a label (§7.4), so 10 → 20 → 40 never loses one; the test checks 10 ⊆ 20. 9 is not inside 20: on quebec B=10000, `--nb-mus 9` finds more labels than 20 (134,671 / 430,863 strong / useless against 134,637 / 430,802) and builds 95 fewer variables [S3 gate]. "20, or 9 on large instances" in the help text are two recommendations, not an ordered pair.

### 2.4 What it costs

- **Labelling** at 4 threads, gecot's own preprocessing time [S2 timing]: aix70 B=2 dec9 12.3 s, dec10 13.2 s, dec20 21.5 s (rule, without frontier: 29.3 s); quebec B=2133 dec9 118 ms, dec20 182 ms; bike_mid B=155 dec9 22 ms, dec20 30 ms. The μ = 0 pass alone takes 3.3 s on aix70 (osweep).
- **Where it gains nothing** the cost is pure overhead. aix70 B=10, osweep labelling at 4 threads: μ = 0 alone 3.3 s; dec9 15.7 s (4.8x); dec10 16.7 s (5.1x); dec20 27.5 s (8.4x); the old cur:10 grid 40.1 s (12.2x) [S2 timing]. On quebec and bike_mid at 10% of the total, the 3-decade grid costs 9x and 3.2x a single μ = 0 pass with frontier and redbreak (11.6-12.7x without) [S2 sweep quebec/bike]. Before frontier, end to end at aix70 B=10, nm10 and rule took 1.07-1.09x the time of nm0 [S1 timing].
- Nothing in the code detects a budget where no multiplier can gain. A per-arc slope-lemma skip (§1.3) would be the exact saver; it was not tested on real instances [S2 judge, S2 critic].

### 2.5 Proposed text for the user documentation

The block below is a proposal; the owner has not reviewed it. Its first paragraph rests on §1.3 (labels are only added, and stay valid) and on the timed runs: no Lagrangian variant built more variables than `--nb-mus 0` in any of the 21 cells of [S1 timing] that have both an `--nb-mus 0` and a Lagrangian run (the 22nd, biorevaix_N2 at 0.5%, has only `--nb-mus 0`, §9.1) or the 4 cells of [S2 timing] that have an `--nb-mus 0` run (their CSVs, checked for this note), and every variant reached the same optimum at every point that finished [S1 timing]. That the model can never grow is an inference, not a tested property.

> **Constrained preprocessing (experimental).** `flow_mip` and `tree_lp` can take the budget into account in their preprocessing. With `--nb-mus N` (N > 0) gecot tests N + 1 Lagrange multipliers besides the usual pass and marks more arcs as always or never on a best path for every option set that fits the budget. It only adds to what `--nb-mus 0` finds, so the model is usually smaller (it was never larger in our tests), and the optimum is unchanged.
>
> - It helps only when the budget is small in absolute terms, so that few options fit together; the fraction of the total option cost is not a reliable guide. On the instances tested its gain vanished from 2-20% of the total option cost (2% on the largest), or, on the grid instances, from a budget of about 6-10 cost units whatever the fraction (at 10% of the total, the smallest grid instance still lost 10% of its contracted arcs). Where it gains nothing it only adds preprocessing time, and where it gains little, or the solve is short, the extra preprocessing can outweigh the smaller model. Compare with `--nb-mus 0` on your instance before relying on it.
> - Use `--nb-mus 20` when one labelling pass takes well under half a second, and `--nb-mus 9` when a pass takes about half a second or more (on a 70x70 grid instance, about 1 s per multiplier).
> - `--mus-sweep decades` (the default) tests the multipliers over at most the three decades below the largest useful one, then 0 again. `--mus-sweep full` starts from the inverse of the largest option cost per arc, a one-unit penalty that never added a label in our tests. Neither has any effect without `--nb-mus`.
> - Doubling `--nb-mus` (10, 20, 40) never removes a label; other changes, such as 9 to 20, can.
> - If one option improves an arc far more per unit of cost than any other (for example a cheap option on a near-zero-probability arc), try `--mus-sweep full`: that option sets the top of the decades window.

## 3. Review against the thesis, and the code defects [R]

### 3.1 Conformance

- **μ = 0 is exactly Algorithms strong and useless:** 0 differences on quebec, aix30 and aix40.
- **T** starts from F(u,v) and grows from pass to pass, as line 508 prescribes; line 508 fixes no order of μ.
- **The λ^μ generalisation** to options covering several arcs and to several options per arc is valid, since a simple path holds at most |A_i| arcs of option i and the floored shares sum to at most ⌊μβ⌋. min_mu is a sound addition.
- **T handling** and **source u** in the useless loop: see §1.4.
- **One real deviation: the zero-length-cycle guard on the strong ties** (§1.4, §5.4). [R] reviewed red ties, the deviation at the time.

### 3.2 Defects A1-A7 and their fixes

| # | Defect at 6c59214 | Effect | Fix and evidence |
|---|---|---|---|
| A1 | `for(mu = min_mu; mu <= max_mu; mu *= ratio)` drops max_mu by rounding: on Aix at N = 5, 8, 10, 11, 30, 40; over random ranges at N=10, 82-99% of the time | Strength only; `--nb-mus` was non-monotone (N=10 beaten by N=1 on Aix) | Grid by index, last point exactly max_mu (df9a8581). Patched labels contain HEAD's everywhere; strictly more on Aix at num_mus 5 and 10, e.g. aix40 nm10 strong 164,653 → 165,700 and useless 184,927 → 187,663, aix70 nm5 +19,999 / +49,851. Every extra label comes from the grid [S1 verify]. |
| A2 | Endless grid loop when the ratio rounds to 1, or is inf (cost-0 option) | Crash, never a wrong result; easy at a coarse `--resolution` | Bounded loop, inf capped. The 9 formerly endless fuzz seeds pass; the formerly endless CLI instance gives the exhaustive optimum 4.9985000014985 at `--nb-mus` 0, 1, 3, 10 [S1 verify]. |
| A3 | Duplicate heap seeds (v in the strong loop; u, and v red then blue, in the useless loop) break melon's PRE_HEAP precondition | Debug aborts; Release provably harmless | `w != v`, `w != u` and the useless-loop stop. 30,000 instances with assertions on, 0 aborts; HEAD's header aborts in the same binary [S1 verify]. At df9a8581 the stop was required; since frontier it is only an early exit (§4.3, p4). |
| A4 | μ·B ≥ 2^64 makes the uint64 penalty 0 | Invalid labels and a wrong optimum, reproduced with the CLI | Cap 2^62/max(B, 1) at df9a8581, then 2^62/max(B, max s_o, 1) (§10.4). The CLI instance (costs 0.001, 1e6, 1e9; B=1e9) gave 0.75 {o2} at `--nb-mus 10` before and gives 0.9 {o1}, the exhaustive optimum, after [S1 verify]. |
| A5 | Options without arcs in the case: c/0 = inf, then NaN or inf cast | UB; the value is unused | Such options are skipped. UBSan on quebec B=1000: 0 reports; the old header hits the NaN cast there [S1 verify]. |
| A6 | min_mu is a one-unit penalty: 5-7 of the 10 positive points add nothing on the instances counted (5 on aix70 B=2, 7 on aix30 B=4, 6 on quebec) | Time only | The decades sweep (§6). |
| A7 | No test on an affordable scenario: fuzzy_test checks the all-options scenario at total/10, where the multipliers gain nothing | Coverage | `preprocessing.constrained_labels` (§4.5). |

- **A4 at the default resolution** [S1 verify]: at 1e-8 and 1e-12 the cap never binds on any of the 11 instances (max_mu < 2^62/max(B, 1) even at B = total cost), and the cap comment's "2^62 outweighs any path" and "no uint64 sum wraps" are proven there by S_{n−1}, the sum of the |V|−1 largest base lengths: at most 1.0e17 at 1e-12, with P + S_n at most 2e17 (at 1e-8 even the crude bound (|V|−1)·max length is at most 3.2e14). At 1e-15 neither is proven (§11).
- [R] measured the A1 damage end to end at 6c59214, aix70 B=2, `--big-m tight`: `--nb-mus 0` 32.2 s; `--nb-mus 1` (grid {0, min_mu, max_mu}) 13.3 s; `--nb-mus 10` (max_mu dropped) 33.1 s, all at the same optimum. The shipped grid got about 31% of the reduction that the same grid with max_mu added gives: Σ G_t arcs 399,223 against 359,748 (cur:10), unconstrained 416,593; max_mu alone (cur:1) gives 361,155 [S1 sweep].
- At 6c59214, B=5 and B=10 on aix70: `--nb-mus 1` 12% slower at B=5 with a 1.1% smaller model; at B=10 the same model and 2.9 s slower (critic runs, 2 repeats each, not rerun by [R]).

### 3.3 Thesis text errors

None reached the code [R §C].

1. **algo:beta_useless** (line 533) and its proposition (566-567) take T ⊆ 𝔽^β; it must be T ⊆ 𝕎^β. Read literally the proposition is false. W_{μ,T} is never defined and the proposition has no proof; the file ends at its `\end{proposition}`. "T ⊆ W_{μ,T}" also fails when v ∈ T, and for a T not closed under blue predecessors. Minimal fix: return T ∪ {blue vertices}.
2. **algo:beta_strong.** The initialisation (477-480) colours every out-neighbour of u red with no w ∉ T check, so "T ⊆ F_{μ,T}" (504-506) and "at least T" (508) are false for the printed algorithm, though its outputs stay valid. H (line 363) bars both colours from T, the algorithm only red, so "amounts to running Algorithm strong on H" (466) is inaccurate.
3. **Line 508** cites algo:strong; it means algo:beta_strong.
4. **Positive lengths.** The general part assumes them; the constrained section drops the assumption (CSPP box at 324 has l_a ≥ 0; lines 348, 505, 566 say only l^- ≤ l^+). The algorithms need "no zero-length cycle in any feasible scenario", and the thesis applies them to −log π, which is 0 at π = 1.
5. **Proof of lemma F_mu_T_in_F_beta** (434-457): a quantifier slip, '=' where '≤' is meant, a (distance, hops) tie-break needed with zero lengths, and the "trivial" T ⊆ F_{μ,T} fails if u ∈ T (only through a zero-length cycle).
6. **μ range** (513-517) divides by c_a, which line 324 allows to be 0. It should also state that max_mu is included and dominates every larger μ.
7. **Parallel arcs.** The initialisation loops assume a simple digraph; gecot's SCC contraction creates parallel arcs. The code is immune: it has no initialisation loop.

### 3.4 The M3 refinements

| M3 element | In this preprocessing | Gain if ported |
|---|---|---|
| Shares c_j/k_j, k_j = min(\|T_j\|, \|H_j\|) | Uses c_j/\|A_j\|; porting is valid in both loops | 0: \|A_j\| = k_j for every option of every instance |
| Grid θ/β, θ ∈ {0, ½, 1, 2, 4, 8} | Does not transfer: λ saturates at l^+. The useful window ends at θ_max: 0.033 on bike_small, 0.34 on aix70 B=2, 227 on quebec | Negative or nil |
| Budget tolerance τ | Not needed: contraction and removal can only lower F_t for scenarios over budget | n/a |
| One multiplier per whole path | Already in place: the blue seed carries ⌊μβ⌋ once | n/a |
| Per-copy (option-conditioned) uselessness | Missing; valid with a tolerance | Structurally 0 on Aix, quebec, aude; 6 copies on bike_small, 4 of 6,486 on bike_mid B=200 |
| Source filter on base arcs | Missing; valid | 0.04-2.5% of base arcs |
| Tight big-Ms after `--nb-mus ≥ 1` | Consistent: M3 works on the G_t it receives | n/a |

Side finding: G_t is never pruned backward from t (`compute_contracted_generalized_flow_graph.hpp:117-134` prunes forward only). On bike_small at B=100, 856 copies and 4,360 base arcs of 6,587 G_t arcs cannot reach t.

### 3.5 Numerics

Shared with `--nb-mus 0` [R §E]. The clamp p < r and the per-arc floor make labels exact for the integer lengths only. The per-pair loss is relative, about P*·(h−1)·r for an h-hop path: 4.5e-8 at h=10, 5e-6 at h=1000. It is not bounded by the resolution in absolute terms, so "exact up to --resolution" (`src/commands/registry.cpp:53`) overclaims. quebec target 206 shows 5.46e-6 relative with `--nb-mus 0` as well.

### 3.6 Owner's question 3: do the multipliers keep the lengths positive?

No.
- Integer lengths are uint64, so never negative, but zero lengths exist. An arc with p within about r of 1 has base length 0 (p > 1/(1 + r): from §1.1, floor(−ln p / log1p(r)) = 0 iff −ln p < log1p(r), up to rounding at the boundary; derived for this note, no source states it); an option that raises p to 1 has l^o = 0, μ = 0 always runs, and small penalties floor to 0. Blue uses the base lengths.
- Measured [S1]: after reformulation Aix has 2,586 / 4,112 / 13,964 zero-length arcs (aix30 / 40 / 70), the same count under λ^0. quebec has none under base lengths but 51-80 under λ^0, growing with the budget. aix_N2 and biorevaix_N2 have 51,446 and 51,118. aude and the bikes have none.
- Zero-length cycles: 0 nontrivial zero-length SCCs and 0 zero-length self-loops at every point tested (32 Aix, 36 quebec / bike / aude, every N2 budget), under base lengths and λ^0 [S1]. Raw aix30 has 10, raw aix70 76.
- Only `trivial_reformulate`'s SCC contraction removes zero cycles, and only SCCs of exact p == 1 base arcs [R]. The code stays valid with zero cycles thanks to the guard, whose arcs keep red ties (§5.4).

## 4. Validity testing

### 4.1 The brute-force fuzz harness

- **What it does.** It compiles the real header, generates small random instances (6-12 vertices in [S2 theory]), and checks every recorded label against every feasible option set:
  - integer-length check: each strong / useless label must hold under the integer lengths of every feasible scenario (counters Sv_int, Uv_int);
  - end-to-end check: F_t computed on the original graph against F_t on the contracted G_t, per target and scenario (e2eViol; the largest relative deviation is reported);
  - superset: the constrained labels contain the unconstrained ones (notSup);
  - nesting: the labels of N within those of 2N [S3].
- **Modes.** `ref` (through `trivial_reformulate_instance`, as the CLI) and `raw`. Generators: `std`; `tight` (every option fits alone, at most one or two together) and `tight2` (denser, more one- or two-arc options, weaker base arcs), added in [S3] because `std` gains in only about 1.2% of instances. Extra modes: budgets ×1e12 and ×1e17 where the cap binds; resolution 0.3 and 0.5; vertex-only options; one option 1e10 times cheaper; `huge15` (about 30% of options 1e15 times dearer, always-true predicate) [S1, S3, S4].
- **Variants** are generated from the live header by a script that asserts an exact count for every replacement, so each control is one deliberate edit [S1, S3].
- **Trace check** [S3 fuzz]: every 10th instance also runs D10 and F10 at trace level, and the logged "µ sequence" line is compared with the grid the harness replicates: 0 mismatches in 4,000+ trace runs; 400 more (Release) and 200 (assertion build) on fresh seeds [S3 critic]; 0 mismatches on the final header [S4].
- **Real-number "violations"** (Sv_re, Uv_re) equal the unconstrained baseline in every run: they come from floating-point lengths, not from the multipliers [S1 verify, S3 fuzz].

### 4.2 Counts per phase

Every brute-force row found 0 invalid labels, 0 end-to-end violations and 0 losses against the unconstrained labels. The "fix 2" row is a label-identity and assertion check, not a validity run.

| Phase | Header | Instances | Configurations | Notes |
|---|---|---|---|---|
| [R] | 6c59214 | four harnesses: ~20k ref + 10k raw (6.4M end-to-end checks); ~45k; 14k; 5.3k | various | End-to-end mismatches ≤ 1.5e-9 relative |
| [S1 verify] | df9a8581 | 20,000 ref + 10,000 raw; 15,000 at budgets ×1e12 / ×1e17 | num_mus 1, 3, 10, 200 | Largest end-to-end deviation 1.46e-9 (ref), 1.23e-9 (raw) |
| [S1 verify] UBSan | df9a8581 | 75,000 fuzz instances, 300,000 calls (ref + raw 20,000 each at 1e-8; 5,000 + 5,000 vertex-only; 5,000 + 5,000 at resolution 0.3; 5,000 ref at 0.5; 5,000 + 5,000 with one option 1e10 times cheaper) + 66 library calls on 23 crafted cases | num_mus 1, 3, 10, 200 | 0 reports in scope; out-of-scope library cases hit the cost-0 divisions, 1.0/num_mus at num_mus 0 and the per-arc cast of case h1 (§10.5). 841 formerly endless seeds and 2,168 former μ·B overflows included |
| [S1 verify] fix 2 | df9a8581 | 80,000 Release calls (4,000 seeds) + 30,000 Debug calls | num_mus 1, 2, 3, 10, 200 | Removing fix 2 changes no label |
| [S2 theory] | df9a8581 pass code | 20,000 ref + 10,000 raw | 156 order variants (4 grids × 39 orders and rules) + header nm3, nm10 | 0 identity breaks, 0 labels outside the fixpoint, 0 canonical-distance mismatches |
| [S3 fuzz] | 80bb8a4e | 158,000 instance-runs; the six main release runs are ref/raw × std 20,000, tight2 10,000, tight 5,000 | decades and full × num_mus 1, 2, 9, 10, 20, 40 | Plus 3,000 + 3,000 big-budget runs |
| [S3 critic] | 80bb8a4e | 2,000 fresh ref tight2 seeds (Release), 1,000 raw std (assertions) | the same 12 | 201,059 end-to-end checks |
| [S4] | 26de0dac | 38,000 instance-runs incl. always-true predicate and `huge15` | the same 12 | 192,000 label-set pairs (26,703,793 labels) identical to 80bb8a4e under `cost <= budget` |

The commit message's "about 200k small random instances" is approximate, and no source breaks it down; read as a sum, it adds the instance-runs of [S3 fuzz], [S3 critic] and [S4], not distinct instances ([S3 fuzz]'s six main runs are 70,000 instances, each under the 12 configurations), on two header states tied together by [S4]'s identity check.

- **Decades against full** [S3 fuzz], instances where decades has more / fewer labels than full at the same num_mus: ref std 72 / 0 (nm1), 73 / 18 (nm9), 41 / 14 (nm20) of 20,000; ref tight2 591 / 1, 1,299 / 98, 834 / 90 of 10,000; ref tight 180 / 0, 226 / 19, 138 / 19 of 5,000; raw std and raw tight2 within 4 instances of ref. Decades has fewer labels only when raising min_mu to max_mu/1000 removes points; in the 488 std instances where it removes none, the two sweeps give identical labels, as the supersequence argument (§7.4) predicts.

### 4.3 Positive controls

Mutations that must be caught, or must be shown harmless:

| Control | Result |
|---|---|
| No blue penalty, strong loop | Caught: 1.36M violations [R]; 1,614,413 (ref) and 784,514 (raw), first bad seed 2 [S1 verify] |
| No blue penalty, useless loop | Caught: 0.88M [R]; 1,027,003 and 509,552 [S1 verify] |
| Shares doubled | Caught: 6.3k [R]; 2,186 strong + 4,066 useless (ref), first bad seed 6 [S1 verify]; not caught at huge budgets, where every scenario is affordable, as expected |
| Fiber seeds without the penalty | Caught: 99,787 (cur10, asc) and 204,049 (top3p10, desc0first) on ref [S2 theory]; as p1 on 80bb8a4e, Sv 26,661 / Uv 155,289 / e2e 79,315 on ref std at D10, first bad seed 2 [S3 fuzz]; each loop caught on its own; on fresh seeds 500001-502000 (ref tight2, D10), 4,775 invalid strong / 40,903 invalid useless labels [S3 critic] |
| p2: every fiber seed dropped | Valid, lossy: loses 42 strong / 170 useless labels in 118 of 20,000 ref std instances [S3 fuzz]; 56 / 424 on the fresh seeds [S3 critic] |
| p3: explicit seed pushed after the fiber seeds | Valid, lossy: 15 / 48 labels lost in 27 instances (ref std), 98 / 403 in 319 (ref tight2) [S3 fuzz]; 18 / 103 on the fresh seeds [S3 critic] |
| p3r: explicit seed first, fiber seeds reversed | Identical labels [S3 fuzz]; also on the fresh seeds [S3 critic] |
| p4: no useless-loop stop | With frontier: 0 aborts in 12,000 assertion-build instances, labels identical. p4nf (also without frontier) aborts at PRE_HEAP in 2,179 of 3,000 ref std and 2,635 of 3,000 ref tight2 [S3 fuzz] |
| HEAD's header (6c59214), assertions on | Aborts at `competing_dijkstras.hpp:184` (PRE_HEAP) [S1 verify, S2 theory] |
| HEAD's header on the 23 crafted library cases (66 calls) | bad_alloc on b1, b2, c1-c3 (num_mus 3 and 10), d1, d2, e2 and f1-f3; invalid labels on g1 and g2 ("0->1 strong, 0->2 useless" for t=1); PRE_HEAP abort with assertions on (v1, g1); NaN and overflow casts. The patched header passes all 66 [S1 verify] |
| df9a8581 with fix 2 reverted, Debug | Aborts at PRE_HEAP within seeds 1-50, so the fuzz reaches the re-seed paths [S1 verify]. Without the useless-loop stop alone it also aborts (§3.2, A3); with frontier it no longer does (p4 below) |
| Old header on the cheap-option fuzz | 7,323 invalid strong + 8,923 invalid useless labels on 2,000 instances [S1 verify] |
| Header without the cap, big budgets, UBSan | Aborts at seed 1, "2.50142e+19 is outside the range" [S3 fuzz] |
| Old cap (2^62/max(B,1)) on the crafted probe of §10.4 | UBSan: "2.30584e+19 is outside the range" [S4] |

### 4.4 Assertion and sanitizer builds

- melon's PRE_HEAP asserts on (`-UNDEBUG`): 30,000 instances at df9a8581, and every UBSan fuzz build and the 66 crafted library calls, which also had asserts on [S1 verify]; 3,000 + 2,000 brute-force and 30,000 comparison-mode instances over every order variant [S2 theory]; 5,000 per ref/raw × std/tight2, 2,000 per ref/raw tight, and the controls, at 80bb8a4e [S3 fuzz]; 1,000 fresh raw std seeds [S3 critic]. Real instances: aix30 B=1 and quebec B=2133, 33 osweep specs each (orders, cycles, stops, savers) [S2 build]; 5 cells × 10 specs, hash- and counter-identical to Release, and 45 of 45 hash-equal against the real melon class, REC included [S2 skeptic]; aix30 B=1 and aix40 B=1.42 [S1 critic]; quebec, aix30, bike_small and the `test` instance [S3 review]. No abort in any of these, and labels equal to Release.
- **The exception: self-loops.** An arc with u = v still seeds u twice and trips PRE_HEAP in Debug, in this header and in the unconstrained one; raw bike_mid has 10 self-loops, so a library caller on raw data aborts in Debug. The CLI never sees them: reformulation drops intra-component arcs [S1 verify, S1 critic] (§10.5).
- UBSan with float-cast-overflow and float-divide-by-zero (GCC's `-fsanitize=undefined` does not include them): 75,000 fuzz instances and quebec B=1000 [S1 verify]; 1,000 per ref/raw × std/tight2 mode plus 1,000 per big-budget mode [S3 fuzz] (its 1,500-per-mode runs used plain `-fsanitize=undefined`); 16 modes × 1,000 [S4]. 0 reports in scope.
- Never run: ASan, TSan, a Debug build of gecot itself and `gecot_test` with assertions on [S1 critic, S3 critic]. The trace block's divisions never ran under UBSan [S1 verify].

### 4.5 The ctest regression test

`preprocessing.constrained_labels` (test/preprocessing_test.cpp) [S3 testBuild]:

- **Setup.** The raw committed quebec instance through `parse_instance`, B=2133, predicate `cost <= budget`, resolution 1e-8, decades sweep at num_mus 10 and 20. B=2133 has the largest gain of the budgets scanned on the raw instance:

| B | unconstrained | decades N=10 | decades N=20 | full N=20 | gain at N=20 |
|---|---|---|---|---|---|
| 500 | 200,660 / 539,281 | 201,310 / 540,180 | 201,334 / 540,221 | 201,314 / 540,155 | 1,614 |
| 1000 | 170,467 / 499,773 | 178,159 / 510,278 | 178,678 / 510,981 | 177,837 / 510,298 | 19,419 |
| **2133** | 149,347 / 464,878 | 159,368 / 482,575 | **160,083 / 483,907** | 157,875 / 480,331 | **29,765** |
| 4000 | 139,226 / 442,160 | 142,938 / 450,684 | 143,209 / 451,207 | 142,739 / 449,902 | 13,030 |
| 10000 | 134,617 / 430,853 | 135,128 / 432,461 | 135,197 / 432,682 | 134,929 / 431,970 | 2,409 |
| 20000 | 132,910 / 424,838 | 132,953 / 425,291 | 132,988 / 425,341 | 132,980 / 425,225 | 581 |
| 42665 (total/10, fuzzy_test's budget) | 132,910 / 424,838 | same | same | same | 0 |

- The table is red ties. With blue ties the pin is 160,562 / 483,907 (§5.4); the strong count moves, the useless count does not.
- **Asserts.** (a) the unconstrained labels are a subset of N=20's, strong and useless separately; (b) N=10's labels are a subset of N=20's; (c) validity: on the empty solution and 4 random feasible solutions (`std::mt19937(0)`), t's contribution on the generalized flow graph equals that on G_t within 1e-6 relative; (d) strict gain; (e) pins: strong 160,083, useless 483,907. The gain is 29,765 labels, 4.8% of the unconstrained 614,225 (strong +10,736, useless +19,029). The label lists are identical at 1, 2, 7 and 16 threads. Runtime 0.31 s.
- **The pins are raw-instance numbers.** The CLI reformulates first: at the same budget, decades N=20 gives 153,904 / 465,153 on the reformulated instance [S3 review].
- **Helper.** The old unused helper `compute_original_contribution` compiled against none of the tuples; it was replaced in place by `compute_solution_contribution`, which matches `gecot::compute_solution_cases_pc_num` to 1.3e-16 relative. The random solutions change 111-356 targets' contributions.
- **Mutations** run on a scratch copy:

| Mutation | Result | Assertions that fire |
|---|---|---|
| m1: positive points skipped | Caught | (d), (e): exactly the unconstrained labels |
| m2: final μ = 0 dropped | Caught | (e) only: strong 159,866 (−217), useless 483,623 (−284), so the final 0 adds 501 labels here |
| m3: frontier erases every fiber seed | Caught | (e) only: strong 149,472, useless 464,878 |
| m4: fiber seeds without the penalty | Caught | (b) 876 lists not nested, (c) 378 invalid pairs, (e); in an exploratory run (c) caught it with the empty solution alone (73 invalid pairs) |
| m5: old grid formula min_mu·pow(ratio, k) | **Not caught** | none: same totals, still nested at B=2133 |

- **Limits.** The test cannot pin the grid formula (m5): only the fuzz harness and osweep check bitwise nesting [S3 critic]. Its 1e-6 tolerance sits 2.4x above the largest deviation at B=2133 (4.2e-7), but at B=10000 the unconstrained labels alone reach 5.46e-6 (1 of 21,238 checks), so moving the budget, as the pin comment invites, can fail (c) with no Lagrangian defect [S3 review]. fuzzy_test, at total/10, cannot catch any regression of the multipliers.
- ctest: 25/25 passed after the change [S3 testBuild, S3 critic].

### 4.6 The harness gate: the real binary against the harness

- **Rule.** gecot's flow_mip variable count = the harness's predicted columns (`pred_vars`) + the option count of the *reformulated* instance at the cell's budget: 346 not 358 on bike_mid B=155 (322 at B=110), 207 not 260 on quebec B=2133 (167 at B=1000, 255 at B=10000), 17 not 18 on aix30 B=1.72 (16 at B=1) [S3 gate].
- **[S2 design]:** 6 runs (aix70 B=2, bike_mid B=155, 3 sweeps each): the "µ sequence" line equals osweep's sequence character for character, every trace row matches to 3 decimals, variable counts equal. The scratch build with frontier, run as `--mus-sweep full --nb-mus 10`, reproduces the real build without frontier on aix70 B=2 (369,264 variables, 191,337 constraints, identical solution).
- **[S2 timing]:** variable counts equal in all 28 (cell, variant) pairs. **[S2 critic]:** 6 runs on two unseen cells (bike_small B=62, quebec B=4267).
- **[S3 gate], on 80bb8a4e:** 49 gecot runs on 13 cells, aix_N2 included. Min µ, every trace row and the variable count match the prediction on all 49 runs, and the µ sequence on the 37 runs that print one (the 12 df9a8581 runs do not), 0 mismatches; in-process, harness and real header are hash-equal on all 49 configurations. Variables:

| Cell | `--nb-mus 20` | `--nb-mus 9` | `full --nb-mus 10` | df9a8581 `--nb-mus 10` |
|---|---|---|---|---|
| bike_small B=31 | 5,912 | 5,923 | 5,933 | 5,933 |
| bike_small B=62 | 6,544 | 6,556 | 6,640 | 6,640 |
| bike_mid B=110 | 14,169 | 14,578 | 15,334 | 15,334 |
| bike_mid B=155 | 15,428 | 15,794 | 17,911 | 17,911 |
| aix30 B=1 | 18,843 | 18,875 | 18,893 | 18,893 |
| aix30 B=1.72 | 27,362 | 27,365 | 27,563 | 27,563 |
| aix40 B=2 | 57,034 | 57,037 | 57,164 | 57,164 |
| aix40 B=2.84 | 60,739 | 60,782 | 60,873 | 60,873 |
| quebec B=1000 | 134,344 | 135,452 | 137,071 | 137,071 |
| quebec B=2133 | 195,029 | 197,633 | 206,590 | 206,590 |
| quebec B=10000 | 294,485 | 294,390 | 295,625 | 295,625 |
| aix70 B=2 | 368,593 | 368,629 | 369,264 | 369,264 |
| aix_N2 B=53.72 | – | 8,991,987 | – | – |

- `full --nb-mus 10` builds the same variables and constraints as the df9a8581 binary on all 12 cells. Both reach optimality with identical solutions on six of them: bike_small B=31 and B=62, bike_mid B=155, aix30 B=1, aix70 B=2, and aix30 B=1.72 on a rerun without the time limit. The other cells stop at the time limit, where equal incumbents prove nothing [S3 gate].
- aix_N2 B=53.72, whole graph, `--nb-mus 9`: 8,991,987 variables, 4,533,378 constraints; preprocessing 127 s, 930 s wall to the 1 s solver limit (the contracted graphs, tight big-Ms and model build dominate), peak RSS 2.3 GB [S3 gate].
- **[S3 critic], on 80bb8a4e:** gate cell bike_mid B=110 rerun from scratch with HiGHS instead of Gurobi: the same three variable counts (14,169 / 14,578 / 15,334), 0 VERIFY mismatches; µ sequences of 23, 12 and 12 entries, only the two decades runs ending on a second 0.

## 5. Where the multipliers gain [S1]

### 5.1 Budget sweeps per family

Contracted arcs removed over unc, cur:10 / dense:40; "–" means not run. On Aix, strong and useless labels grow by only 1-6%, but G_t shrinks 4-10 times more in relative terms; elsewhere the ratio is smaller (quebec B=1000: labels +2-4% for −10.3% of arcs; aix_N2 at 0.5%: useless +5.75% for −6.74%).

| Instance | Budget (fraction) | cur:10 | dense:40 |
|---|---|---|---|
| aix30 | 1 (5.8%) / 1.72 (10%) / 3 / 3.44 (20%) / 4 / 8, 8.6 | −22.8 / −9.4 / −2.15 / −0.94 / −0.26 / 0% | −23.0 / −10.1 / −2.33 / −1.10 / −0.29 / 0% |
| aix40 | 1.42 (5%) / 2 (7%) / 2.84 (10%) / 4 / 5.68, 14.2 | −11.6 / −8.95 / −2.74 / −0.21 / 0% | −11.8 / −9.15 / −2.89 / −0.22 / 0% |
| aix70 | 1 (1.6%) / 1.228 (2%) / 2 (3.3%) / 3 / 3.07 (5%) / 5 / 6.14 (10%) / 10 / 12.28, 30.7 | −24.8 / −18.6 / −13.65 / −6.75 / −6.31 / −1.15 / −0.29 / 0 / 0% | −24.9 / −18.7 / −13.74 / −6.81 / – / −1.16 / −0.31 / 0 / – |
| quebec | 1000 (0.23%) / 2133 (0.5%) / 4267 (1%) / 8533 (2%) / 10000 / 20000 / 21333 (5%) / ≥ 42665 | −10.26 / −7.26 / −2.24 / −0.77 / −0.42 / −0.14 / −0.05 / 0% | −11.65 / −11.31 / −3.73 / −1.42 / −0.82 / −0.17 / −0.06 / 0% |
| bike_mid | 78 (0.5%) / 155 (1%) / 200 / 311 (2%) / 777 (5%) / ≥ 1554 | −16.62 / −6.09 / −4.28 / −3.88 / −0.08 / 0% | −21.30 / −17.69 / −7.17 / −5.31 / −0.34 / 0% |
| bike_small | 31 / 62 / 100 / 124 / 310 / 620 / ≥ 1241 | −1.87 / −0.80 / −0.38 / −0.51 / −1.21 / −0.11 / 0% | −2.24 / −2.10 / −1.55 / −1.17 / −1.43 / −0.23 / 0% |
| aix_N2 | 0.5% / 1% / 2% / 50% | −6.74% (top:2:5 −7.16%) / theta grid −1.08% / −0.0026% / 0 | not run |
| biorevaix_N2 | 0.5% / 1% / 2% / 50% | −2.48% (top:2:5 −3.75%) / theta grid −0.065% / 0 / 0 | not run |
| aude | 1.5, 3, 5, 7.5 | 0 for every grid, dense:40 and every singleton included | 0 |

- Aix gains nothing where no option fits, nor where only the cheap 0.2-0.4-cost options fit (aix30 0.344 and 0.86, aix40 0.568, aix70 0.307); aix70 0.614 is effectively nil (3 arcs, +6 labels).
- On N2 at 0.5%, options costing more than B are dropped, so unc itself is smaller (aix 9,663,091 gt_arcs against 10,142,146 from 0.93% on); the relative gains there are against that smaller baseline. 5%, 10% and 20% were not run on N2.
- **The landed grids on the whole aix_N2 graph at 0.5%** [S2 sweep N2]: top:3:10 removes 7.22% and top:3:21 7.38% of gt_arcs (8,965,058 and 8,950,131 against unc's 9,663,091, the same baseline as [S1]'s; the agent's "−713,960, −7.39%" is an arithmetic slip for −712,960), so top:3:10, the positive points of `--nb-mus 9`, slightly beats top:2:5's −7.16% even without the final 0. On a seed-2 sample at aix_N2 B=80, REC gets +4.87% of top:3:10's label gain [S2 skeptic].
- **The landed grids on quebec** [S2 judge], 0 / ascending / 0, against the same unc baselines as [S1]'s (151,331 at B=1000, 221,314 at B=2133): `--nb-mus 20` removes 12.06% at B=1000 and 12.49% at B=2133 (133,082 and 193,675 gt_arcs), `--nb-mus 9` 11.33% and 11.31%; the 21-point fixpoint reaches 12.52% at B=2133. `--nb-mus 20` thus beats [S1]'s dense:40 (11.65% and 11.31%). These runs used the old grid formula, whose labels equal the landed one's on both cells (§6.4).
- On quebec, unc's gt_arcs grows with B (151,331 → 306,608) and is constant from B=20000, once every option is individually affordable.

### 5.2 Where the useful multiplier sits

max_mu does not depend on the budget on any instance once the options that set it are affordable, so "a fixed fraction of max_mu" means a fixed μ and "a fixed θ" means μ ∝ 1/B; singleton runs ({0, μ} against {0}) tell them apart.

| Family | max_mu, θ_max | Useful multipliers | What sets max_mu |
|---|---|---|---|
| Aix | 1.7185e7 whenever a cost-1 option fits (7.595e6 when only the 0.2-0.4-cost options fit); θ_max = 0.172·B nats | The top decade. The peak is at max_mu while θ_max ≤ ~0.35 nats, then, on aix30 and aix40, 1-2 grid steps below (0.45-0.67·max_mu); on aix70 it stays at max_mu up to B=6.14 (θ_max 1.06). The ≥ 50%-of-peak window always contains [0.45, 1]·max_mu | Ordinary cost-1 two-arc options, p 0.894 → 0.975 (per-arc cost 0.5); no arc has p < 1e-8 |
| bike | 33,333 on both crops; θ_max 0.01-2.6 nats | max_mu/2 at every budget, the saturation multiplier of most (option, arc) pairs (saturation θ p10 = p50 = θ_max/2); θ_peak grows with B; at max_mu the gain drops, to 0 at bike_small B=620 and bike_mid B=777 | No clamped pair |
| quebec | 22,741,581.3 at all 11 budgets; θ_max from 227 (B=1000) to 48,514 (B=213,327) | θ ≈ 21-25 nats at every budget with a gain (21.0, 22.5, 21.9, 20.9, 24.5, 23.0, 24.5 at B=1000 … 21333), while μ/max_mu falls from 9.2e-2 to 5.1e-3; the singleton grid's step is ×1.8, and [S2]'s finer scan (25 points from 2 to 60 nats, B=2133 and 8533) puts the broad peak at 17-22 nats, where one multiplier alone gives 86-90% of the grid's gain; nothing at max_mu | The clamp: 100% of the improving pairs have base p < 1e-8 |
| N2 | aix_N2 at 0.5%: 2.28e8; θ_max = 2.28·B (aix), 2.02·B (biorev) nats | θ ≈ 1-30 nats; peaks at θ=10 (aix, μ/max_mu 0.082) and θ=3 (biorev); on aix the peak stays at θ=10 when B doubles; max_mu alone gains 0 | Cheap options at cost 1 per arc (aix option 1224, cost 11 over 11 arcs, p 0.1 → 0.98); no clamp |
| aude | θ_max 54-272 nats | none | All 30 improving pairs clamped |

- quebec's window does not follow the clamp: at B=1000, θ_max is 170 / 227 / 284 at resolution 1e-6 / 1e-8 / 1e-10, while the peak stays at 16.0 / 21.0 / 16.2 nats (and 21.8 / 20.9 / 24.2 at B=8533). A different resolution also changes the integer lengths and the base counts, so this probe locates the window only [S1].
- Neither "fixed θ" nor "fixed fraction of max_mu" holds across families: aix70 behaves as a fixed fraction (label windows at 3 budgets), aix40 and N2 as a fixed θ (N2: aix_N2 at two budgets only), quebec wants θ ≈ 10-40, bike wants μ = max_mu/2 [S1]. That is why the grid spans decades (§6.2).
- On N2 the top decade is pure cost: p9 and p10 of top:3:10 (θ 57 and 123) take 12.3% and 15.4% of the cost on aix_N2 at 0.5% and add 0 labels [S2 sweep N2].
- **One multiplier does most of the work on N2.** One budget-scaled θ captures 94.6% (aix_N2, θ=10) and 86.6% (biorevaix_N2, θ=3) of top:2:5's gt_arcs gain, at about a quarter of top:2:5's labelling time [S1 sweeps]. In [S2]'s {0, p_k} singletons the useful θ is 5-12 nats: p7 (θ 12.3) alone gives 96.2% of asc's label gain on aix_N2 at 0.16 of its cost, p6 (θ 5.1) 89.5% on biorevaix_N2 [S2 sweep N2]. The best θ differs between the two instances, so no single θ is a safe default.

### 5.3 min_mu never adds a label

min_mu is a one-unit (1e-8 nat) penalty on the dearest share. Its singleton gain is 0 at every budget, and cur:1 = {0, min_mu, max_mu} labels exactly as the max_mu singleton wherever measured [S1]. `--mus-sweep full` keeps it; decades raises the lower end to max(min_mu, max_mu/1000).

### 5.4 Blue ties (implemented on branch `blue-ties`)

Letting blue win ties in the strong loop, at every μ, as the thesis does, with one guard the thesis lacks. On Aix it is a larger lever than the multipliers, and it does not vanish at large budgets. Sources: [S5] unless marked. Note: `--nb-mus 0` on the CLI runs the unconstrained function.

**The rule.**
- Both strong loops instantiate `strong_arc_default_traits`; both useless loops stay on `useless_arc_default_traits`.
- An arc is **zero-cyclic** if l^+ = 0 and both ends lie in one strongly connected component of the subgraph of *all* arcs with l^+ = 0, improvable ones included (`detail::zero_cycle_arcs_map`). It seeds v one unit higher. On integer keys b + 1 ≤ r ⟺ b < r, so this is exactly red ties for that arc, and the fiber seeds inherit the unit through the stored D.
- The shares are `trunc(nextafter(μ · nextafter(s_o, 0), 0))`; P(μ) stays `trunc(μ·B)` (Floating point below).
- History: the strong loop used `strong_arc_default_traits` until 83729a6 (2023-06-27, "ok for preprocessing, mu shitty"), the day after 96ba172 ("to dezbug arc contraction"). Unguarded blue ties do break the contraction, so that is the probable reason, though no message says so.

**What a blue tie certifies, and why that is enough.** A tie only says that uv is on *some* shortest u-t path for every feasible X. The contraction keeps one strong arc per tail (the first unimprovable one in sorted order; the others are skipped), so "some" is enough, unless arcs certify each other. On a cycle uv, vu of length 0, both are tie-strong for every t both reach equally, and the contraction merges u into v and v into itself. Improvable arcs must count in the component: K0 = {w→z p 1, z→w p 1−3e-9 improved to 1, w→t .5, z→t .25} has blue from z coming back to w through the improvable arc, and a guard over unimprovable arcs loses flow (1.5 → 1 with sources at w and t). Ties in the useless loop stay invalid even without zero lengths: two parallel equal paths make each other's arcs useless, and both go.

**Theorem.** With the guard, for every target t and feasible X, every vertex that keeps source flow has the same X-distance to t in G_t as in G.

*Lemma 1 (per-arc claim).* Let (t, wz) be a label, F_t the fiber when t was claimed, and Q a simple w-t path avoiding wz. (a) w is never claimed in a pass of wz, so w ∉ F_t. (b) If y is the first vertex of Q in F_t or claimed in t's pass, then l^X(Q[w..y]) ≥ D(y), where D(y) = l^+(wz) + l^+(P_y) for a walk P_y from z to y in G − wz.
- (a) A blue key at w is at least l^+(wz) + l^+(closed walk back to w) + P ≥ 1 + P > 0, the red key, because a closed walk through wz has base length ≥ 1: either l^+(wz) ≥ 1, or wz is not zero-cyclic, so no base-0 walk leads from z back to w. For a guarded arc the strict rule gives (a) anyway.
- (b) Let μ' be the pass that claimed y and F_y ⊆ F_t its fiber. Q[w..y] lies in H_{F_y} and no interior vertex is blue in that pass. The interior need not be settled when y pops, since melon stops once no blue candidate is left, so argue by heap order: let x be the first vertex of Q[w..y] not settled red when y pops. Its predecessor was settled red, so x holds an entry ≤ (λ^{μ'}(Q[w..x]), red); pops come in non-decreasing comparator order (settling on seeding keeps this, as z's seed is the smallest blue key), and y popped first, so D(y) + P(μ') ≤ λ^{μ'}(Q[w..x]) ≤ λ^{μ'}(Q[w..y]). Hence D(y) ≤ λ^{μ'}(Q[w..y]) − P(μ') ≤ l^X(Q[w..y]) by §1.3. This uses nothing specific to μ' = 0.

*Corollary.* d^X(w) = l^X(wz) + d^X(z) for every feasible X: the weak sense. It is not sufficient alone: unguarded blue ties satisfy it everywhere and still break G_t.

*Proof of the theorem.* Fix t and X. Write σ(u) for the arc that contracts u. An arc is **tight** if d^X(tail) = l^X(a) + d^X(head) < ∞, and **allowed** if it is not useless, its tail is not t, and its tail has no σ or a = σ(tail). Paths of allowed arcs are the paths of G_t. It suffices to show that every s with d^X(s) < ∞ is **good**: a walk of allowed tight arcs leads from s to t. Facts: tight arcs are never useless (this needs S ≤ P); σ(u) is tight (corollary); σ arcs are unimprovable, so l^X(σ(u)) = l^+(σ(u)); every u ≠ t with d^X(u) < ∞ has an allowed tight out-arc.

Induct on the values δ of d^X, increasing. Let C be the non-good vertices at level δ, and suppose C ≠ ∅.
- C is closed under allowed tight arcs, and tight arcs inside C have X-length 0.
- Following a shortest path out of C gives c₀ ∈ C with a tight arc leaving C. That arc is not allowed, so c₀ has σ(c₀) ≠ that arc, and σ(c₀) enters C at X-length 0, hence base length 0.
- Step. Let w ∈ C with σ(w) = wz, z ∈ C, l^+(wz) = 0, and a tight arc ww' with w' ∉ C. Then w' is good; an allowed tight walk from w' to t, with its cycles removed, never enters C, so Q = ww'·(that walk) is a simple X-shortest w-t path avoiding wz. Lemma 1 gives y ∉ C and d^X(w) = l^X(Q) ≥ l^+(wz) + l^+(P_y) + l^X(Q[y..t]) ≥ l^X(wz) + l^X(P_y) + l^X(Q[y..t]) ≥ d^X(w). All are equalities, so l^X = l^+ on P_y and all its arcs are tight. P_y leaves C from a last vertex c₁ of its prefix inside C; the prefix has base length 0, and c₁ satisfies the hypothesis of the step.
- Iterating gives w₀, w₁, … in C joined by nonempty base-0 walks starting with σ(wᵢ). C is finite, so some wⱼ = wᵢ: a closed base-0 walk through σ(wᵢ), which is therefore zero-cyclic and uses red ties, so its labels are strict and it lies on every X-shortest wᵢ-t path. But Qᵢ is an X-shortest path avoiding it. Contradiction. ∎

**Floating point.** The theorem and the strictness of useless labels need S ≤ P, where S sums the shares of X along a path.
- The old rounding breaks it by one unit at μ = max_mu when an option's cost fills the budget (c = B = 11, |A| = 5: fl(μ·fl(c/|A|)) reaches the integer Δl while fl(μ·B) lands just under 5·Δl). Red ties' strict comparison hid this for strong labels, which is the whole reason blue ties looked unsafe at μ > 0: a crafted path instance breaks them 200 of 200 under the old rounding and 0 of 200 now. It did not hide it in the useless loop, which labelled X-tight arcs (130,511 labels in 400,000 instances of a generator built for it, in the shipped red-tie code; no distance loss found).
- The fix: pred(fl(x)) < x(1 − 2⁻⁵⁴) for x > 0, so S ≤ μ·B·(1 − 2⁻⁵⁴)² < μ·B/(1 + 2⁻⁵³) ≤ fl(μ·B), and S is an integer, so S ≤ trunc(fl(μ·B)), at every magnitude up to the cap. Proved twice independently; checked on 5.3M exact-rational adversarial cases and exhaustively on mini-floats, 0 violations (nearest shares: 194,823 violations in the same search).
- Rounding the budget penalty up as well was tried and dropped: it is redundant by the above and can lose a valid label.
- One step of `nextafter` instead of two is the open choice, see Owner's call.
- Solver tolerance: the proof assumes Σc ≤ B exactly; the MILP accepts Σc ≤ B + tol. The slack is μ·tol units: at the default `-t 1e-7`, about 1.7 on Aix, 2.3 on quebec, 363 on aude, 0.003 on bike_mid. It is pre-existing and one-sided: walks of G_t are walks of G, so sets in the band can only be under-valued (−6e-7 at most on 90,000 instances; exact-feasible sets exact).

**Blue ties at μ > 0 add almost nothing.** Identical labels to blue at μ = 0 only on all 25 real cells, at 9 and 20 steps; 0-2 labels per 2,000 fuzz instances, about 84 per 100,000 on the zero-cycle generator. At μ = 0 ties are structural (13,964 of aix70's 19,880 reformulated arcs have length 0). A new tie at μ > 0 needs l^+(uv) + d^+(v, w) + P(μ) to equal red's distance to the unit. Allowing them costs nothing and removes the special case; they also recover labels the rounding would otherwise lose (`preprocessing.blue_tie_at_positive_mu`).

**Gain: contracted arcs, red → blue** (reformulated, resolution 1e-8; labels are identical for the final code on all 25 cells; `--nb-mus 9`, decades):

| cell | `--nb-mus 0` | `--nb-mus 9` |
|---|---|---|
| aix30 B=1 | 22,101 → 14,765 (−33.2%) | 17,037 → 9,171 (−46.2%) |
| aix30 B=1.72 | 28,384 → 19,978 (−29.6%) | 25,514 → 16,374 (−35.8%) |
| aix30 B=4 | 29,259 → 20,965 (−28.3%) | 29,175 → 20,855 (−28.5%) |
| aix40 B=1.42 | 53,829 → 37,125 (−31.0%) | 47,476 → 29,840 (−37.1%) |
| aix40 B=2 | 59,730 → 43,525 (−27.1%) | 54,259 → 36,488 (−32.8%) |
| aix40 B=4 | 59,730 → 43,525 (−27.1%) | 59,597 → 43,376 (−27.2%) |
| aix70 B=1 | 362,916 → 232,130 (−36.0%) | 272,552 → 141,341 (−48.1%) |
| aix70 B=2 | 416,593 → 282,449 (−32.2%) | 359,113 → 222,713 (−38.0%) |
| aix70 B=3 | 416,593 → 282,449 (−32.2%) | 387,993 → 253,719 (−34.6%) |
| aix70 B=5 | 416,593 → 282,449 (−32.2%) | 411,698 → 278,135 (−32.4%) |
| aix70 B=10 | 416,593 → 282,449 (−32.2%) | same |
| quebec B=2133 | 221,314 → 220,798 (−0.2%) | 196,279 → 195,763 (−0.3%) |

Aude and bike_mid: none. Useless labels never change. From [S1]: aix_N2 −0.37 to −0.40%, biorevaix_N2 −1.52 to −1.71%; where only the cheap 0.2-0.4-cost options fit, blue removes most of the graph (−98.8% at aix30 B=0.344, −99.6% at aix70 B=0.307).

**End to end** (`flow_mip`, `--solver gurobi --big-m tight --max-threads 4`, median total seconds; same optimum everywhere; machine load 4.5-7 from niced agents; seeds per cell in the table, so aix70 at B = 1, 2, 5 and 10 is thin):

| cell | seeds | nm0 red | nm0 blue | x | nm9 red | nm9 blue | x |
|---|---|---|---|---|---|---|---|
| aix30 B=1 | 5 | 0.29 | 0.25 | 0.87 | 0.55 | 0.58 | 1.05 |
| aix30 B=1.72 | 5 | 0.57 | 0.53 | 0.92 | 0.86 | 0.81 | 0.94 |
| aix30 B=4 | 10 | 1.81 | 1.52 | 0.84 | 1.93 | 2.05 | 1.06 |
| aix40 B=1.42 | 5 | 1.42 | 1.24 | 0.87 | 1.80 | 1.85 | 1.02 |
| aix40 B=2 | 5 | 2.05 | 1.99 | 0.97 | 1.90 | 1.98 | 1.04 |
| aix40 B=4 | 10 | 1.99 | 3.09 | 1.55 | 2.98 | 4.04 | 1.35 |
| aix70 B=1 | 2 | 14.01 | 9.48 | 0.68 | 18.30 | 18.64 | 1.02 |
| aix70 B=2 | 2 | 31.41 | 15.29 | 0.49 | 23.34 | 23.12 | 0.99 |
| aix70 B=3 | 5 | 48.30 | 18.96 | 0.39 | 34.14 | 41.01 | 1.20 |
| aix70 B=5 | 2 | 83.26 | 61.43 | 0.74 | 94.62 | 62.99 | 0.67 |
| aix70 B=10 | 1 | 173.80 | 163.46 | 0.94 | 188.38 | 172.22 | 0.91 |
| quebec B=2133 | 3 | 4.04 | 3.99 | 0.99 | 3.90 | 3.78 | 0.97 |
| bike_mid B=155 | 3 | 3.33 | 3.35 | 1.00 | 1.93 | 1.92 | 1.00 |

- These runs used an earlier build whose labels equal the final code's on every cell (checked), and whose models the final binary reproduces on aix30 B=1, 1.72 and aix40 B=2.
- Blue `--nb-mus 0` beats red `--nb-mus 9` on aix70 at every budget. It also labels in a quarter of the time, so §2's advice to pay for multipliers on Aix should be re-measured with blue ties.
- The regressions (aix40 B=4, aix70 B=3 at nm9) are Gurobi branching where red solved at the root. The LP bound is identical red and blue in all 11 cells checked, so it is a search path, not a weaker relaxation. At aix70 B=3 with nm0 the same effect goes the other way.
- Labelling at `--nb-mus 9` costs +14-16% on aix70, +12-17% on aix40, +4-14% on aix30, about 0% on quebec, in same-binary pairs.
- The worst end-to-end deviation on the standard fuzz rises from 2.6e-9 to about 6e-9 relative (2.2e-8 at most, on the zero-cycle generator): ties hide differences smaller than one length unit, so it stays in the rounding-error class, far under the test's 1e-6.

**Verification of the final code** (fuzz harness of §4.1 extended with label-level, exact per-source and zero-cycle generators):
- Fuzz: 498,600 instances, 4.84M feasible scenarios, 30.3M end-to-end checks per variant, in release, assertion and ASan+UBSan builds: 0 strong violations, 0 useless violations, 0 end-to-end violations, 0 distance violations. Label sets identical to an independent reference implementation. The unguarded and blue-useless controls violate on every run that can show it (1.5M end-to-end, 23M useless).
- Earlier, on the same rule: 10M exhaustive tiny-instance checks and 800k fuzz, 0 counterexamples; two independent reviews of the proof.
- Real cells: labels identical to the reference on all 25.
- Raw instances, which do have zero-length cycles (zero-cyclic arcs: 1,556 on aix30, 3,632 on aix40, 9,240 on aix70; none on quebec or aude): 0 violations for `unc`, dec20 and nm0, up to 196,800 checks per run. Unguarded controls fail 3,168 of 3,168 target checks on raw aix30 B=1, silently, with 18 contributions of exactly 0.
- Tests: 21 mutants against `preprocessing.*`. Each new test fails on the change it guards: `zero_length_cycle` (guard, either header), `zero_length_cycle_with_improvable_arc` (K0), `zero_length_cycle_in_later_passes` (guard in the first pass only; fiber seeds losing the unit), `blue_tie_at_positive_mu` (red ties at μ > 0), `penalty_rounding` (nearest shares, in either loop, with positive controls one unit away).

**Coverage limits.**
- The 25 reformulated cells have no zero-cyclic arc, so the guard is inert on them; its real-world evidence is raw Aix.
- The fuzz cannot tell a rounded-up budget from a plain one (identical labels on 160,200 instances); the case for dropping it is the proof and targeted near-tie runs (19 valid labels regained on 540,000 instances).
- Surviving mutants, knowingly: the guard only at μ = 0 (no small instance: 0 flow failures in 600,000 with affordable options, 1 with options over the budget); one `nextafter` step instead of two (below); the unconstrained useless loop on blue ties is caught only by `constrained_labels`' superset check, not by a flow check.
- The headers contain no `assert`, so the assertion builds exercise melon's only.

**Owner's call.** Two `nextafter` steps on the share, or the inner one alone (`trunc(μ · nextafter(s_o, 0))`). One is a call shorter and keeps a few crafted near-tie labels; no label differs on the real runs compared, and `blue_tie_at_positive_mu` would have to move to a tighter instance. But its guarantee holds only for S ≤ 2^53: above that it can exceed P (cost 0x1.fffffffffffffp+12, |A| = 9, μ = 0x1.2000000000006p+50 gives S − P = 1280). Reaching it takes at least 4.89M improved arcs on one path at resolution 1e-8, but 326 at 1e-12 and 3 at 1e-14, so `--resolution` brings it within range. The outer step alone is worse: it fails on decimal-tight sets whose doubles overrun B (costs 119.7 and 1060.64, |A| = 7 each, B = 1180.34, μ = 100 gives S = 118034 > P = 118033). The code keeps two: no side condition.

**Contraction of a self-loop (fixed).** The contraction used to remove the vertex, and read freed memory (the range-for over `source_options_map[u]` reallocates it when u == v), when a strong arc had become a self-loop. It needs a cycle of contractible strong arcs, which the guard excludes and only the unguarded controls reached. It now removes such an arc and keeps its vertex (`preprocessing.contraction_of_a_strong_self_loop`, which also pins that the self-loop is gone); no other self-loop survives in practice: the contracted graphs of aude, aix30 and aix70 hold none, at `--nb-mus` 0 and 9, as the reverse arc of a contracted strong arc is labelled useless; the unguarded controls then run clean under ASan on the seeds that aborted, and still fail their flow checks.

**Pre-existing, unchanged.** A self-loop in the input trips melon's PRE_HEAP assertion in Debug, in all four loops; the CLI cannot reach it, as `trivial_reformulate_instance` drops arcs inside a component.

### 5.5 Zero-length arcs and cycles

See §3.6: zero-length arcs exist on Aix, quebec (under λ^0) and N2; zero-length cycles were found on no reformulated instance at any point tested.

## 6. The multiplier grid

### 6.1 cur:10 against top:3:N

- **Contracted arcs** [S1 judge]: top:3:10, the 3-decade grid with 10 points, ascending, no final 0 (`--nb-mus 9` today, which also appends the final μ = 0), gets 96.1-101.6% of dense:40's gt_arcs gain on Aix, 91.3-112.5% on bike, 92.1-100.8% on quebec, and 100.9% / 104.9% of top:2:5's on aix_N2 / biorevaix_N2 at 0.5% (dense:40 was not run on N2). cur:10 gets 85-100%, 24-85%, 52-88% and 66-94%.
- **No other grid holds on every family** [S1]; none with fewer passes does, and halve:10, with the same 11, fails on quebec:
  - top-decade grids do well on Aix (top:1:3: 83.9-102.5%) and, with 10 points, on bike (top:1:10: 93-112%; top:1:3 only 27-95%), but not on quebec ({max_mu} alone 0%, top:1:10 0-75%);
  - top:2:5 (6 passes) is the best cheap grid on Aix (≥ 83.9%) and the reference on N2, and good on quebec up to B=10000 (88-94%), but gets 11-20% at quebec B ≥ 20000 and 27-95% on bike;
  - the theta list has the highest share at 9 of the 11 non-nil Aix budgets (Aix 96.9-101.6%, N2 98.0-98.6%, quebec 79.8-112.5%) but collapses to 2-4 points on small bike budgets (39.2% at bike_small B=100, 46.8% at bike_mid B=155; 39-100% on bike);
  - cur:1 to cur:5 get 0-58% on quebec and N2;
  - halve:10 is best on bike (100-135%) but behind on quebec at 8533 and 10000 (93.7%, 83.5%).
- **Against top:3:10@asc in [S2]**, cur:10 loses 0.4-11% of the gain on Aix, 6-45% on quebec, 19-67% on bike and 7-42% on N2 [S2 judge]; the commit message's "0.4-67%".
- **Why.** cur:N spreads its points from min_mu, about 15 decades below max_mu on N2: at 0.5% cur:10 puts 2 points in N2's window and cur:3 none; on Aix only cur:10's last three points (0.041, 0.20, 1)·max_mu are in the useful window [S1].

### 6.2 Why three decades

- **The useful band** (§5.2) is the top decade on Aix, max_mu/2 on bike, μ/max_mu from 9.2e-2 down to 5.1e-3 on quebec, and 0.008-0.25 on N2. With 3 decades, quebec's θ ≈ 21 peak stays inside the window up to B ≈ 92,000 [S1]; quebec's gain is tiny beyond B=21,333 (43 labels at B=30000 [S2 skeptic]) and nil from B=42,665 [S1].
- **Two decades** fail quebec at B ≥ 20000 (top:2:10: 10.7-19.8%) and bike_mid B=155 (46.4%) [S1]. On aix70, a 2-decade grid labels more: top:2:10 + 0 gains 76,024 labels over unc at B=2, against 75,775 for top:3:10 + 0 (top:2:10 at the fixpoint, 76,102, is the cell's ceiling) [S2 sweep aix70].
- **The bottom decade is not dead weight on quebec, N2 and bike_mid**: in ascending order, passes below max_mu/100 add labels on quebec (top:3:21: 4,565 at B=1000, 11,028 at B=2133; top:3:10: 3,344, 6,733, and 159 at B=21333) and add fiber on N2 and bike_mid [S2 judge].
- **On Aix it nearly is.** On aix30 and aix40 the lowest three points of top:3:10 add 0-12 labels by the judge's per-pass table (aix70: 44 at B=1) [S2 judge] and 0 gt arcs [S2 sweep aix]. On aix70 the bottom 6 points are dead weight once the final 0 runs: 0, p7..p10, 0 (6 passes) gains 75,765 labels against 75,775 for the 12-pass 0 / asc / 0 at B=2, and the same labels at B=1, at 0.60 of the cost of asc with frontier and redbreak; halve:3 + 0 (0, max_mu/4, max_mu/2, max_mu, 0) gets 75,673 at B=2 and 112,360 at B=1 (asc + 0: 112,093) at 0.52, halve:2 + 0 75,570 at 0.445 [S2 sweep aix70]. These short top-heavy grids are safe only where the useful μ sit at max_mu; on quebec (θ ≈ 21) and N2 (θ 3-12) they would miss them. They were measured on aix70 only, and the agent asked for checks elsewhere before any default [S2 sweep aix70].
- **A 4th and 5th decade** add almost nothing [S2 skeptic]: top:5:36 beats REC only at quebec B ≥ 15000, by 124, 24 and 8 labels (B = 15000, 21333, 30000), at most 0.04% of arcs; on aix_N2 at B=214.86 the 4th / 5th decade gain 1 label against 3 for REC, and at B=430 no grid gains anything. cur:10 finds 36 labels that REC misses over 7 quebec cells.
- Only D=3 was tested from the CLI; D was chosen on the instances that score it (§11).

### 6.3 How many points

Here N counts positive points (top:3:N), not `--nb-mus` steps: `--nb-mus N` runs top:3:(N+1).

- **aix70, whole graph, B=2, ascending** [S2 sweep aix70], label gain against top:3:10's 75,302: N=1 95.9%, N=4 97.7%, N=6 99.3%, N=8 99.84%, N=10 100%, N=13 +0.22% (1.27x passes), N=16 +0.30% (1.54x), N=21 +0.30% (1.98x); cur:10 −720 labels. At B=1: N=10 111,878, N=21 112,344, cur:10 111,260.
- **aix_N2, 0.5%, 10% arc sample** [S2 sweep N2], against top:3:10 (plain cost in brackets): N=4 −2.8% (0.47), N=6 −12.8% (0.64), N=8 −5.0% (0.82), N=13 +0.52% (1.27), N=16 +1.41% (1.54), N=21 +2.51% (1.99), N=31 +2.97% (2.89), N=41 +3.23% (3.78). biorevaix_N2: N=21 +4.2%, N=41 +4.99%. aix_N2 at 1%: N=21 +10.1%. Below 13 points the result depends on whether a point lands near the peak (top:3:4 contains θ 12.3, the best singleton, and beats top:3:6 and top:3:8).
- **21 against 11 points**: quebec +7.4 / +6.7 / +4.9 / +8.1% of the gain at B = 1000 / 2133 / 4267 / 8533, and 11 points alone are 5.6% below top:3:10 at B=8533 [S2 judge]; run both as 0 / asc / 0, +7.7 pp at B=2133 (0.96% fewer contracted arcs than 11 points, 0.85 pp of unc's) and +11.9 pp at B=10000, +4.2 pp on aix_N2 at B=80 [S2 skeptic].
- **41 points** can add up to +18.1 pp of the gain (aix40 B=3.4), +8.5 pp at quebec B=10000, +5.2 at B=15000, +4.7 at aix40 B=1.7, +3.5 at aix30 B=3.44, but at most 0.31 pp of contracted arcs [S2 skeptic]. This corrects the judge's "+0.4 to +1.6%".
- **The bike peak**, corrected by [S2 skeptic]. Earlier agents read bike's gain as a sharp peak: [S1 judge] chose N=10 for top:3:10's point at 0.464·max_mu, [S2 sweep quebec/bike] put a spike at 0.500-0.505·max_mu that 11-point grids hit and 10-point grids miss, and [S2 judge] took 21 points as needed. Over N = 8..42 points on bike, the gain peaks exactly at N ≡ 1 (mod 10), the grids that contain 1000^(−1/10) = 0.501·max_mu; 11 points give exactly REC's labels on all 10 bike cells; the single-point gain stays within 4-9% of its peak for f ∈ [0.45, 0.55]·max_mu, a broad peak that top:3:10's point at 0.464 just straddles. Grids not aligned on it lose up to 13% (bike_mid B=155: N = 13-18 lose 7-13%, N=26 8.4%).
- **On bike the two metrics disagree**, because the gt_arcs share saturates there: at bike_mid B=78, asc on top:3:10 is at 99.2% of the cell's best gt_arcs gain but has 4% fewer labels than top:3:11; bike_small B=100 is at a 73.9% gt share for a 12-label gap [S2 sweep quebec/bike].
- **16 points is a trap**: it misses 0.501·max_mu, −12.4% of the gain at bike_mid B=155 [S2 judge]; top:3:16 reaches 41.9% of the cell's best gt_arcs gain there [S2 sweep quebec/bike].
- **Gains are not monotone in N** beyond about 8 points: top:3:8 > top:3:11 at aix40 B=2.84, top:3:31 > top:3:41 at aix40 B=2.84 and B=4 [S2 sweep aix]; a ratio-2 grid can straddle quebec's broad θ peak (11 points lose 4.4-5.6% at B=8533, where top:3:10 has a point at 19.4 nats) [S2 sweep quebec/bike].
- **More points beat more cycles** at the same pass count: top:3:21@asc against top:3:10@fixpoint, about 2x passes each: N2 +2.51 / +4.20 / +10.1% against +0.007 / +0.34 / +0.063% [S2 sweep N2]; on 4 of 5 quebec cells, +1134 / +2448 / +103 labels against +290 / +363 / +30, and on 6 of 7 bike cells [S2 sweep quebec/bike]; cycles win at quebec B=4267 (+670 against +730) and at aix40 B=2 (+5 against +15) [S2 sweep quebec/bike, S2 sweep aix]; two cycles over 11 points (24 passes) are 4.2-9.0 pp below REC at quebec B = 1500-6000 [S2 skeptic]. On aix70 at B=2 it is the reverse (fixpoint 75,801, top:3:21 75,527), at B=1 not [S2 sweep aix70].

### 6.4 Nesting and the formula change

- **The old formula**, lo·pow(r, k) with r = (max_mu/lo)^(1/N), does not nest: the 11-, 21- and 41-point grids are not bitwise nested in 2,005 of 2,005 grids tested, the real ranges of aix, quebec, bike and N2 included [S2 skeptic]; N ⊆ 2N fails in 74% of 140,000 checks [S3 fuzz]; 24-26 of the 32 points the 10 / 20 / 40 grids should share differ on 5 real ranges [S2 critic]. Labels lost by it in measurement: 0 (top:3:11 ⊆ REC in 29 of 29 cells [S2 skeptic]).
- **The new formula**, lo·pow(max_mu/lo, k/N), nests: 0 failures in 15,000 grid pairs [S2 skeptic]; 0 violations of N ⊆ 2N over 280,000 per-instance checks and 2.8 million random ranges [S3 fuzz]; grid N inside 2N and 3N over 3.5M random cells, N = 1, 9, 10, 20, 21, 40 included [S3 review].
- **Shift.** The change moves 487 of 672 grid points bitwise on 16 cells, by 0-17 ulps of the old value (medians 1 for dec9 and cur:10, 3 for dec20; maxima 8 for dec9, 17 for cur:10, 11 for dec20; largest relative shift 2.4e-15) [S3 gate], and by up to 20 ulps at N=20 and 39 at N=40 on random 3-decade ranges [S3 fuzz]. The brief's "1-3 ulps" was too tight.
- **Effect on labels: none in any comparison made.** No floor(μ·B) changes; 0 label differences in 48 comparisons (16 cells × dec20, dec9, cur:10) [S3 gate]; the fuzz snapshot with and without the new formula: 0 differences [S3 fuzz]. That supports carrying the old-formula numbers of §5-§9 over to the landed grid, but the other grids and cells there (top:3:4 to top:3:41, halve, top:4 and top:5, theta lists, [S1]'s grids) were not rerun, and every number measured before the change is for the old grid [S2 critic].
- **Labels nest**: 11 ⊆ 21 ⊆ 41 on all 16 gate cells, and 41 adds labels over 21 on 12 of them (quebec B=2133 +309 / +306, aix70 +99 / +125) [S3 gate]; D1 ⊆ D2, D10 ⊆ D20, D20 ⊆ D40 with 0 violations in every fuzz run [S3 fuzz].

### 6.5 21 points do not dominate top:3:10

Corrected by [S2 skeptic], confirmed by [S2 critic]:
- At quebec B=10000, REC has 38 fewer labels than top:3:10 (565,439 against 565,477, −1.56% of a 2,433-label gain; gt_arcs +0.013%), and lacks 173 of top:3:10's labels.
- REC misses some of top:3:10's labels in 9 of 29 cells, 686 in all, 294 of them at quebec B=2133.
- Counting the denser grids, REC reaches 76.7-100% of the best spec seen in a cell, not the judge's 97.7-100%: 92.1% at quebec B=10000, 84.8% at aix40 B=3.4, 95.5% at aix40 B=1.7. The 76.7% (quebec B=30000, 33 of 43 labels) and 88.0% (B=21333, 176 of 200) cells rest on tiny counts.
- On average REC gains: +0.25 to +13.9% of the gain on the other cells with a gain, and up to −2.5% of contracted arcs [S2 skeptic].

### 6.6 The decades window is anchored on max_mu

max_mu is the largest improvement per cost of a single (option, arc) pair. One option that improves an arc far more per unit of cost than the rest (a cheap option on a near-zero-probability arc, say) raises max_mu, and the whole window with it, above the multipliers that gain labels; cur:N (`full`) still has points down to min_mu.
- **Constructed counterexamples** [S1 skeptic]: on bike_mid B=155 with the option that sets max_mu made 1,000 times cheaper, top:3:10 gets 70.7% of dense:40's gain against 99.6% for cur:9 (the same 11 passes); 100,000 times cheaper, 0% against 101.0%. aix30 with option 137 at cost 1e-5: 0% against 82.5% (B=1.72) and 81.2% (B=1). The same option 1,000 times cheaper on aix30 is not a counterexample: top:3:10 leads with 90.1%.
- **The review's counterexample** [S3 review], CLI, B=2133, labels per target strong / useless:

| quebec copy | unconstrained | decades N=9 | decades N=20 | full N=9 | full N=20 |
|---|---|---|---|---|---|
| original (max_mu 2.27e7) | 341.20 / 1061.65 | 363.01 / 1101.23 | 365.57 / 1104.88 | 356.15 / 1090.69 | 360.59 / 1096.80 |
| vertex_1625 at cost 10 (max_mu 1.84e8) | – | 363.36 / 1101.14 | 365.19 / 1104.40 | 357.02 / 1089.28 | 362.12 / 1099.07 |
| vertex_1625 at cost 1 (max_mu 1.84e9) | 341.21 / 1061.65 | 352.74 / 1083.34 | 352.74 / 1083.34 | 349.65 / 1077.06 | **359.58 / 1095.62** |

  At cost 1 (instead of 4,212), decades N=9 and N=20 give identical labels: the window has left the useful range. At N=20 decades gains 33.2 labels per target against 52.3 for full; on the original instance decades gains 67.6.
- **Library path:** a cost-0 option sends max_mu to the cap; at B=2133 the window becomes [2.2e12, 2.2e15] and every positive pass is a no-op. The CLI drops cost-0 options in `trivial_reformulate_instance` [S3 review].
- **Owner's instances are safe so far**: their extreme pair sits only 2.0-9.2 times above the median pair (quebec 6.2 and 9.2, aix70 2.3, bike 2.0, aix_N2 5.0), against about 2,000 and 200,000 times in the constructed cases [S1 skeptic].
- **Never implemented or run:** a median guard [S1 skeptic], about 4 lines: take the median pair ratio, and fall back to min_mu if it is below max_mu/1e4. It would not fire on the owner's instances; on the constructed cases its effect is inferred from their cur:9 runs: 0% back to 81-101% when the option is 100,000 times cheaper; it stays off when 1,000 times cheaper, where cur:9 beats top:3:10 on bike_mid but not on aix30.

## 7. Order of the multipliers

The owner asked whether the order matters, since the fiber carries over from one multiplier to the next. It does, but mainly through where μ = 0 sits.

### 7.1 μ = 0 first

| Family | 0 last, ascending (asc0last) | 0 last, descending (desc0last) | No μ = 0 |
|---|---|---|---|
| aix30, aix40 | Same labels as asc + 0 on 10 of 10 cells, with no proof that it must; its first point, max_mu/1000, is not inert there (it adds 10 labels on aix40 at B = 1.42, 2 and 2.84) [S2 sweep aix, S2 judge] | Loses 53-92% of the gain | no0asc falls below unc on the nil-gain cells; no0desc 200 gt arcs worse than unc at aix40 B=4 |
| aix70 | Same as asc + 0 at B ≤ 2, where p1 = max_mu/1000 acts as 0 [S2 sweep aix70]; +48 / −22 labels at B=3 (sample) | Loses 54 / 67 / 88 / 92% at B = 1 / 2 / 3 / 5 (all on the 10% sample) | no0asc loses 320 unc labels at B=10 |
| quebec | Loses 12.8 / 25.0 / 40.4 / 61.4 / 76.1% at B = 1000 / 2133 / 4267 / 8533 / 21333 | Loses 55-87% | Loses more than the whole gain |
| bike | Up to −0.6% | Loses 7-47% | – |
| N2 (10% sample) | Loses 8.7 / 14.1 / 18.8 / 69.6% (aix_N2 0.5%, biorevaix 0.5%, aix_N2 1%, 2%) | Loses 84-96% | Below unc: no0asc −222,812 labels at aix_N2 2% |

Sources: [S2 sweeps, S2 judge]. asc0last is free but not safe: when the first positive point is not negligible it breaks (halve:3@asc0last keeps 31-38% of the gain on aix70) [S2 sweep aix70].

### 7.2 The final μ = 0

- **Proved** never to lose a label: appending a pass can only add fiber vertices (§7.4) [S2 theory]. Ascending without the final 0 is a subset of REC in 31 of 31 cells [S2 skeptic].
- **Cost:** one pass, +7-9% of the plain counters, +4-8% with frontier [S2 judge]; +6-7% wall time over frontier alone on every large cell (dec9 / top:3:10 with frontier: 1.064-1.073) [S2 timing].
- **Gain:** aix70 whole graph +473 labels (+0.63% of the gain, gt_arcs −260) at B=2 and +215 (+0.19%) at B=1 [S2 sweep aix70]; on the 10% sample of aix70 B=2 it measured +1.36%, about twice the whole-graph figure [S2 judge]. On 21 points: quebec +1.38 / +1.88 / +3.14 / +0.45% of the gain, Aix small cells 0-0.26%, N2 0-0.014%, bike 0-0.2% [S2 judge]. On the raw quebec test instance at B=2133 it adds 501 labels (§4.5).
- **Why it works.** In the fixpoint's second cycle, the μ = 0 pass supplies most of the extra labels: 473 of 499 at aix70 B=2 and 215 of 308 at B=1 [S2 sweep aix70], 314 of 363 at quebec B=2133 and 395 of 730 at B=4267 [S2 sweep quebec/bike]. A blocker won at a large μ unlocks dependents that a later μ = 0 pass claims (the G1 mechanism, §7.6) [S2 theory]. On top:3:10, asc + 0 thus gets 95% (B=2) and 70% (B=1) of the fixpoint's extra labels on the whole aix70 graph [S2 sweep aix70].
- **It does not always reach the fixpoint.** On fuzz, asc + 0 reached the per-arc fixpoint on every instance [S2 theory], but on real instances REC stays short of the 21-point fixpoint by 1.98 pp of the gain at quebec B=1500, 0.69 at B=3000, 0.35 at B=6000, 0.27 at B=2133, 0.12 at aix40 B=1.7 and 0.10 at aix70 B=1.228 [S2 skeptic]. On top:3:10, asc + 0 reached the fixpoint on all 10 aix30 / aix40 cells [S2 sweep aix] and got 80 / 86.5 / 54 / 90% of its extra labels on quebec at B = 1000 / 2133 / 4267 / 8533 [S2 sweep quebec/bike].

### 7.3 Order of the positive multipliers

Among one-cycle orders that start with 0:
- **Aix and N2:** asc is the best or within 0.36% (Aix) and 0.21% (N2) of the best. desc, bisect and zigzag lose up to 2.95% of the gain (aix30 B=1.72; 126 labels at aix40 B=1.42) [S2 sweeps aix, N2].
- **quebec:** asc is near the worst, ranked 1, 7, 2 and 1 by labels (1 = fewest) among the 56-58 one-cycle orders tried at B = 1000 / 2133 / 4267 / 8533. The best permutation gains +1.63 / +1.19 / +7.10 / +0.69% of the gain; bisect +7.05% at B=4267 [S2 sweep quebec/bike, S2 critic].
- **bike:** every order within ±2 labels [S2 sweep quebec/bike].
- **No order wins on both.** The quebec/bike agent recommended bisect + 0 on top:3:10: it gets 94 / 90 / 99 / 97% of the fixpoint's extra labels on quebec (B = 1000 / 2133 / 4267 / 8533) against 80 / 86.5 / 54 / 90% for asc + 0, and exactly the fixpoint on every bike cell, at 1.04-1.07x asc with frontier and redbreak [S2 sweep quebec/bike]. The judge rejected it as not robust: on top:3:10 it is −2.84% at aix30 B=1.72 (asc + 0: +0.035%), and on 21 points it gives +12.2% against +9.7% for asc + 0 at quebec B=4267, but −0.85% against +0.26% at aix40 B=2 [S2 judge]; bisect + 0 and desc + 0 lose 1.9-3.0 pp on aix30 B=2.3 and aix40 B=1.7 [S2 skeptic].
- **Oracles do not help.** On aix70 singleton gains rise monotonically with μ, so the oracle order (decreasing singleton gain) is desc and the anti-oracle asc, hash-identical; the greedy marginal order ends at desc's labels [S2 sweep aix70]. The per-loop splits (S=asc|U=desc and the reverse) never beat the better of the two orders on quebec, bike and aix70; on biorevaix_N2, S=asc|U=desc beats both by 0.007 pp (+0.084% of the gain, desc +0.077%) [S2 sweeps].
- **Cost:** every one-cycle order costs 0.999-1.017x asc [S2 judge].
- Decision: fix the order as 0, ascending, 0 (under the default `--mus-sweep decades`; `full` keeps 0, ascending, without the final 0, §2.1); expose no option [S2 judge, S2 design].

### 7.4 Fixpoint and the idempotence skip

- **Theory** [S2 theory, proved]: a pass is monotone in F, inflationary and idempotent, so for a fixed multiplier set every finite sequence (any order, repeats, early stops) gives labels inside the per-arc fixpoint, and the fixpoint does not depend on the order. More generally, a subsequence never yields more labels than its supersequence, so inserting multipliers anywhere, a final μ = 0 included, never loses a label. Skipping a multiplier whose last pass saw the same fiber is exact (equal size suffices, since fibers only grow).
- **Observed:** fixpoints from asc and desc are hash-equal on every cell run (10 aix30 / aix40, 15 quebec / bike, 4 N2), and so is the asc + 0 fixpoint on quebec / bike; on aix30 / aix40 asc + 0 itself equals the fixpoint, and on N2 asc cycle:2 does [S2 sweeps]. Two cycles always suffice for labels: max_cycles 3-4, and cycles 3 and 4 add only non-target fiber vertices [S2 sweeps].
- **Cost** relative to REC, counters [S2 critic]:

| Cell | Fixpoint (re-runs every pass) | Fixpoint with the idempotence skip |
|---|---|---|
| quebec B=1500 | 1.919 | 1.160 |
| quebec B=2133 | 1.908 | 1.163 |
| quebec B=4267 | 1.903 | 1.066 |
| bike_mid B=155 | 1.662 | 1.329 |
| aix30 B=1 | 1.843 | 1.313 |
| aix40 B=1.7 | 1.870 | 1.209 |
| aix70 B=2 | 1.86 | 1.251 |
| aix_N2 sample | not run | 1.327 |

  aix70 B=2 labelling: 29.2 s with the skip, 42.5 s without, 23.5 s for REC. The judge's "1.8x, not worth it" was the cost without the skip.
- **The skip checked empirically** [S2 critic]: fixpoint with the skip is hash-identical to the plain fixpoint on all 8 cells, and REC with the skip to REC. On fuzz, the fixpoint in asc order with the skip costs +0.026-0.039% passes over asc, against +15.0-38.5% in desc order, so it must be computed in asc order [S2 theory].
- **Gain over REC:** quebec +1.98 / +0.27 / +2.62 pp of the gain (B = 1500 / 2133 / 4267; gt_arcs −0.25 / −0.035 / −0.10%), aix40 B=1.7 +0.12, aix70 B=2 +0.06, N2 +2 labels, bike and aix30 B=1 0 [S2 critic].
- **Verdict:** the one exact strategy that makes the order irrelevant; it pays only on quebec. About 10 lines in the header; **not implemented** (it was optional). REC with the skip applied only to the trailing 0 saves 0.5-2.7% of the counters: not worth it [S2 critic].
- A pruned second cycle (rerun 0, then only the multipliers that added fiber in cycle 1) reproduced the fixpoint on 15 of 15 quebec / bike cells at 1.35-1.57x on quebec; the rule is empirical [S2 sweep quebec/bike].

### 7.5 stopK

Stopping a loop after K passes without fiber growth is lossy and order-sensitive [S2 sweeps, S2 judge]:
- asc + stop1 stops right after the inert low points: it returns about unc's labels on Aix and loses 35-100% of the gain on quebec and bike, 7.5-78% on N2.
- desc + stop2 with frontier and redbreak keeps 99.86% (B=2) and 100.02% (B=1) of asc's gain on aix70 at 0.41x the cost of asc with both, because the useful multipliers sit at max_mu there; it loses 96-100% of the gain on quebec and N2.
- Elsewhere desc + stopK fails: on the small Aix cells desc + stop1 loses up to 557 labels and desc + stop2 up to 176 (aix40 B=1.42) [S2 sweep aix]; on N2 desc + stop1 returns exactly unc's labels, since its first positive pass, at max_mu, adds nothing there [S2 sweep N2].
- Other variants: on N2, asc + stop3 loses 0.18% / 0.06% of the gain (aix_N2 / biorevaix_N2 at 0.5%) at 0.53 / 0.46 of the plain cost, and top:3:21@asc + stop5 + frontier + redbreak gives +2.02 / +4.06 / +9.14% over top:3:10@asc (aix_N2 0.5%, biorevaix_N2, aix_N2 1%) at 0.52 / 0.37 / 0.36 of plain top:3:10's cost, but in asc order stopK skips the top passes, where Aix gains [S2 sweep N2]; on quebec, oracle + stop1 loses 2-18% at 0.16-0.22x [S2 sweep quebec/bike].
- On fuzz, stop1 keeps 0.4-39% of the fixpoint gain and stop2 11-87% [S2 theory].
- Rejected.

### 7.6 Why: the fiber acts as a set

- Stored distances are canonical (§1.2), so the fiber matters only as a set of vertices: each claimed vertex removes red's arcs into it and becomes a blue seed at its canonical distance [S2 theory, S2 critic]. μ = 0 first wins because it claims the most blockers for the least penalty.
- The sweep agents' explanation, that a vertex claimed early keeps an inflated "frozen" distance, is wrong: the stored distances were checked canonical on real instances (§1.2) [S2 critic].
- **Single sweeps are order-dependent in both directions** [S2 theory]: G1 (5 vertices, 7 arcs, 2 options of cost 0.75, B=1) has a label desc finds and asc misses; G3 the reverse. A blocker won only at μ > 0 needs a red path whose options cost more than B (slope lemma), so at least 2 options and 2 improvable arcs.
- **The set of multipliers that win a vertex is not an interval**, even for a fixed fiber and without floors (counterexample G2: won at μ ≤ 8.9 gu, lost for 9.1-19.9 gu, won again from 20.1 to 1000 gu, with 1 gu = 1e6 length units), because intermediate vertices block red paths. No grid can be pruned by window or bisection reasoning. At F = {} on a 42-point grid, 0 non-interval triples were found among 371,113 (ref) and 251,025 (raw) [S2 theory].
- **Chaining the fiber helps asc a little:** the "star" strategy, where every multiplier is seeded by the μ = 0 fiber only, is a subset of every chained order and falls short of asc by up to 3.0% of the gain (aix30 B=1.72: 85 of 2,851) [S2 sweep aix].
- On fuzz, the Lagrangian gains anything in only 0.4-1.1% of instances; asc reaches 96-99% of the fixpoint gain and desc with μ = 0 last (desc0last in §7.1) 89-95% [S2 theory].

### 7.7 Sampling caveat

aix70's order matrices (full at B = 1 and 2, reduced at B = 3, 5 and 10) and every N2 cell ran on 10% arc samples; at aix70 B = 1 and 2 only the key strategies were rerun on the whole graph [S2 sweep aix70, S2 sweep N2]. The restriction itself is exact (0 extra, 0 missing labels in 8 checks), but order effects sit on few arcs: desc − asc is +102 (seed 1), +52 (seed 2) and −45 on the whole graph at aix70 B=2; the differences lie on 71-127 arcs, the top 10 carrying 36-64% of them [S2 sweep aix70]. The order conclusions for aix70 B = 3 / 5 / 10 are sample-only, and so is every N2 order conclusion, aix_N2 and biorevaix_N2 at 0.5% included: the only whole-graph N2 run had unc, top:3:10 and top:3:21, in asc order [S2 sweep N2]. The N2 sweep used sample seed 1; [S2 skeptic]'s seed-2 reruns ran aix_N2 B=53.72 (asc and asc + 0 on top:3:10, REC) and B=80 (cur:10 in asc; top:3:10 in asc and asc + 0; top:3:11 in asc + 0; top:3:21 in asc, asc + 0 and bisect + 0). Use samples for cost and number of points, not for order.

## 8. Cost reductions

### 8.1 frontier

- **Definition.** Before the fiber seeds of a pass are pushed, drop from `mu_fiber` every vertex w whose out-arcs other than uv all enter the fiber. Such a w relaxes nothing, and keeps relaxing nothing because the fiber only grows. The explicit seed (v or u) is never dropped and is pushed first; dropping it when dead is unsafe, because `add_*_source` settles red during seeding [S2 build].
- **Exactness** [S2 theory, proved]: in Lemma 1 seeds matter only through D and through blue reachability. The last fiber vertex on a tight path to x has its next arc leaving the fiber, so it is a frontier vertex and stays seeded; a dropped seed's only effect was to keep the run alive as a blue candidate, which does not change the claimed set. `dead_seed(w)` holds exactly when the subgraph gives w no out-arc [S3 review].
- **Label identity, checked:** 130 pairs on Aix, 65 on aix70, 433 on quebec / bike, every N2 pair, 58 new pairs on 29 cells, 45 against the real melon class [S2]; the scratch build with frontier reproduced the real build without it end to end (§4.6) [S2 design]; on and off on 9 cells with the new formula [S3 review]; `nofrontier` against the real header at every num_mus in fuzz [S3 fuzz].
- **Port trap:** once dead seeds are erased, `mu_fiber.size()` is no longer the fiber size, which the old "no growth" test and per-μ counters used. The landed header counts `fiber_gain` per pass instead [S2 judge, §10.1].
- **Saving.** Counters 0.42-0.57x of plain on Aix and N2, 0.56-0.70x on quebec, 0.19-0.38x on bike (top:3:10@asc) [S2 judge]. gecot's own preprocessing time, dec9 against rule (which also lacks the final 0): 0.42x on aix70 B=2, 0.78x on quebec B=2133, 0.23x on bike_mid B=155 [S2 timing]. Without frontier, 21 points take 3.24x as long on aix70, 2.92x on aix_N2, 2.98x on aix40, 1.69x on quebec and 7.6x on bike_mid (osweep, 4 threads) [S2 timing]. Frontier is what makes 21 points affordable.
- On fuzz-sized graphs, frontier cuts heap pops by 35%, but its own rescans add 8-10% arc scans; a per-vertex count of open out-arcs would make it free [S2 theory]. On real instances the scan is a net win: 0.29 s against 0.44 s on quebec B=2133 N=20, 1.14 s against 2.71 s on aix30 B=1 N=20 [S3 review].

### 8.2 redbreak (dropped)

- **Strong loop, exact** [S2 theory, proved]: if at the start of a pass every out-arc of u other than uv enters the fiber, run that pass and stop; later passes add nothing.
- **Useless loop, literal version, wrong**: stopping once red cannot leave v loses v whenever a smaller multiplier follows (counterexample G4, 3 vertices) [S2 theory]. Measured: it lost 245 useless labels in 32 of 224 runs, all on quebec and all with μ = 0 not first [S2 build]; on fuzz, in desc order, 3,515 (cur10) and 7,514 (top3p10) labels, falling below the μ = 0-only labels [S2 theory].
- **Exact useless version (rb3):** after the stuck pass, test v alone at each remaining multiplier, with D_v = min over in-arcs (w, v) ≠ uv, w ∈ F ∪ {u}, of D(w) + l^+(wv); claim v iff D_v + P(μ) < λ^μ(uv). Label-identical in 224 of 224 runs; its analytic branch gave v to blue 307 times, only on quebec [S2 build].
- **Alone it saves, but frontier subsumes it.** Without frontier, redbreak (exact) cuts S+U pushes by 21.0 / 8.4 / 20.2 / 22.0% on aix30 / quebec / bike_mid / aix70 (top:3:10@asc) and skips 11,700 / 1,054 + 1,278 / 2,278 + 2,250 / 60,660 passes (strong + useless; Aix has no useless skips); both flags together save about what frontier alone does [S2 build]. On fuzz, the exact strong redbreak alone (rb1) saves 7% of passes and rb3 17% [S2 theory].
- **Not worth it.** On top of frontier the counters move by 0.9925-0.9999x [S2 judge], wall time by 0.997-1.002x [S2 timing]; on Aix and N2 the useless loop never isolates v. About 30 lines. Dropped.
- The landed `if(fiber_map[v]) break;` is not redbreak: it stops the useless loop once v is blue, and is exact (§1.2).

### 8.3 Counters, wall time, osweep and gecot

- Counters miss frontier's rescans and the O(V) reset per pass. Inside gecot they track wall time to within about 12%: 5.84 ns per counted operation for rule, 5.34 for dec9, 5.19 for dec20 on aix70 [S2 timing].
- osweep labels slower than gecot's own preprocessing: aix70 B=2 rule 29.30 s in gecot against 35.73 s in osweep (+22%), dec9 12.28 against 13.33 s (+9%), dec20 21.51 against 23.04 s (+7%); quebec +10-21%; bike_mid up to +48%. The gap is larger for the plain variants, so osweep overstates frontier's saving (0.37 / 0.71 / 0.20 against gecot's 0.42 / 0.78 / 0.23). Ratios among frontier variants agree (dec20 / dec9: 1.75 in gecot, 1.73 in osweep) [S2 timing].
- Labelling repeatability on a quiet machine: under 1.3% between repetitions; thread scaling 4 → 16 threads (8 cores with SMT) 2.6-2.9x on aix70, aix_N2 and aix40, 2.1-2.7x on bike_mid, 1.4-2.5x on quebec, where fixed overhead dominates (μ = 0 alone goes from 46 to 32 ms) [S2 timing].

## 9. End-to-end timing

### 9.1 [S1]: the three fixes and the rule build

flow_mip `--big-m tight -v`, Gurobi, header df9a8581 (no frontier, no final 0), no `--max-threads` (TBB and Gurobi at 16 threads), 2 repetitions. The CSV has 204 rows over 22 instance/budget points, the control included (the agent's summary says 203 runs and 26 points; neither count reconciled). Every variant reached the same optimum at every point that finished (≤ 2.07e-16 relative). Mean total time (the CSV's mean_ms) at every timed point, with variables for aix70 B=2:

| Cell | nm0 | nm1 | nm10 | rule |
|---|---|---|---|---|
| aix30 B=1 | 0.24 s | 0.25 s | 0.47 s | 0.46 s |
| aix30 B=1.72 | 0.52 s | 0.52 s | 0.71 s | 0.75 s |
| aix30 B=4 | 1.80 s | 1.36 s | 1.92 s | 1.83 s |
| aix40 B=1.42 | 1.31 s | 1.14 s | 1.72 s | 1.68 s |
| aix40 B=2 | 2.00 s | 1.25 s | 1.81 s | 1.79 s |
| aix40 B=4 | 1.85 s | 2.24 s | 2.81 s | 2.64 s |
| aix70 B=1 | 11.9 s | 9.8 s | 18.2 s | 17.0 s |
| aix70 B=2 | 30.5 s (426,109) | 13.5 s (370,671) | 22.3 s (369,264) | 22.0 s (368,889) |
| aix70 B=3 | 40.1 s | 20.8 s | 27.4 s | 28.3 s |
| aix70 B=5 | 87.6 s | 101.7 s | 96.0 s | 93.9 s |
| aix70 B=10 | 158.3 s | – | 169.3 s | 171.8 s |
| bike_mid B=78 | 1.91 s | 1.09 s | 0.99 s | 0.92 s |
| bike_mid B=155 | 2.49 s | 2.19 s | 2.10 s | 1.68 s |
| bike_mid B=200 | 2.85 s | 2.50 s | 2.66 s | 2.01 s |
| bike_mid B=311 | 2.94 s | 3.08 s | 2.41 s | 2.56 s |
| bike_mid B=777 | 22.2 s | 22.0 s | 42.8 s | 26.4 s |
| bike_small B=62 | 91 ms | 100 ms | 89 ms | 64 ms |
| quebec B=1000 | 2.72 s | 2.70 s | 2.36 s | 2.34 s |
| quebec B=2133 | 4.04 s | 4.00 s | 3.76 s | 3.94 s |
| quebec B=4267 | 7.74 s | 7.86 s | 7.63 s | 7.30 s |
| quebec B=8533 | 8.44 s | 8.54 s | 8.11 s | 8.15 s |

- **nm3** (5 passes) labels exactly what nm1 labels on aix70 B=1 and B=2 and on aix30 and aix40 at every budget, but labels for longer; it was not run at aix70 B ≥ 3. **Control:** the rule binary run with `--nb-mus 0` on aix70 B=2 took 30,249 ms against 30,391 / 30,544 ms for the real binary, so the two builds time alike.
- nm1 ({0, min_mu, max_mu}, 3 passes) was the fastest Lagrangian variant on aix30 and aix40 at every budget and on aix70 up to B=3; at aix70 B=5 it was the slowest (101.7 s against 96.0 s for nm10 and 93.9 s for the rule). Each pass cost about 0.93 s on aix70, so the rule's 11 passes (10.2-11.0 s) left it 7-8.5 s slower than nm1 at B=1-3 for a model only 0.1-0.5% smaller.
- The model's share removed shrinks with the budget: aix70 24.2 / 13.4 / 6.7 / 1.1 / 0% at B = 1 / 2 / 3 / 5 / 10 (rule against nm0); bike_mid 19.8 / 16.9 / 7.0 / 5.5 / 0.3% at B = 78 / 155 / 200 / 311 / 777.
- **When the saving is cancelled.** [S1 timing] put the threshold at labelling above about 10% of the solve time; its own CSV does not bear that out (at aix70 B=2 the rule's labelling is 90% of its solve phase, 10.4 against 11.6 s, and it still saves 28%). Read from the CSV, two kinds of cells lose:
  - where the model shrinks by about 1% or less: aix70 B=5 (1.1%) and B=10 (0%), aix40 B=4 (0.2%), bike_mid B=777 (0.3%; nm1, with nm0's model, ties). The cut is not sharp: nm1 is 1.05x nm0 at bike_mid B=311 with a 1.3% smaller model;
  - where nm0 itself is short against the labelling: nm10 and the rule lose at aix70 B=1 (rule 17.0 against 11.9 s, model 24% smaller), aix30 B=1 and 1.72 and aix40 B=1.42 (models 9-21% smaller); nm1 wins or ties there except at aix30 B=1 (0.25 against 0.24 s).
- At aix70 B=5, nm1 was 1.16x nm0 with a 1.1% smaller model (the [R] critic's slowdown reproduced). bike_mid B=777: nm10 1.93x nm0 with 16 fewer columns, the rule 1.19x.
- biorevaix_N2 at 0.5% (11,276,591 variables, 6,385,939 constraints) died after 247 s with Gurobi "Out of memory" under the agent's own 14 GB `ulimit -v` (RSS 10.5 GB); not retried.

### 9.2 [S1 critic]: Gurobi seeds

Seeds 0-4 through `gurobi.env`, Threads 4, `--max-threads 4`, rotated order, identical optima:
- bike_mid B=155: rule 1.91-2.05 s, nm10 2.40-2.53 s, nm0 2.98-3.47 s; rule fastest in 5 of 5 seeds, ranges disjoint; nodes rule 267-348, nm10 358-490.
- bike_mid B=200: rule 2.05-2.21 s, nm10 2.86-3.23 s, nm0 3.30-4.05 s; rule fastest in 5 of 5; nodes rule 176-217, nm10 354-491.
- bike_mid B=78: a tie (rule 1.06-1.12 s, nm10 1.08-1.18 s).
- quebec B=2133 solves at the root node in every run, so the seed changes nothing; nm10 3.78-3.86 s, rule 3.94-3.97 s, nm0 4.07-4.16 s. The rule build run with `--nb-mus 10` (197,153 columns) took 3.56-3.60 s: a column difference under 1% moves quebec's root solve by about 10%, so quebec's differences between grids are not grid quality.
- The same rule build with `--nb-mus 10` (top:3:11) on bike_mid B=155: 15,428 columns (the rule 15,794) in 1.75-1.90 s, so 11 points instead of 10 keep the bike gain.
- So the 3-decade grid's end-to-end win over nm10, at that stage, was measured on bike_mid B=155 and B=200 only (and on bike_small B=62, 64 against 89 ms, where start-up dominates runs under 0.1 s).

### 9.3 [S2]: labelling and end to end at 4 threads

Labelling, osweep `--algo real --no-gt`, 4 threads, medians of 3 interleaved repetitions on a quiet machine [S2 timing]:

| Cell | rule (T10) | dec9 | dec10 | dec20 | dec20 without frontier |
|---|---|---|---|---|---|
| aix70 B=2 | 35.73 s | 13.33 s | 14.22 s | 23.04 s | 74.55 s |
| aix_N2 B=53.72, 10% sample | 31.72 s | 12.82 s | 13.71 s | 22.69 s | 66.35 s |
| aix40 B=2 | 2.539 s | 1.027 s | 1.089 s | 1.699 s | 5.06 s |
| quebec B=2133 | 184.6 ms | 130.2 ms | 136.2 ms | 200.4 ms | 338.0 ms |
| bike_mid B=155 | 139.1 ms | 28.3 ms | 29.0 ms | 37.0 ms | 281.8 ms |

End to end, flow_mip `--big-m tight --max-threads 4`, Gurobi seeds through `gurobi.env` (Threads 4), median total time; nm0 and nm10 are the binary at df9a8581, dec9 / dec10 / dec20 the scratch design build (old grid formula, frontier, final 0) [S2 timing]:

| Cell (seeds) | nm0 | nm10 | rule | dec9 | dec10 | dec20 |
|---|---|---|---|---|---|---|
| aix70 B=2 (2) | – | – | 40.7 s | 23.0 s | 23.5 s | 31.3 s |
| quebec B=2133 (5) | 4.01 s | 3.74 s | 3.88 s | 3.85 s | 3.45 s | 3.44 s |
| quebec B=1000 (5) | 2.67 s | 2.40 s | 2.36 s | 2.32 s | 1.99 s | 2.34 s |
| bike_mid B=155 (10) | 3.27 s | 2.43 s | 1.91 s | 1.84 s | 1.74 s | 1.75 s |
| bike_mid B=110 (10) | 4.40 s | 2.57 s | 1.83 s | 1.76 s | 1.83 s | 1.83 s |

- aix70 B=2: preprocessing is 53-72% of the total (rule 29.3 s, dec9 12.3 s, dec10 13.2 s, dec20 21.5 s); Gurobi's own time barely moves (10.8 / 10.2 / 9.7 / 9.2 s). dec20 costs +8.2 s (+36%) over dec9 for 36 fewer variables (0.01%). On large graphs, labelling time decides.
- [S1]'s rule took 22.0 s at aix70 B=2 and [S2]'s 40.7 s: [S1] ran at 16 threads, [S2] at 4.
- On bike_mid, dec10 and dec20 build the same model (same variables and node counts per seed).

### 9.4 What model size explains

- **On small graphs, not the timing differences.** The headline "dec20 is 11.4% faster than rule at quebec B=2133 and 8.4% at bike_mid B=155" does not come from the smaller model [S2 critic]: at quebec B=2133, nm10 beats rule (3,738 against 3,883 ms) with 8,646 more variables; at quebec B=1000, dec10 beats dec20 by 15% with 1,137 more variables; at bike_mid B=155, 72 of the 161 ms dec20 gains over rule come from preprocessing alone, and dec20's range overlaps dec9's. quebec and aix70 solve at the root node with about 1% seed spread, so 5 seeds do not sample structural variance.
- **[S1]'s counterexamples** to "fewer variables, faster solve": at bike_mid B=311 the rule has the smaller model (19,323 against nm10's 19,691) and is 6% slower; at quebec B=2133 it removes 11.1% of the columns against nm10's 7.2% and is 4.7% slower; at aix40 B=4 every Lagrangian variant is 1.21-1.52x slower than nm0. nm1 labels nothing on quebec at any of the four budgets (its column count equals nm0's) [S1 timing].
- **What stands** [S2 critic]: frontier makes labelling 1.7-7.6x faster at equal labels; redbreak buys nothing; 21 points (dec20) cost 1.3-1.8x the labelling of dec9. From [S2 timing]: on aix70 the labelling time decides, and by the medians the 3-decade variants (rule, dec9, dec10, dec20) beat `--nb-mus 0` on bike_mid by 1.71-2.51x, nm10 by 1.34x at B=155 and 1.71x at B=110.
- **Not measured:** whether 1-2.5% fewer contracted arcs ever pays on a hard MIP (minutes of solve); no timed N2 end-to-end run [S2 critic].

## 10. The implementation, as landed

### 10.1 The diff

4d4c14c changes 4 files, +195 / −32 lines.
- **Header** (`compute_constrained_strong_and_useless_arcs.hpp`, 80 lines changed):
  - new trailing parameter `bool decades_sweep = true`;
  - options without arcs skipped in both option loops; `max_arc_cost` tracked;
  - `max_mu = min(max_mu, 0x1p62 / max({budget, max_arc_cost, 1.0}))`; under decades, `min_mu = max(min_mu, max_mu / 1000)`;
  - the grid built by index, `min_mu * pow(max_mu / min_mu, k / N)` for k < N and exactly max_mu at k = N, each kept if `mus.back() < mu && mu <= max_mu`;
  - a final μ = 0 under decades if any positive point exists; a trace line "µ sequence", printed with `fmt::join` from the new `#include <fmt/ranges.h>`;
  - `dead_seed` and an `erase_if` over `mu_fiber` before the fiber seeds in each loop (frontier); `w != v` / `w != u` guards; per-pass `fiber_gain` counters replace `mu_fiber.size()`;
  - the useless-loop stop `if(fiber_map[v]) break;`;
  - comments on the cap, the grid formula, the final μ = 0, frontier and the seed order.
- **`mip_solver_base.hpp`:** `bool decades_sweep = true`, passed through.
- **`abstract_mip_interface.hpp`:** the `--nb-mus` help text and `CLI::NonNegativeNumber`; `--mus-sweep` through `add_option_function<std::string>`, `CLI::IsMember({"decades", "full"})`, `default_val("decades")`.
- **`test/preprocessing_test.cpp`:** the helper replaced, 4 includes, the test of §4.5; fuzzy_test unchanged.

### 10.2 The commit message's claims

| Claim | Source | Note |
|---|---|---|
| The grid of N is bitwise a subset of the grid of 2N, so doubling `--nb-mus` never loses a label | [S2 theory], [S2 skeptic], [S3 fuzz], [S3 review] | Proved for the labels (§7.4); grid nesting checked (§6.4) |
| `--mus-sweep full` keeps the range from min_mu, whose first point never adds a label | [S1 sweeps] | Measured, not proved: singleton gain 0 at every budget measured (§5.3) |
| Today's `--nb-mus 10` grid found 0.4-67% less of the gain than three decades | [S2 judge] | Aix 0.4-11%, quebec 6-45%, bike 19-67%, N2 7-42% |
| μ = 0 last loses 9-76% of the gain on quebec and N2 | [S2 judge] | asc0last: 8.7-76.1% |
| The final μ = 0 provably only adds labels, for 6-7% more labelling time | [S2 theory], [S2 timing] | Wall time with frontier, 1.064-1.073 |
| The order of the positive multipliers moves at most 3% of the gain on Aix and N2 and 7% on quebec | [S2 sweeps], [S2 skeptic] | Aix −2.95 to +0.36%, N2 ≤ 0.38%, quebec up to +7.10% on 10 points; on 21 points desc + 0 loses 3.04 pp at aix30 B=2.3, so "at most 3%" is approximate |
| No order is better on both | [S2 judge], [S2 skeptic] | §7.3 |
| Frontier leaves every label unchanged; preprocessing of `--nb-mus 9` 0.42x / 0.23x / 0.78x | §8.1, [S2 timing] | gecot's own times, dec9 against rule, which lacks frontier and the final 0 |
| Cap at 2^62 / max(B, largest cost per arc, 1) | [S4] | |
| `--nb-mus` rejects negative values | [S3 testBuild] | §10.3 |
| Fixes: grid by index, no fiber vertex seeded twice and the useless-loop stop, options without arcs skipped | [S1 verify] | §3.2, A1-A3 and A5 |
| About 200k random instances, both sweeps, `--nb-mus` 1 to 40, assertion and UBSan builds: no invalid label, N within 2N | [S3 fuzz], [S3 critic], [S4] | Approximate: a sum of instance-runs, not distinct instances, over two header states (§4.2); num_mus 1, 2, 9, 10, 20, 40 |
| On 13 cells, aix_N2 included, the models have the predicted sizes; full `--nb-mus 10` builds the previous models | [S3 gate] | 49 runs; 12 cells identical. "Previous" is the df9a8581 build, not 6c59214, whose loop drops max_mu at `--nb-mus 10` on Aix |
| The test checks quebec B=2133, where the multipliers add 29,765 labels | [S3 testBuild] | On the raw instance |

### 10.3 CLI checks

On the build of 80bb8a4e [S3 testBuild]:
- `gecot optimize flow_mip -P` lists `--nb-mus INT:NONNEGATIVE [0]` and `--mus-sweep TEXT:{decades,full} [decades]` with their help texts.
- `--mus-sweep foo`, `--mus-sweep Decades` (case-sensitive), `--nb-mus -1` and a bare `--mus-sweep` exit 1 with a CLI11 error. Before, `--nb-mus -1` silently turned the Lagrangian off [S2 design].
- `--nb-mus 20` without `--mus-sweep` on quebec B=2133: "Min µ: 2.2742e+04, Max µ: 2.2742e+07" and a 23-entry µ sequence with 0 first and last; the table has a final-0 row. `--mus-sweep full`: 22 entries from 4.873e-04 to 2.274e+07, no final 0.
- tree_lp exposes and honours both options (bike_small B=31) [S3 critic]; [S3 testBuild]'s report that they appear only under flow_mip was wrong [S3 review].

### 10.4 The cap fix and the dead default predicate [S4]

- **Change.** max_mu is capped at 2^62/max(B, max_arc_cost, 1) instead of 2^62/max(B, 1), where max_arc_cost is the largest c_o/|A_o| among admissible options. The old cap bounded μ·B but not μ·s_o, so a predicate admitting an option with s_o ≥ 4·max(B, 1) could make the per-arc penalty cast overflow [S3 review finding 2].
- **Crafted probe** (raw, B=1): o0 at cost 1e-10 raises a p = 1e-9 arc to 1, so max_mu sits at the old cap (4.61e18); o1 costs 5 on one arc, so its penalty is 2.31e19 ≥ 2^64 and UBSan reports the cast. Under the new cap max_mu = 2^62/5 = 9.22e17, no report, and brute force finds 0 invalid labels with every unconstrained label kept. Over 1,000 seeded variants under UBSan and 2,000 in Release, old-cap UB occurs exactly when o1's arc cost / (4·max(B,1)) ≥ 1; where it is 2-4x max(B,1), the two caps give different but valid grids (380 labels only under the old, 598 only under the new, of 7,632 pairs).
- **The default predicate is unreachable.** A 3-argument call does not compile under GCC 15 ("couldn't deduce template parameter"): template deduction ignores default arguments, and `compute_strong_and_useless_arcs` has the same problem. Every in-repo caller passes `cost <= budget`, under which s_o ≤ B, so through the repo the cap change changes no label (192,000 label-set pairs identical) and matters only to library callers with a custom predicate admitting s_o > max(B, 1). The comment was reworded to "a predicate may admit arc_cost > budget".

### 10.5 Review findings, applied and left

| Finding | Source | Status in 4d4c14c |
|---|---|---|
| The decades window hangs on one pair (max_mu) | [S3 review 1], [S1 skeptic] | Left: a design caveat, recorded here (§6.6); no guard |
| μ·s_o cast unbounded under a predicate admitting s_o > max(B, 1) | [S3 review 2] | Fixed by the new cap (§10.4) |
| Help: "the 3 decades" → "at most the 3 decades"; say that `--mus-sweep` needs `--nb-mus > 0` | [S3 review 3] | Applied in the help text; at run time `--mus-sweep` is still silently ignored at `--nb-mus 0` |
| Seed order undocumented | [S3 critic gap 1] | Applied: comment in both loops |
| The test's 1e-6 tolerance sits below the numerical floor at other budgets | [S3 review 4] | Left |
| The CLI default lives in two places: `add_option_function` does not run the callback for the default (CLI11 2.7.2), so the struct initializer decides and `default_val` is only what `-P` shows | [S3 review 5] | Left; `->run_callback_for_default()` would remove it |
| "common ratio" trace ≤ 1 or inf in degenerate cases; the final μ = 0 rebuilds a length map identical to index 0; `for(int k ...)` overflows at INT_MAX; the comment says "2 * num_mus" where any multiple holds | [S3 review 6] | Left (nits) |
| The test's pins depend on libm (`pow`, `log`, `log1p`, then truncating casts); tests are disabled on Windows, so the residual risk is another glibc or CPU on Linux | [S3 review 6] | Left (low) |
| Library default `num_mus = 10` gives 11 points, not the recommended 21 | [S2 design] | Left |
| `decades` as the default silently changes what an existing `--nb-mus 10` runs; old logs need `--mus-sweep full` | [S2 design] | Owner's call: decades kept. `full` reproduces the df9a8581 build in every comparison made (§6.4); 6c59214's loop dropped max_mu on Aix at N = 5, 8, 10, 11, 30, 40 and at N=10 on 82-99% of random ranges, so its logs at those N are not reproduced exactly |
| The cap comment ("a blue seed of 2^62 outweighs any path") overclaims whenever B < max(1, largest cost per arc): for 0 < B < 1, and, with a custom predicate, for B below the largest cost per arc, since at the cap the top blue seed is 2^62·B/max(B, max_o s_o, 1) < 2^62 (header line 99) | [S1 verify] (on the df9a8581 cap, for B < 1), code | Left (strength only, never validity) |
| Divisions by a cost-0 share (raw instances) and by `num_mus = 0` at header level | [S1 verify] | Left; library only, labels valid |
| Self-loops push u twice and trip PRE_HEAP in Debug (raw bike_mid has 10) | [S1 verify] | Left; pre-existing, also in the unconstrained header; the CLI never sees self-loops |

Checked and found correct [S3 review, S3 critic]:
- **Grid edge cases.** k=0 gives exactly min_mu and the last point is exactly max_mu; points are deduplicated only when max/min is within a few ulps of 1. min ≥ max gives [0, max, (0)]; max = 0, or no option with arcs, gives [0] with no final 0; an infinite max is capped; min = DBL_MAX gives [0, cap, 0]; N=1 gives [0, min, max, 0]; a header-level N=0 gives [0, max, 0], which only library callers can reach (the CLI routes 0 to the unconstrained function). Grid N is a subsequence of grids 2N and 3N over 3.5M random cells [S3 review].
- **The test's `std::ranges::includes`** needs sorted input: both label maps are sorted before return (header lines 276-279 at 4d4c14c, 269-272 at 80bb8a4e; `compute_strong_and_useless_arcs.hpp:191-195`, from 748d20a) [S3 critic, check 7].
- **The new `<fmt/ranges.h>` include.** spdlog uses the external fmt 10.2.1, already a direct dependency; the header covers `join` in fmt 10 and 11+, and the `{:.3e}` output was checked [S3 review].

## 11. Caveats and open items

- **Not run** [S1, S2 critic, S3 gate, S3 critic]:
  - the N1 instances (unc alone exceeds 130-150 s at 2%); aude beyond showing no gain; biorevaix_N2 beyond 0.5% in [S2], and never timed (out of memory under a 14 GB cap);
  - N2: any timed end-to-end solve (the landed `--nb-mus 9` ran through gecot on the whole aix_N2 graph only to a 1 s solver limit, §4.6); on the whole graph, the final 0's own gain (nothing ran without it), 21 or 41 points and a 4th decade; budgets of 5-20%;
  - aix70: B = 3 / 5 / 10 end to end in [S2] ([S1] timed them at 16 threads, before frontier); a 4th decade; timing or contracted-graph size for 41 points anywhere (on aix70 B=2, 41 points add +99 / +125 labels over 21 [S3 gate]);
  - end-to-end timing on aix30 and aix40 of the landed build or the design build (the landed build's model sizes were gated there, §4.6);
  - `--mus-sweep full` at N ≠ 10 gated against the harness (it ran at N=20 on quebec B=2133 and at N=9 under tree_lp, with trace checks only, §10.3);
  - the √2-anchored grid; hard MIPs (minutes of solve).
- **In-sample.** D = 3 and the point counts were tuned and scored on the same seven families; no instance was held out. Nesting limits the damage of a wrong N, but 3 decades is a tested floor, not a proven one [S1 critic, S2 judge].
- **Sample-only results.** Order conclusions for aix70 B = 3 / 5 / 10 and every N2 order conclusion (0.5% included) rest on 10% arc samples, seed 1 apart from [S2 skeptic]'s few seed-2 cells (§7.7). N2 shares in [S1] are relative to top:2:5, not dense:40.
- **Overhead without gain.** At budgets where nothing is gained, dec9 labels 4.8x and dec20 8.4x as long as `--nb-mus 0` on aix70 (§2.4); nothing detects such budgets. The per-arc slope-lemma skip is untested on real instances.
- **The decades anchor** (§6.6): one cheap option on a near-zero-probability arc can push the window off the useful band; full is better there. The median guard was never implemented or run.
- **Blue ties**: implemented on branch `blue-ties` (§5.4). Open: whether §2's settings still hold with them (blue `--nb-mus 0` beats red `--nb-mus 9` on aix70), one or two `nextafter` steps, and the coverage limits listed there.
- **Fixpoint with the idempotence skip** (§7.4): not implemented; pays only on quebec.
- **Thesis errors** to fix in chap_preprocess.tex (§3.3).
- **Timing confounds.** [S1] ran at 16 threads, 2 repetitions of deterministic solves, the same variant order in every repetition for the small cells, `-v` on, under load 4-10 [S1 critic]; [S2] ran at 4 threads on a quiet machine but with 2 seeds on aix70. Root-node solves (quebec, aix70) barely sample Gurobi's variance.
- **Aix data.** Every Aix number uses the working-tree CSVs with the owner's uncommitted edits (aix40 arcs, options and vertices; aix70 arcs and vertices; aix30 vertices; generate_instance.py), dated 2026-11-02 18:41-18:44, before every run; they cannot be reproduced from the committed tree [S1 critic].
- **Instance files.** The committed Aix/N1.json, Aix/N2.json and BioRevAix/N1.json fail the current schema (a `quality` column where source_quality / target_quality are expected), and BioRevAix/N2.json does not exist; the harnesses used adapted copies mapping `quality` to both [S1]. [S3 gate]'s gecot run on aix_N2 used the same adapted copy as [S1]'s sweep harness; its log names the file and reports 583 options.
- **Resolution.** Every sweep and timing on real instances ran at 1e-8, except a quebec singleton probe at 1e-6 and 1e-10 [S1 sweeps] and an aix30 B=1.72 probe at 1e-3, where max_mu/min_mu = 86 and the decades window falls back to [min_mu, max_mu] [S1 skeptic]; the fuzz also used 0.3 and 0.5 (§4.2). quebec's θ_max is set by the clamp, so coverage there moves with `--resolution` [S1 critic]. At 1e-8 and 1e-12 the cap never binds on the 11 instances and its two claims are proven (§3.2, A4); at resolution 1e-15 the cap binds on quebec, Aix N1 / N2 and BioRevAix N1 / N2, and "2^62 outweighs any path" and "no uint64 sum wraps" are no longer proven there (the old code was already UB at those points) [S1 verify].
- **Verification gaps.** No Debug build of gecot and no ctest in Debug; no ASan or TSan; the test cannot pin the grid formula (m5); its tolerance is thin at other budgets (§4.5).
- **cli_parity.** `test/cli_parity.sh` cases R05 and R06 (`flow_mip ... 10 --nb-mus 3`) and E33 (`tree_lp ... 10 --nb-mus=2 --resolution=1e-6`) will show DIFF against a binary older than 4d4c14c, as expected: decades is now the default [S3 review].
- **Harness drift.** Rebuilding the [S2] osweep against the landed header gives spurious `--verify` mismatches (it passes 6 arguments and inherits `decades_sweep = true`), and its README's copy-sync check now prints the expected 60-line delta [S3 gate].
- **Owner decisions left open:** the default N (20 vs 9) and whether a size-dependent default is wanted; whether the library default `num_mus` should become 20; a guard or documentation for the max_mu anchor; the fixpoint; the `nextafter` steps (§5.4).

## 12. Appendix: reproduction

All harnesses below lived in a temporary scratch directory, which is not kept. They are described here so they can be rebuilt.

- **Fuzz harness** (fuzzv in [S1], fuzz2 in [S3], plus the theory fuzz of [S2] whose pass code is generated from the live header). It includes the real header, generates random small instances from a seed, enumerates every feasible option set, and checks labels as in §4.1. Control variants are produced by a script from the live header with an exact replacement count asserted per edit. It replicates the grid to compare against the header's "µ sequence" trace line. Seeds are reproducible within a build; a run reports counts per configuration and the first bad seed of every control.
- **sweep** [S1]. Parses an instance, reformulates it with `trivial_reformulate_instance(raw, budget)` as `prepare_instance` does, labels each case with the real unconstrained header (`unc`) or with a copy of the constrained header whose positive multipliers come from a grid spec, then calls `compute_contracted_generalized_flow_graph` for every target as `flow_mip.hpp` does. One CSV row per spec: labels, gt_arcs, gt_vertices, label time, the predicted flow_mip columns. Specs: `unc`, `cur:N`, `top:D:N`, `dense:N`, `theta:a,b,...`, `list:`, `old:N` (the loop before the fixes), and a `blue:` prefix for blue-wins ties. Modes: `--singletons K` (one {0, μ} run per grid point), `--zero-scc`, `--verify`, `--no-gt`, `--raw`.
- **osweep** [S2]. A renamed copy of the header in which each pass runs the header's own statements, plus a renamed copy of melon's competing_dijkstras that counts seeds, pushes, pops and promotions by colour (thread-local, identical at 1, 4 and 8 threads). Spec grammar: `unc`, or a per-loop spec `set[@order]{+modifier}`, or `S=<spec>|U=<spec>` to give the two loops their own. Sets: `none`, `cur:N`, `top:D:N`, `dense:N`, `halve:N`, `list:`. Orders: asc, desc, asc0last, desc0last, no0asc, no0desc, bisect, zigzag, `idx:i,j,...` (explicit sequence; repeats and omissions allowed, so `idx:0,1,...,n,0` is 0 / asc / 0), `perm:SEED`, `perm0:SEED` (splitmix64 Fisher-Yates). Modifiers: `cycle:K`, `fixpoint` (cap 50), `stopK`, `frontier`, `redbreak` (exact), `redbreakN` (the literal, lossy one). Options: `--algo counting|real`, `--melon-check`, `--verify`, `--diff-ref K`, `--positions FILE` (one row per pass), `--arc-sample F --sample-seed S`, `--dump`, `--raw`. Labels are compared by an FNV-1a hash of the sorted (vertex, arc) list. Derived builds: osweep_idem (idempotence skip) and osweep_canon (an independent Dijkstra on G−uv per arc) [S2 critic]; osweep_exp (the new grid formula, `--grid-form old|exp`, `--seq-digits 17`, a `--verify` that passes the sweep flag explicitly) [S3 gate].
- **Fidelity rules** every run relied on:
  - the harness's header copy must diff empty against the real header (or show only the documented delta), checked before and after each batch;
  - `--verify`: cur:N@asc hash-equal to the real header, and unc to `none@asc` run through the copy;
  - the counting class hash-equal to the real melon class (`--melon-check`);
  - internal identities: cycle:1 = plain, fixpoint = cycle:50, every saver flag = plain, an arc sample = the restriction of the full run, per-loop splits carry each loop's own hash;
  - gecot's variable count = predicted columns + the option count of the reformulated instance;
  - outputs other than label time identical at 1, 4 and 8 threads.
- **Practice.** One build at a time under a lock; every run wrapped in `ulimit -v` and `timeout -k`; 4 threads for sweeps; wall times on a shared machine are indicative only, and the deterministic counters carry the cost comparisons.
