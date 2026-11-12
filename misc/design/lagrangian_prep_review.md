# Review: gecot Lagrangian preprocessing (compute_constrained_strong_and_useless_arcs.hpp)

Date: 2026-11-05. Code reviewed: gecot `prep-mip-tight-big-m` at 6c59214.

The review was a workflow of 50 agents:
- five lenses: thesis conformance, validity, implementation, the M3 gap, and an empirical lens;
- 44 adversarial verifiers;
- one completeness critic.

Raw output: /tmp/claude-1000/-home-plaiseek-Projects-mippp/f1325762-f5cf-40e5-a297-798929dc8673/scratchpad/lagprep/review_dump.md (findings, every verdict, and the critic) and /tmp/claude-1000/-home-plaiseek-Projects-mippp/f1325762-f5cf-40e5-a297-798929dc8673/scratchpad/lagprep/verdicts_rest.txt.

Caveat: every Aix number below uses the working-tree Aix CSVs, which carry the owner's uncommitted edits. All runs used the same files.

## Verdict

**Validity: no wrong label found.**
- **Fuzzing.** Four independent brute-force harnesses compiled the real header and checked every recorded label against every feasible option set. Two of them also compared F_t on the original graph with F_t on the contracted G_t, end to end:
  - empirical lens: about 20k reformulated + 10k raw random instances, 627k + 307k scenarios, 6.4M end-to-end checks;
  - validity lens: about 45k instances;
  - thesis lens: 14k instances;
  - implementation lens: 5.3k instances, about 309k end-to-end checks.
- **Result.** 0 violations in the integer-length model, and 0 end-to-end mismatches beyond 1.5e-9 relative.
- **Negative controls.** All three were caught heavily. Dropping the blue penalty in the strong loop gives 1.36M violations; doubling the shares gives 6.3k; dropping the blue penalty in the useless loop gives 0.88M.
- **Real instances.** Full enumeration on aix30_4, aix40_2 and aix70_2 (one target in eight on aix70_2, 8104 scenarios) found 0 pairs off by more than 1e-9. The CLI optimum is identical for --nb-mus 0 and 10 on Aude B=3, Aude B=5 and Aix30 B=4.

**Strength: the multiplier grid almost never tests its best multiplier.** I reran flow_mip on aix70 at B=2 with --big-m tight, once per setting:

| --nb-mus | Multipliers actually run | Variables | Lagrangian prep | Total | Optimum |
|---|---|---|---|---|---|
| 0 | (unconstrained) | 426,109 | n/a | 32.2 s | 2253080.0526475124 |
| 1 | 0, 2, 1.7185e7 (= max_mu) | 370,671 | 2.9 s | **13.3 s** | 2253080.0526475124 |
| 10 | 0, 2 … 3.481e6 (max_mu dropped) | 408,739 | 10.4 s | 33.1 s | 2253080.052647512 |

Logs: /tmp/claude-1000/-home-plaiseek-Projects-mippp/f1325762-f5cf-40e5-a297-798929dc8673/scratchpad/lagprep/mycheck/aix70_B2_nm{0,1,10}.log. They match the critic's repeated runs: N=0 30.0/32.4/32.5 s, N=1 13.3/13.3/13.5 s, N=10 32.4/32.2 s.

The gain depends on the budget (critic's runs, 2 repeats each, not rerun by me):
- B=5: N=1 is 12% **slower** (97.1 s vs 86.3 s), although the model is 1.1% smaller.
- B=10: N=1 labels nothing more; the model is identical and the run is 2.9 s slower.

## Before testing idea 0

Do not test --nb-mus 10 as is: it drops max_mu, so it measures a broken grid. Either:
- use --nb-mus 1 now as a proxy for the fixed grid. On Aix its grid is exactly {0, min_mu, max_mu}, and min_mu adds nothing;
- or fix the grid first (fix 1 below).

## A. Code defects (all verified; none fixed)

| # | Defect | Effect | Reach | Fix |
|---|---|---|---|---|
| A1 | **The grid drops max_mu.** Lines 92-99: `for(mu = min_mu; mu <= max_mu; mu *= ratio)`. Rounding in pow and in the chained products lands a few ulps either side of max_mu. On Aix it is dropped at N = 5, 8, 10, 11, 30 and 40, and kept at N = 1-4, 6, 7, 9, 12 and 20. Over random ranges at N=10 it is dropped 82-99% of the time. | Strength only. --nb-mus is non-monotone: N=10 is beaten by N=1 on Aix. On aix70_2, the shipped grid gets about 31% of the reduction that adding the single point max_mu gives (Σ G_t arcs 399,223 vs 359,748; unconstrained 416,593). | Every --nb-mus run. The function default `num_mus = 10` (line 26) is one of the dropping values. | Build the grid by index, with the last point exactly max_mu. |
| A2 | **Endless grid loop**, until bad_alloc or the OOM killer. Triggered when the ratio rounds to exactly 1 (max_mu/min_mu within a few ulps of 1, e.g. costs 0.3 over 3 arcs vs 0.1 over 1), or by an improving cost-0 option (ratio = inf). | Crash; never a wrong result. | CLI, with --nb-mus ≥ 1 and degenerate data. Easy at a coarse --resolution: at ≥ 0.466 test/instances/worst hangs; at 0.1, uniform unit costs with gains of 0.5→0.55 hang. Cost-0 options only reach it through the library, because trivial_reformulate drops them. | The A1 grid, built only if `max_mu > min_mu && isfinite(max_mu)`. |
| A3 | **Duplicate heap seeds.** Lines 155-158: v in the strong loop. Lines 206-209: u in the useless loop, and v seeded red then blue. This breaks melon's PRE_HEAP precondition (competing_dijkstras.hpp:184). | A Debug build aborts on the first arc with ≥ 2 multipliers; preprocessing_test's fuzzy_test also aborts under gcc15_c++26_debug. Release is provably harmless: the duplicates have equal priorities, arcs into fiber vertices are filtered so they are never relaxed, and the blue-candidate count stays balanced. | Every --nb-mus ≥ 1 run in Debug. | Skip w == v (strong) and w == u (useless). In the useless loop, stop once v is in the fiber: red then has no source and everything u reaches is already useless. |
| A4 | **mu·B ≥ 2^64**: the uint64 cast (line 45) gives a budget penalty of 0 (UINT64_MAX with -march=native). | **Invalid labels and a wrong optimum**, reproduced end to end with the shipped CLI. | Needs cost ratios of about 1e10 or a tiny --resolution. Real instances are 7-11 orders of magnitude below (Quebec 4.5e11, aix70 B=5 8.6e7). | Skip any mu with mu·B ≥ 2^62: such a mu cannot add a label. |
| A5 | **UB cast for options with no arc in the case**: c/0 = inf, then 0·inf = NaN or mu·inf, cast at line 111. | The value is never used. GCC's UBSan only flags it with -fsanitize=float-cast-overflow (clang's -fsanitize=undefined does). | Every --nb-mus ≥ 1 run with vertex-only options, options that only touch another case, or options whose arcs fall inside a contracted SCC. | `continue` on empty `arcs_options[i]`, before lines 80 and 109. |
| A6 | **The bottom of the grid is wasted.** min_mu = 1/max share is a one-unit (1e-8 nat) penalty, so 5-8 of the 10 positive points add nothing: 5/10 on aix70_2, 7/10 on Aix30 B=4, 6/10 on Quebec. | Time only. | All runs. | See "Grid shape" below. |
| A7 | **No test exercises the function on an affordable scenario.** fuzzy_test checks the all-options scenario, which lies outside X^β for the constrained branch. The constrained branch is wired only to flow_mip and tree_lp. | Coverage. | n/a | Turn the fuzz harness into a gtest: brute force over X^β plus the end-to-end F_t check. Add crafted cases for A1, A2 and A4. |

**Grid shape**, where the verifiers disagree. The useful window depends on the instance:
- **Aix:** the top step of the grid, [max_mu/5, max_mu].
- **Quebec:** 2-4 decades below max_mu (gains at μ = 581, 8174, 1.15e5). The p < 1e-8 clamp inflates max_mu there: θ_max = max_mu·log1p(1e-8)·B is 227, while counts stop changing near θ ≈ 30.

"Concentrate everything near max_mu" therefore fails on Quebec. The safe change is to always include max_mu and spread the other points over about 2-3 decades below it. m3gap's grid, 10 points geometric over [max_mu/100, max_mu], gave the most arc reduction of the grids tried at the same labelling cost: aix70_2 −13.8% vs −4.2% for the shipped grid, aix40_2 −9.1% vs −3.0%, Quebec 134,218 vs 135,809 arcs. Solve times for the alternative grids were never measured.

Use fewer points than 10: preprocessing time is what cancelled N=10's gain. On aix70_2, 5 top points ≈ 10 top points (359,456 vs 359,303 arcs) at about 60% of the labelling time (10.5 s vs 17.1 s), and on Aix N=1 already captures nearly all of it.

## B. Conformance to the thesis (chap_preprocess.tex 319-567)

Conforming and valid:
- **The μ=0 pass is exactly Algorithms strong and useless.** It gives 0 differences on Quebec, Aix30 and Aix40.
- **T** starts from F(u,v) and grows over increasing μ, as line 508 prescribes.
- **λ^μ generalisation.** λ^μ_a = min(l^+_a, min_i l^i_a + ⌊μ c_i/|A_i|⌋) generalises λ^μ to options covering several arcs and to several options per arc. It is valid because a simple path holds at most |A_i| arcs of option i, and the floored shares sum to at most ⌊μβ⌋. min_mu is an addition to the thesis, and a sound one.
- **T handling.** The code removes arcs into T for both colours and re-seeds T at stored distances. The thesis states two other versions: the graph H, and the printed algorithm, which only bars red. All three give identical output on every random and real instance. The stored distances are never stale: red ≤ blue on every arc, so a blue claim always lands at l^+_{uv} + d^+_{G−uv}(v,w) + ⌊μβ⌋. My preliminary hypothesis 3 holds; IMPL-8's "stale distances lose strength" was refuted.
- **Useless loop, source u.** u is a contestable blue source at ⌊μβ⌋, where the thesis fixes u blue at 0 and puts μβ on u's out-arcs. The two agree when lengths are positive. With zero-length cycles the code's version is the valid one, and the thesis version produced invalid labels (1,547 in 3,000 random instances).

**The one real deviation (TC1/LP5): the strong loop lets red win ties** (useless_arc_default_traits, lines 135-138). The thesis lets blue win (lines 126 and 486-488).
- **Same in N=0.** The unconstrained compute_strong_and_useless_arcs.hpp:145-148 has the same rule, so every flow_mip run is affected, not just --nb-mus.
- **Deliberate.** Commit 83729a6 (2023-06-27) switched both strong loops while debugging the contraction.
- **Load-bearing.** On raw Aix 30x30, which has 10 zero-length SCCs, blue ties break 2464/2464 end-to-end checks. A self-loop guard still leaves 2450 broken. A second reason: floating-point rounding can make Σ ⌊μ c_i/|A_i|⌋ along a path exceed ⌊μβ⌋ by one unit, and the strict rule absorbs it. This was never observed.
- **What blue ties would gain.** On the reformulated instances, blue ties were valid in every check (0/987 on Aix30 at 10%, 0/567 exhaustive on Aix30 at 2%, 0/858 on Aix40). They give +28-32% strong labels on Aix (aix70 B=10: 2,037,687 vs 1,593,378), only +0.3% on Quebec, and about 16% more contracted sources.
- **Guard needed to switch.** There must be no nontrivial SCC among arcs of integer length 0 under λ^0. The reformulation only contracts SCCs of exact p == 1 base arcs, so the check must use the rounded lengths. It currently finds 0 such SCCs on reformulated Aix30, Aix40 and Quebec, and 10 on raw Aix30.
- **Not measured:** the effect on solve time.

## C. Thesis text errors

1. **algo:beta_useless.** At line 533 and in its proposition (566-567), T ⊆ 𝔽^β must read T ⊆ 𝕎^β. Read literally the proposition is false: a strong t in T is returned as useless. Line 566 even says "T ⊆ 𝔽^β(u,v) of the β-useless vertices". W_{μ,T} is never defined, and the proposition has no proof; the file ends at its \end{proposition}.
   - "T ⊆ W_{μ,T}" also fails when v ∈ T: v is seeded red at 541-542 despite the w ∉ T guard at 554. It fails as well for a T that is not closed under blue predecessors.
   - Minimal fix: return T ∪ {blue vertices}. Optionally add the shortcut "v ∈ T ⇒ everything reachable from u".
2. **algo:beta_strong.**
   - The initialisation loop (477-480) colours every out-neighbour of u red with no w ∉ T check, so a vertex of T can end up red. The proposition "T ⊆ F_{μ,T}" (504-506) and "at least T" (508) are false for the printed algorithm. The outputs stay valid.
   - H (line 363) bars both colours from entering T; the algorithm bars only red. So "amounts to running Algorithm strong on H" (466) is inaccurate. Both are valid.
3. **Line 508** says "The use case for Algorithm~\ref{algo:strong}"; it should be algo:beta_strong.
4. **Positive lengths.** The assumption is stated in the general part (lines 8, 18, 50, 235, 288), but the constrained section drops it: the CSPP box at 324 has l_a ≥ 0, and lines 348, 505 and 566 only say l^- ≤ l^+.
   - What the algorithms need is "no zero-length cycle in any feasible scenario".
   - The thesis applies them to −log π, which is 0 at π = 1.
5. **Proof of lemma F_mu_T_in_F_beta (434-457).**
   - It has a quantifier slip, and '=' where '≤' is meant.
   - With zero lengths, minimality needs a (distance, hops) tie-break.
   - The "trivial" T ⊆ F_{μ,T} fails if u ∈ T, which can only happen through a zero-length cycle.
6. **μ range (513-517).** It divides by c_a, which line 324 allows to be 0, and "above this value λ^μ_a = l^+_a" is false for c_a = 0. Also state that the endpoint max_mu is included, and that it dominates every larger μ: λ is saturated there, while the bound d^+ − μβ keeps decreasing.
7. **Parallel arcs.** The initialisation loops of algo:strong and algo:beta_strong assume a simple digraph. That is consistent with the definition in chap_tools.tex:70, but gecot's SCC contraction creates parallel arcs. The code is immune: it has no initialisation loop, and settle_red_prefix relaxes every parallel copy.

## D. The M3 refinements, one by one

| M3 element | In the Lagrangian preprocessing | Gain if ported |
|---|---|---|
| Shares c_j/k_j, k_j = min(\|T_j\|, \|H_j\|) | Uses c_j/\|A_j\|. Porting is valid in both loops. The useless-loop proof needs one extra step: the walk uv·Q can revisit u, and that case is settled by the zero-cycle argument. | **0**: \|A_j\| = k_j for every option of every instance. |
| Grid θ/β, θ ∈ {0, ½, 1, 2, 4, 8} | Does not transfer. Here λ saturates at l^+, because base probabilities are > 0, unlike M3's copies. The useful window ends at θ_max: 0.033 on bike_small and 0.34 on aix70_2 (every M3 point above it), 227 on Quebec (every point below it). | Negative or nil. |
| Budget tolerance τ | Not needed. Contraction and removal can only lower F_t for scenarios over budget. The tolerance is needed only in a per-copy affordability cut. | n/a |
| One multiplier per whole path | Already in place: the blue seed carries ⌊μβ⌋ once, and re-seeding subtracts the previous penalty. | n/a |
| Per-copy (option-conditioned) uselessness | Missing. Valid with a tolerance. | Structurally 0 on Aix, Quebec and Aude, where every improved arc is the only out-arc of its tail. 6 copies on bike_small and 4 of 6486 on bikemid200. |
| Source filter on base arcs | Missing. Valid; almost all of its effect comes from the λ=0 Π^+ test. | 0.04-2.5% of base arcs. |
| Tight big-Ms after --nb-mus ≥ 1 | Consistent. M3 computes everything on the G_t it receives, and x = 0 ∈ X^β. | n/a |

A side finding, distinct from M3: **G_t is never pruned backward from t.** compute_contracted_generalized_flow_graph.hpp:117-134 only prunes forward from the sources. On bike_small at B=100, 856 copies and 4360 base arcs, out of 6587 G_t arcs, lie in components that cannot reach t. This is what m3gap first attributed to the per-copy test. It matters only on disconnected cut-outs.

## E. Numerics (pre-existing, shared with --nb-mus 0)

The clamp p < 1e-8 → finite length, and flooring each arc's length, make labels exact for the integer lengths only. The per-pair loss is relative, about P*·(h−1)·resolution for a path of h hops: 4.5e-8 at h=10 and 5e-6 at h=1000. It is not bounded by the resolution in absolute terms, so "exact up to --resolution" in src/commands/registry.cpp:53 overclaims. On Quebec, target 206 shows 5.46e-6 relative with N=0 as well.

## Recommended fixes (not applied)

1. **Grid:** index-based, with the last point exactly max_mu. Build it only if `max_mu > min_mu && isfinite(max_mu)`, and skip mu with mu·B ≥ 2^62. This fixes A1, A2 and A4.
2. **Seeds:** skip w == v / w == u, plus the useless early exit. This fixes A3 and makes Debug runs possible.
3. **Empty options:** `continue` on empty `arcs_options[i]`. This fixes A5.
4. **Thesis text:** the C list.
5. **Experiments for the owner to decide:**
   - (a) flow_mip end-to-end timings over a budget sweep with the fixed grid. Keep --nb-mus opt-in until then, because of B=5.
   - (b) Blue-wins ties behind the integer-zero-SCC guard, on the N=0 path as well, timed end to end.
   - (c) Grid shape: include max_mu, plus 2-3 decades below it.
6. **Test:** the gtest of A7.

## Not verified by me

- B=5 and B=10 timings: critic only, 2 repeats.
- The +28-32% blue-ties label gains and the m3gap grid comparisons: reviewer probes, cross-checked by verifiers but not rerun by me.
- No solve-time measurement exists for blue ties or for alternative grids.
