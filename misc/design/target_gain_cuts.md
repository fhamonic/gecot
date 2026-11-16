# multiflow_benders: cuts per source on the target quality gains, removed

Status, 2026-11-16: removed from the code. `--target-gain-cuts` was an experimental flag of `multiflow_benders`, added in 140f91b, which the tag `archive/multiflow-target-big-m` keeps. The cuts tighten the root much more than any constant of the rows f^i_t ≤ M · x_i, but cost more time than they save. The measurements are in `multiflow_target_big_m.md`, and this note keeps what is needed to bring them back.

## The cut

A target quality gain q^i_t has a variable f^i_t = x_i · f_t, which the master bounds by f^i_t ≤ f_t and f^i_t ≤ M^t_t · x_i. The second row is weak at a fractional x_i, whatever the constant: the exact one moves the root by at most 1.34 points (`multiflow_target_big_m.md`, the flow MIP per target). The cuts split it per source, as the cuts of f_t split the flow:

    f^i_t ≤ Σ_s min( m_st · x_i , F_s(w_s · x_i + Σ_j g_sj · x_j) )

- m_st = w̄_s · Π̄(s, t), the most that s can bring to t, as in the arc constants of the method.
- F_s(supply) is the largest flow of s to t at the current point. Its base supply w_s is multiplied by x_i, its gains g_sj are left as they are.
- For s = t: min(w̄_t · x_i, w_t · x_i + Σ_j g_tj · x_j).

**Valid at every scenario y.**
- If y buys i, both terms are at least what s brings to t, f_s(y), since m_st bounds it and F_s with the supply w_s + Σ_j g_sj y_j is f_s(y). So the sum is at least f_t(y) = f^i_t.
- If y does not buy i, f^i_t = 0 and both terms are at least 0.

**Linear at the point x̂ of a round.**
- Per source, take the term that is smaller at x̂.
- The first term is m_st · x_i.
- The second is the flow's Benders cut, the same dual solution ρ as the cut of f_t. Its constant ρ_s · w_s goes to the coefficient of x_i.

## Where it was in the code (140f91b)

- **`target_block::separate(x̂, rebuild, gain_cuts)`** filled one cut per target quality gain, in the order of `target_quality_gains`.
  - t's own term came first, from the f_t cut's start. Let own = value − w_t · (1 − x̂_i). The term is m_tt · x_i if m_tt · x̂_i ≤ own, with m_tt = `_best_quality(t)`; otherwise the f_t start with w_t added to the coefficient of x_i.
  - Then, in the loop over the sources, after the flow of f_t, for each gain k with option i, with cap = m_st · x̂_i:
    - if cap > 0, a second flow: flow = `_largest_flow(source_s, supply − w_s · (1 − x̂_i), x̂, search)`, on the same A_s and search;
    - if flow < cap, the term is the flow, and `add_duals(source_s, c, c.coefficients[i])` adds its dual terms, with the constant into the coefficient of x_i;
    - otherwise, the term is cap, and m_st goes to the coefficient of x_i.
  - `add_duals` was the dual part of the cut of f_t, made a lambda to be shared.
- **`_separate(blocks, x̂, gain_cuts)`** filled them block by block.
- **The rounds on the relaxation** read the f^i_t at the optimum. A gain cut was added when violated, with the tolerance of the cuts of f_t; `cut_constraint` took the variable to bound.
- **At the candidates**, nothing: at a scenario the two rows of f^i_t are exact.
- **`root_gain_cuts`** was reported with the flag.

## What it gave

The full tables are in `multiflow_target_big_m.md`. Root gaps are in % of the best gain.

- **The root, 30 s, the 23 short-suite runs with target quality gains:**
  - `urban-greening` at 7500: 2.67 → 0.65 %, against 1.84 % with constants at or below the exact ones;
  - `rural-planting` at 510: 0.36 → 0.12 %;
  - `periurban-mixed` at 2500: 1.96 → 1.40 %;
  - unchanged on `dks` and `bike-squirrel`.
  - With the cuts in place, even those constants add at most 0.03 points, except on `urban-greening` at 1000.
- **The time, 120 s, larger budgets:**
  - the 12 runs both solve take 443.9 s with the cuts, against 381.6 s without;
  - the rounds cost 2 to 14 s more on 9 of the 12, one flow more per source and gain, and the extra rows slow the nodes;
  - only `urban-greening-large` at 16000 gains clearly, from 48.2 to 32.5 s.
- **hexagon 1_1900_3800 at 151, 300 s, `--no-stored-arcs`:** 520 gain cuts in 5 rounds. The relaxation and the bound stay where they are, as the rounds are far from converged. The rounds take 4 s more, and the memory 2.43 against 2.41 GB.
- **Very small budgets, 60 s:**
  - the roots move by at most 0.78 points;
  - where at most 7 options fit, the exact constants come within 0.2 points of the cuts;
  - beyond, the cuts are tighter by 0.11 to 1.25 points.

**Validity, as checked then:**
- the oracle: 469 of 469 runs over 7 configurations, 3 of them with the cuts (with `--no-in-out`, with `--no-stored-arcs`);
- 201 of 201 runs with `max_pushes` 0 and 1, through the fallback of a source that gives up;
- a fuzz with target quality gains on most vertices: 360 of 360 runs, and 180 of 180 at each `max_pushes`, with the gain cuts firing.

## If they come back

- **One flow for both.** The supply of a source for f^i_t is at most its supply for f_t, so its flow along the best paths is a prefix of the one for f_t. Its labels, at the point where the smaller supply runs out, would give the cut of f^i_t without a second flow. That would remove most of the rounds' extra cost.
- **Fewer cuts.** Separate them only in the plain rounds after in-out, or only for the targets whose row f^i_t ≤ M^t_t · x_i is tight at the optimum.
- **The removal left the cut of f_t as it was on main.** `separate`, `_separate`, `cut_constraint` and the rounds are back to main's text, without the `add_duals` lambda.
