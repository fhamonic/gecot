# `multiflow_benders` step by step

This page follows a run of `multiflow_benders` in the order in which it computes things, from the instance to the result. Each step says what is computed and how, which parameters change it, and what `-v` logs about it. The formulation and its proofs are in [Benders decompositions](benders.md#multiflow_benders-cuts-per-source): this page refers to them rather than repeating them.

The three woods of [A first run](../getting-started/first-run.md), with a budget of 4, serve as the example throughout. Their numbers are worked by hand, in the units of the instance.

## The run in brief

1. [The instance](#1-the-instance) is rewritten, as for every algorithm.
2. [The arcs of each case are labelled](#2-the-labels-of-the-arcs) strong or useless for each target.
3. [The master](#3-the-master) gets the scores of the cases, the criterion, the options and their budget rows.
4. [Each target gets a block](#4-a-block-per-target): its graph $G_t$, the bound $M^t_t$ of its flow, and its columns in the master.
5. [Each block finds its sources](#5-the-sources-of-a-block), with their arcs $A_s$ and their constants $m_{sa}$.
6. [A MIP start](#6-the-cuts-of-the-mip-start), if there is one, gives every block a first cut.
7. [Rounds of cuts on the linear relaxation](#7-the-rounds-on-the-relaxation) tighten the master before the search.
8. [The branch and bound](#8-the-search) checks each candidate of the solver and cuts those that the master overvalues.
9. [The result](#9-the-result) is the best candidate, with the lower of two bounds.

Steps 6 to 8 build their cuts with [the separation of a block](#the-separation-of-a-block-at-a-point), described after step 5.

The run of the example, with the list of parameters that `-v` prints first left out:

```console
$ gecot optimize multiflow_benders three-woods.json 4 -v
[   0.0s] GECOT 2.0, algorithm multiflow_benders
[   0.0s] Instance: /home/user/three-woods.json
[   0.0s] Options count: 3
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s]    case     | vertices | habitats | improvable |  arcs   | improvable
[   0.0s]             |          |          |  vertices  |         |    arcs
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s]    squirrel |        3 |        3 |          1 |       4 |          4
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s] Budget 4: 57.14 % of the options' total cost (7)
[   0.0s] The reformulation removes, buys and merges nothing
[   0.0s] Qualities scaled by 2^6: algorithm's scores by 2^12
[   0.0s] Initial score 18.4, for the algorithm 75366.4
[   0.0s] Preprocessing the 'squirrel' graph
[   0.0s] Computing contractions of the 'squirrel' graph
[   0.0s] -------------+-------------+--------------+------------+-----------
[   0.0s]     case     | strong arcs | useless arcs |  vertices  |    arcs
[   0.0s]              | per target  |  per target  | per target | per target
[   0.0s] -------------+-------------+--------------+------------+-----------
[   0.0s]     squirrel |        2.00 |         2.00 |       3.00 |       4.00
[   0.0s] -------------+-------------+--------------+------------+-----------
[   0.0s] Targets: 3, source-target pairs: 6, stored arcs: 16
[   0.0s] Solving the optimization model
[   0.0s] Ctrl-C stops the algorithm and keeps its best result so far
[   0.0s] -------------+--------------+--------------+-------------
[   0.0s]    round     |  fractional  |   max gain   |  violation
[   0.0s]              |     gain     |              |
[   0.0s] -------------+--------------+--------------+-------------
[   0.0s]            1 |            - |    +130.65 % |      8.1e-01
[   0.0s]            4 |     +80.49 % |     +80.49 % |      0.0e+00
[   0.0s] -------------+--------------+--------------+-------------
[   0.0s] Rounds on the relaxation: 4, cuts: 4
[   0.0s] Model: 8 variables, 8 constraints, 28 nonzeros
[   0.0s] Solver stopped: optimal
[   0.0s] Cuts on the candidates of the search: 1
Status          optimal
Initial score   18.4
Solution score  28.76 (+56.30 %)
Upper bound     28.76 (+56.30 %)
Guaranteed      at least 100.0 % of the best possible gain
Cost            4 of 4
Time            4 ms
Options bought  2 of 3:
    hedgerow
    plantation
```

## 1. The instance

Before any algorithm runs, `gecot optimize` [rewrites the instance](preprocessing.md#the-instance-given-to-the-algorithms):

- the options of cost 0 are bought, and those over the budget are removed;
- the vertices linked by arcs of probability 1 are merged;
- the qualities and the criterion are multiplied by powers of two.

`multiflow_benders` refuses a criterion that can decrease when the score of a case increases, as its master bounds the scores from above only ([Linearizing the criterion](flow.md#linearizing-the-criterion)). Everything that follows counts in `--time-limit`, and so does the heuristic of `--mip-start-algorithm`, which runs before.

In the example nothing is removed, bought or merged. The qualities are multiplied by $2^6$, so the scores are multiplied by $2^{12}$: the solver sees an initial score of 75,366.4 for 18.4. The numbers below are in the units of the instance, before this scaling.

## 2. The labels of the arcs

For each case, the colouring algorithm of [Preprocessing](preprocessing.md#strong-and-useless-arcs) finds, for each target $t$, the $t$-strong arcs $S(t)$ and the $t$-useless arcs $W(t)$. It makes two runs of an adapted Dijkstra's algorithm per arc, in parallel. `--preprocessing` chooses what the labels are used for, `--resolution` sets the precision of their comparisons, and `--nb-mus` and `--mus-sweep` add the labels that hold within the budget only. The log shows `Preprocessing the 'squirrel' graph`.

In the example, for the target `wood_c`:

- $(a, b)$ and $(b, c)$ are strong: in every scenario, each starts a best path of its tail to wood C. Both are improved by an option, so neither is contracted.
- $(b, a)$ is useless: through it, wood B reaches wood C only by coming back.
- $(c, b)$ leaves the target, and is useless too.

This gives the 2.00 strong and 2.00 useless arcs per target of the table: the other two targets have as many.

## 3. The master

The master is the program that the solver solves. This step gives it:

- a variable $z_k$ per case, and the variables of the [linearized criterion](flow.md#linearizing-the-criterion), the one of the whole criterion being maximized;
- a binary variable $x_i$ per option;
- a budget row per cost dimension, and a row $\sum_{i \in \Omega_l} x_i \le 1$ per conflict set;
- a row $z_k \le 0$ per case, which the columns of step 4 fill.

The master never has a flow variable $\phi$: the flows stay in the blocks.

## 4. A block per target

Each vertex $t$ of a case that can be a target gets a block, in the order of the vertices. A vertex can be a target when $q_t > 0$, or when some option gives it a target quality gain $q^i_t > 0$. With `--all-targets`, every vertex is one.

1. **The graph $G_t$** is [the graph of the target](preprocessing.md#the-graph-of-a-target). Its arcs of $W(t)$ are removed, its strong arcs that no option improves are contracted, and the arcs that leave $t$ and the vertices that no quality reaches are pruned. Each improved copy $a^i$ is an arc of its own, with the probability $\pi^i_a$ and the option $i$.
2. **The constant $M^t_t$** is that of `--big-m` at $t$ (`knapsack_bnb` by default): a bound on the flow that can reach $t$ within the budget. With `tight` and `exact`, the constant of each target quality gain is also lowered. Unlike `flow_mip`, no constant is computed for the arcs: each source gets its own in step 5.
3. **The columns** go into the master:
    - $f_t \in [0, M^t_t]$, with the coefficient $q_t$ in the row of $z_k$;
    - for each gain $q^i_t > 0$, a variable $f^i_t$ with the coefficient $q^i_t$, and the rows $f^i_t \le f_t$ and $f^i_t \le M^t_t \cdot x_i$. With `tight` and `exact`, the lowered constant of the gain of $i$ takes the place of $M^t_t$ in the second row.

The row of the case now reads $z_k \le \sum_{t \in V_k} \big( q_t \cdot f_t + \sum_i q^i_t \cdot f^i_t \big)$. Until a target receives a cut, only its bound $M^t_t$ holds its flow: without it the relaxation would be unbounded. The log gives the size of the graphs, on average per target, in its table of the cases.

In the example, the graph of `wood_c` keeps its three vertices and four arcs:

<figure class="gecot-figure">
<svg viewBox="0 20 560 165" width="560" role="img" aria-label="The graph of the target wood_c. Wood A, of quality 2, has an arc of probability 0.5 to wood B and an improved copy of probability 0.9 for the hedgerow. Wood B, of quality 1 plus 1 with the plantation, has an arc of probability 0.2 to wood C and an improved copy of probability 0.8 for the underpass. Wood C has quality 3.">
  <defs>
    <marker id="mb-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0 L10 5 L0 10 z" fill="currentColor"/></marker>
    <marker id="mb-arrow-accent" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path class="improved-fill" d="M0 0 L10 5 L0 10 z"/></marker>
  </defs>
  <g font-size="13" fill="currentColor" stroke="none">
    <line x1="106" y1="112" x2="252" y2="112" stroke="currentColor" stroke-width="1.6" marker-end="url(#mb-arrow)"/>
    <line x1="306" y1="112" x2="452" y2="112" stroke="currentColor" stroke-width="1.6" marker-end="url(#mb-arrow)"/>
    <path class="improved" d="M 98 92 Q 180 30 260 90" fill="none" stroke-width="1.6" marker-end="url(#mb-arrow-accent)"/>
    <path class="improved" d="M 298 92 Q 380 30 460 90" fill="none" stroke-width="1.6" marker-end="url(#mb-arrow-accent)"/>
    <circle class="habitat" cx="80" cy="112" r="26" stroke-width="1.5"/>
    <circle class="habitat" cx="280" cy="112" r="26" stroke-width="1.5"/>
    <circle class="habitat" cx="480" cy="112" r="26" stroke-width="1.5"/>
    <text class="habitat-label" x="80" y="117" text-anchor="middle" font-size="15">2</text>
    <text class="habitat-label" x="280" y="117" text-anchor="middle" font-size="15">1</text>
    <text class="habitat-label" x="480" y="117" text-anchor="middle" font-size="15">3</text>
    <text x="80" y="158" text-anchor="middle">wood_a</text>
    <text x="280" y="158" text-anchor="middle">wood_b</text>
    <text x="480" y="158" text-anchor="middle">wood_c = t</text>
    <text class="improved-fill" x="280" y="174" text-anchor="middle">plantation: +1</text>
    <text x="180" y="105" text-anchor="middle">0.5</text>
    <text x="380" y="105" text-anchor="middle">0.2</text>
    <text class="improved-fill" x="180" y="44" text-anchor="middle">hedgerow: 0.9</text>
    <text class="improved-fill" x="380" y="44" text-anchor="middle">underpass: 0.8</text>
  </g>
</svg>
<figcaption>The graph of the target wood_c. Each improved copy, in orange, is an arc of its own, which only its option opens. The arc from wood B to wood A is useless for wood C, and the arc from wood C to wood B leaves the target: both are removed.</figcaption>
</figure>

Its constant is what can reach wood C with every option, the plantation's gain counted by the knapsack:

$$
M^c_c = 3 + 2 \times 0.72 + 1 \times 0.8 + 1 \times 0.8 = 6.04\,.
$$

In the same way $M^a_a = 5.96$ and $M^b_b = 6.2$. Wood B has the target quality gain of the plantation, so the master has 8 variables, $z$, $x_h$, $x_u$, $x_p$, $f_a$, $f_b$, $f_c$ and $f^p_b$. It has 4 rows: the budget, the score, and the two rows of $f^p_b$.

## 5. The sources of a block

The blocks find their sources in parallel. For a block:

1. **Two searches from $t$.** Two runs of Dijkstra's algorithm from $t$ on the reversed graph give $\Pi(u, t)$ with no option, the copies being closed, and $\overline{\Pi}(u, t)$ with every option.
2. **The sources** are the vertices $s \ne t$ with $\overline{w}_s > 0$ and $\overline{\Pi}(s, t) > 0$, those that can bring something to $t$. The quality of $t$ itself is no source: it enters every cut directly (step 1 of the separation).
3. **A search from each source.** A run of Dijkstra's algorithm from $s$, with every option, gives $\overline{\Pi}(s, u)$. It does not go on from a vertex $u$ with $\overline{\Pi}(s, u) \cdot \overline{\Pi}(u, t) < \Pi(s, t) \cdot (1 - 10^{-9})$: no arc of $A_s$ leaves such a vertex, nor one reached through it.
4. **The arcs $A_s$** are those $a = (u, v)$ with $\overline{\Pi}(s, u) \cdot \pi_a \cdot \overline{\Pi}(v, t) > 0$ and at least $\Pi(s, t) \cdot (1 - 10^{-9})$. In every scenario, a most reliable path of $s$ to $t$ uses only these arcs ([why](benders.md#multiflow_benders-cuts-per-source)).
5. **The constants:** each improved copy $a$ of $A_s$ that leaves $u$ gets $m_{sa} = \overline{w}_s \cdot \overline{\Pi}(s, u)$, the most that $s$ alone can bring to $u$.

The sets are kept for the whole run. They add up to at most the arcs of $G_t$ times its sources, which can exceed the memory on large graphs: `--no-stored-arcs` keeps none and runs steps 3 to 5 again for each source at each separation. The cuts are the same. The log gives `Targets: 3, source-target pairs: 6, stored arcs: 16`, and the report the pairs as `sources`.

In the example, the block of `wood_c` has two sources:

| Source | $\Pi(s, c)$ | Arc | $\overline{\Pi}(s, u) \cdot \pi_a \cdot \overline{\Pi}(v, c)$ | In $A_s$ | $m_{sa}$ |
|---|---:|---|---:|:-:|---:|
| `wood_a` | 0.1 | $(a, b)$ | $1 \times 0.5 \times 0.8 = 0.4$ | yes | |
| | | $(a, b)$, hedgerow | $1 \times 0.9 \times 0.8 = 0.72$ | yes | $2 \times 1 = 2$ |
| | | $(b, c)$ | $0.9 \times 0.2 \times 1 = 0.18$ | yes | |
| | | $(b, c)$, underpass | $0.9 \times 0.8 \times 1 = 0.72$ | yes | $2 \times 0.9 = 1.8$ |
| `wood_b` | 0.2 | $(a, b)$ and its copy | $0$: B does not reach A in $G_c$ | no | |
| | | $(b, c)$ | $1 \times 0.2 \times 1 = 0.2$ | yes | |
| | | $(b, c)$, underpass | $1 \times 0.8 \times 1 = 0.8$ | yes | $2 \times 1 = 2$ |

Wood B has $\overline{w}_b = 1 + 1 = 2$ with the plantation. That makes 6 arcs for the block of `wood_c`, 6 for that of `wood_a` and 4 for that of `wood_b`: the 16 arcs of the log.

## The separation of a block at a point

Steps 6 to 8 all use this computation: the cut of a target $t$ at a point $\hat{x} \in [0, 1]^\Phi$. The point is fractional on the relaxation and a scenario elsewhere. The cut has the form

$$
f_t \le \text{constant} + \sum_{i \in \Phi} \text{coefficient}_i \cdot x_i\,,
$$

valid in every scenario, and whose right-hand side equals the largest flow at $\hat{x}$. The blocks are separated in parallel, and the sources of a block one after the other:

1. **The target's own quality** starts the cut: the constant $w_t$, the coefficient $w^i_t$ of each of its source quality gains, and the value $w^{\hat{x}}_t$.
2. **One search from $t$**, by Dijkstra's algorithm on the reversed graph over the arcs open at $\hat{x}$, gives each vertex the gain $p_u$ of its best path to $t$. The open arcs are the arcs without option and the copies of the options with $\hat{x}_i > 0$. These labels are feasible for the first search of every source, whose arcs are among the open ones: $p_t = 1$ and $p_u \ge \pi_a \cdot p_v$ on each arc $a = (u, v)$.
3. **For each source $s$, its largest flow at $\hat{x}$**, by Onaga's algorithm (`melon::onaga`):
    1. The capacities are infinite on the arcs of $A_s$ without option, $m_{sa} \cdot \hat{x}_i$ on its copies, and 0 elsewhere. The supply is $w^{\hat{x}}_s = w_s + \sum_i w^i_s \cdot \hat{x}_i$.
    2. A search from $s$ finds a best residual path to $t$. Each residual arc from $u$ to $v$ has a gain $g$. An arc $a = (u, v)$ that is not full is taken forward, with $g = \pi_a$. An arc $a = (v, u)$ that carries flow is taken backward, with $g = 1 / \pi_a$, above 1. The search is Dijkstra's algorithm on the relabeled gains $g \cdot p_v / p_u$, which the labels keep at most 1 ($p_u \ge g \cdot p_v$ on every residual arc), and it stops once it settles $t$. With $D_u$ the relabeled gain of the best path from $s$ to $u$, each vertex settled takes the label $p_u \cdot D_t / D_u$. The labels stay feasible and are tight on the path, so the arcs that the push opens backward get a relabeled gain of 1.
    3. The path takes as much of the supply left as its tightest arc allows: a *push*. Either the supply is sent, or an arc of the path is now full, or an arc taken backward is now empty.
    4. Steps 2 and 3 repeat until less than $10^{-12}$ of the supply is left, or no path is. After 100,000 pushes the flow stops where it is.
    5. One search from $t$ on the last residual graph gives $\rho^s_u$, the gain of the best residual path from $u$ to $t$, 0 where there is none. Once the flow is the largest, these potentials are the least optimal dual solution of the program of $s$.
4. **The part of $s$ in the cut** is read from its potentials:
    - $\rho^s_s \cdot w_s$ is added to the constant;
    - $\rho^s_s \cdot w^i_s$ to the coefficient of each source quality gain of $s$;
    - $m_{sa} \cdot \big( \pi^i_a \cdot \rho^s_v - \rho^s_u \big)^+$ to the coefficient of $x_i$, for each copy $a^i = (u, v)$ of $A_s$;
    - and its flow to the value.

    A flow stopped after 100,000 pushes adds the bound $\text{flow} + \rho^s_s \cdot \text{supply left}$ instead. The potentials are feasible all the same, so the cut is still valid.

By weak duality the cut holds in every scenario, since the constants $m_{sa}$ hold in every scenario. By strong duality it equals the largest flow at $\hat{x}$ when every flow converged.

The labels lead the searches of a source toward $t$, so that a search settles only the vertices that a path as good as the best one could cross, not all of $A_s$. A search after a push starts from the part of the last tree that the push left residual, and finds the arcs to take backward in lists of the arcs that carry flow. These mechanisms are those of `melon::onaga`, in melon 1.1.0.

### In the example

Take the block of `wood_c` at the point $\hat{x}_h = 0.5$, $\hat{x}_u = 0.75$, $\hat{x}_p = 0.25$, chosen to illustrate. It is within the budget, at a cost of $1 + 2.25 + 0.5 = 3.75$. The target's own quality starts the cut with the constant 3. The capacities of the copies are $2 \times 0.5 = 1$ for the hedgerow and $1.8 \times 0.75 = 1.35$ for the underpass, for wood A. Wood A has the supply 2:

| Push | Path | Gain | Amount sent | Reaches C | What it fills |
|---|---|---:|---:|---:|---|
| 1 | hedgerow, underpass | 0.72 | 1 | 0.72 | the copy of the hedgerow |
| 2 | $(a, b)$, underpass | 0.4 | 0.9 | 0.36 | the copy of the underpass, $0.9 + 0.9 \times 0.5 = 1.35$ |
| 3 | $(a, b)$, $(b, c)$ | 0.1 | 0.1 | 0.01 | the supply is sent |

Wood A brings 1.09. Both copies are full, so in the last residual graph $\rho_b = 0.2$, by the arc $(b, c)$, and $\rho_a = 0.5 \times 0.2 = 0.1$, by the arc $(a, b)$. The part of wood A in the cut is

$$
2 \times 0.1 \;+\; 2 \times (0.9 \times 0.2 - 0.1) \cdot x_h \;+\; 1.8 \times (0.8 \times 1 - 0.2) \cdot x_u \;=\; 0.2 + 0.16\, x_h + 1.08\, x_u\,.
$$

Wood B has the supply $1 + 0.25 = 1.25$, and the copy of the underpass the capacity $2 \times 0.75 = 1.5$. One push sends the whole supply through it and brings 1. The copy is not full, so $\rho_b = 0.8$. The part of wood B is $0.8 + 0.8\, x_p$, and the copy of the underpass adds $2 \times (0.8 - 0.8) = 0$. The cut of `wood_c` at $\hat{x}$ is

$$
f_c \;\le\; 4 + 0.16\, x_h + 1.08\, x_u + 0.8\, x_p\,,
$$

which is $3 + 1.09 + 1 = 5.09$ at $\hat{x}$, the largest flow there. In the 8 scenarios it is never below the flow, and equal to it where the hedgerow and the underpass are both bought:

| Bought | none | $p$ | $u$ | $u, p$ | $h$ | $h, p$ | $h, u$ | $h, u, p$ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| $f_c$ | 3.4 | 3.6 | 4.6 | 5.4 | 3.56 | 3.76 | 5.24 | 6.04 |
| cut at $\hat{x}$ | 4 | 4.8 | 5.08 | 5.88 | 4.16 | 4.96 | 5.24 | 6.04 |

## 6. The cuts of the MIP start

With `--mip-start` or `--mip-start-algorithm static` or `greedy`, the solution is handed to the solver as its start. It is also separated, as a scenario, for every block, and each cut goes into the master as a row before the rounds. By default there is no MIP start.

## 7. The rounds on the relaxation

Before the search, the master is solved as a linear program, with $x \in [0, 1]^\Phi$, and cuts are added round after round ([Cuts on the relaxation](benders.md#cuts-on-the-relaxation)). Each round:

1. **Solves the relaxation.** The rounds stop if it ends without an optimum, on Ctrl-C, which then leaves the search to run until the next one, or once half of `--time-limit` has passed, which leaves the other half to the search. The value of the relaxation is the `max gain` of the table.
2. **Reads the optimum**: the options $x^*$, brought back into $[0, 1]$, and the flows $f^*_t$.
3. **Moves the center** toward $x^*$, by `--in-out-ratio` of the way, 0.9 of it by default. In the first round the center is $x^*$ itself.
4. **Separates every block** at the center, or with `--in-out-separation` at the point that share of the way from the old center to $x^*$. With `--in-out-lift δ`, $10^{-7}$ by default, each option of that point is raised to at least $\delta$. Once that point has stopped adding cuts, both steps become 1 and the lift ends: the point separated is $x^*$ from then on. `--in-out-ratio 1` separates $x^*$ from the start.
5. **Scores the point separated** with its flows. The criterion is evaluated with each $f_t$ held under $M^t_t$ and each target quality gain under its constant times $\hat{x}_i$. The best value so far is the `fractional gain`: the relaxation with all the cuts reaches at least that much. A round in which some flow stopped after 100,000 pushes gives no value, nor does a lifted point, which can be over the budget.
6. **Adds the cuts that the optimum violates.** A cut separated away from $x^*$ is tested at $x^*$. It is added when $f^*_t > \text{cut}(x^*) \cdot (1 + \tau) + 10^{-7}$, where $\tau$ is `--root-cut-tolerance` ($10^{-7}$ by default). The `violation` of the table is the smallest $\tau$ at which this round would add no cut.
7. **Decides what comes next:**
    - with `--root-cut-gap g`, the rounds stop once the value of the relaxation exceeds the best value of the points separated by at most $g$ times the latter: the scores behind `max gain` and `fractional gain`;
    - a round that adds a cut is followed by another;
    - a round of the center, or of a lifted point, that adds none hands over to the optimum;
    - a round of the optimum that adds none ends the rounds.

The options become binary again, and the cuts stay in the master as rows for the search. The value of the last relaxation is reported as `root_relaxation_value`. `--no-root-cuts` skips these rounds. The log ends them with `Rounds on the relaxation: 4, cuts: 4`. The table has a row at most every second: this run took 4 ms, so it shows its first round and its last.

In the example, the first relaxation has no cut. Each flow takes its bound, $f_a = 5.96$, $f_b = 6.2$ and $f_c = 6.04$, and the plantation is bought for the target quality gain of wood B. The relaxation is worth $2 \times 5.96 + 6.2 + 6.2 + 3 \times 6.04 = 42.44$, which is $+130.65\,\%$ over the initial score of 18.4: the `max gain` of the first row. Four rounds bring it to $+80.49\,\%$, against the optimum's $+56.30\,\%$. The search closes the rest.

## 8. The search

The master, with its options binary again and its cuts as rows, goes to the branch and bound of the MILP solver. The solver's relative gap is set to $10^{-10}$, Gurobi's `OptimalityTol` to $10^{-9}$, and its feasibility tolerance to `-t`. With Gurobi, CPLEX or COPT, each candidate of the solver goes through a callback before the solver can accept it:

1. **The scenario.** The candidate's options are read as $y_i = 1$ when $x_i > 0.5$.
2. **The true flows.** Each block computes $f_t(y)$ by one run of Dijkstra's algorithm from $t$ on the reversed graph over the arcs open at $y$, in parallel. At a scenario each source sends its whole quality along a single best path, which lies in $A_s$. The labels of this one search therefore give every flow: $f_t(y) = w^y_t + \sum_s w^y_s \cdot \Pi^y(s, t)$, without Onaga's algorithm.
3. **The overvalued targets.** A target is overvalued when the candidate's $f_t$ exceeds $f_t(y)$ by more than $f_t(y)$ times `-t`. The scores of the cases at $y$ are computed on the way.
4. **Their cuts.** Only these blocks are separated at $y$, in parallel, and each cut is handed to the solver as a lazy constraint.
5. **The hand-back.** A candidate that received a cut is rejected by the solver, although $y$ is a valid solution. GECOT hands $y$ back as a new solution, with its true values: $f_t = f_t(y)$, $f^i_t = y_i \cdot f_t(y)$, the scores of the cases and the variables of the criterion. A candidate that received no cut is accepted: its score is exact to within `-t`.

The search ends when the solver has proved its best solution optimal for the master. The progress table shows the solver's score, bound and nodes, and the log ends with `Cuts on the candidates of the search: 1`.

With the other solvers, which take no lazy constraints through mippp, the master is [solved again](benders.md#without-lazy-constraints) with the cuts of each solution as rows, until a solution receives none. `--no-lazy-constraints` makes Gurobi, CPLEX and COPT do the same, for comparison. SCIP is refused, as its model in mippp cannot add columns.

### Why a candidate's cut still needs the flow

The value of a candidate needs only the search from $t$, but its cut also needs Onaga's algorithm. In the example, take the candidate that buys the hedgerow and the plantation, the optimum, and suppose that the master overvalues `wood_c` there.

The search from $t$ over the arcs open at $y$ gives $\Pi^y(b, c) = 0.2$ and $\Pi^y(a, c) = 0.9 \times 0.2 = 0.18$, so

$$
f_c(y) = 3 + 2 \times 0.18 + 2 \times 0.2 = 3.76\,.
$$

The separation at $y$ then runs one push per source:

- **Wood A** sends its 2 through the copy of the hedgerow, whose capacity is $m_{sa} \cdot y_h = 2$. The copy is exactly filled and leaves the residual graph. In it, $\rho_a = 0.5 \times 0.2 = 0.1$, by the arc without option, and not $0.18$. The part of wood A is $0.2 + 0.16\, x_h + 1.08\, x_u$.
- **Wood B** sends its 2 through $(b, c)$, the underpass being closed. Its part is $0.2 + 0.2\, x_p + 1.2\, x_u$.

The cut at $y$ is therefore

$$
f_c \;\le\; 3.4 + 0.16\, x_h + 2.28\, x_u + 0.2\, x_p\,.
$$

It charges the hedgerow, an option that $y$ buys. This differs from [the cut of a target](benders.md#the-cut-of-a-target) of `flow_benders`, where $\sigma = 0$ for the options of $y$. It is no error: the copy that wood A fills exactly leaves the residual graph. Potentials taken without the push, by a search from $t$ over the arcs open at $y$, have $\rho_a = 0.18$ and would give wood A the part $0.36 + 1.08\, x_u$. That part is 0.36 in the scenarios that buy neither the hedgerow nor the underpass, where wood A brings only 0.2. With the charge, the cut equals the flow in every scenario that leaves the underpass closed: 3.4, 3.6, 3.56 and 3.76.

A copy is filled exactly whenever a source has its whole quality $\overline{w}_s$ and reaches the copy as well as it would with every option. On the coverage instances of the benchmark, where every source has quality 1 and every probability is 0 or 1, every source that a candidate connects fills its copy exactly. There, taking the potentials of each source at a candidate without its pushes, by a search from $t$ over its arcs, let `cover-40` at a budget of 12 reach a time limit of 30 s, against 9.5 s with the pushes.

## 9. The result

- **The status** is that of the search: `optimal`, `time limit reached` or `interrupted` with Ctrl-C, among others.
- **The solution** is the solver's best, its options read as in step 8. GECOT computes its score again from the options bought, in the units of the instance.
- **The bound** is the lower of the solver's bound and of `root_relaxation_value`: the relaxation with its cuts bounds the optimum too, and is the better bound when the search stops early.
- **The report** adds to the size of the model and to `model_build_time_ms`:
    - `lazy_constraints`, the cuts of step 8, and `nodes`;
    - `sources`, the pairs of step 5;
    - `root_rounds` and `root_cuts`, from step 7.

If the solver stops before any solution, the result is the MIP start when there is one, and an error otherwise.
