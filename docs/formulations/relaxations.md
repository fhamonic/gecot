# Tree and path relaxations

`path_lp` and `tree_lp` do not solve the problem: they solve a *linear relaxation* of it, in which each pair of vertices, or each target, may choose its own set of options. The relaxation has exponentially many variables and is solved by column generation. It gives what the exact algorithms cannot give on large instances within a short time: an upper bound on the optimum, a good solution, and for each option a bound on what the solutions that buy it can score.

## The path formulation

A *pair* is a couple $p = (s,t)$ of vertices of a case $k$. If the options of a set $S \subseteq \Phi$ are bought, the pair is worth

$$
v_p(S) = w^S_s \cdot q^S_t \cdot \Pi^S(s,t)\,,
$$

and the score of the case in a scenario $x$ that buys the set $X$ is $\sum_{p} v_p(X)$. The path formulation has one variable $y_{pS} \ge 0$, a *column*, for each pair and each set of options, and lets each pair choose its set as long as the options it uses are bought:

$$
\begin{aligned}
\max \;\; & \mathcal{C}(z) \\
\text{s.t.} \;\;
& z_k \le C_k + \sum_{p \in k} \sum_{S \subseteq \Phi} v_p(S) \cdot y_{pS} && k \in K && (\gamma_k) \\
& \sum_{S \subseteq \Phi} y_{pS} \le 1 && p && (\sigma_p) \\
& \sum_{S \ni i} y_{pS} \le x_i && p\,,\; i \in \Phi && (\lambda_{pi}) \\
& \sum_{i \in \Phi} c_i \cdot x_i \le \beta && && (\eta) \\
& x \ge 0\,, \; y \ge 0\,,
\end{aligned}
$$

where $C_k$ is the total value of the pairs that no option changes, and the dual variable of each constraint is written on its right. A scenario that buys $X$ is a solution of this program, with $y_{pX} = 1$ for every pair, and its objective is the score of the scenario. The optimum of the program is therefore an **upper bound** on the optimum of the problem.

It is a relaxation in two ways: the options can be bought fractionally, and two pairs can rely on different sets of options. What it keeps is the essential constraint: a pair benefits from an option only as much as the option is bought.

## Column generation

The program is solved on a small subset of its columns, the *master*, which grows as follows. Each round solves the master, reads its dual values, and looks for the columns that would improve it: those of positive *reduced cost*

$$
r_{pS} = \gamma_k \cdot v_p(S) - \sigma_p - \sum_{i \in S} \lambda_{pi}\,.
$$

Finding the best column of a pair, the *pricing*, is a path problem: choose a path from $s$ to $t$ and the options to buy along it, so as to maximize its value minus the prices $\lambda_{pi}$ of these options. `path_lp` solves it by a label search from $t$ backward, in which a label is a path to $t$ with the set of options it buys, and a label is dropped when another one at the same vertex is worth at least as much for no more.

`--pricing` selects how:

- **`search`** (the default) runs one search per pair and per round, with the current dual values. It ignores the budget, so some of its columns cost more than $\beta$.
- **`enumerate`** lists once, for each pair, the sets of options of cost at most $\beta$, except those that are worth no more than one of their subsets. Whatever the dual values, the best column of the pair is among them, and leaving out the sets over the budget tightens the relaxation. It is the choice for instances whose paths cross few options; when many cheap options lie along the paths, the number of sets explodes.

The rounds stop when no column has a positive reduced cost: the master's optimum is then the optimum of the relaxation, and the status is `converged`.

### The Lagrangian bound

The value of the master is *not* a bound before convergence, as columns are missing. But after each complete round, with $r_p = \max_S r_{pS}$ the best reduced cost of each pair, raising each $\sigma_p$ by $\max\{0, r_p\}$ makes the dual values feasible for the dual of the whole relaxation. By weak duality,

$$
L = \text{value of the master} + \sum_{p} \max\{0\,,\, r_p\}
$$

is an upper bound on the relaxation, hence on the optimum of the problem. `path_lp` reports the lowest $L$ over its rounds as its bound: stopped by a time limit or by Ctrl-C, it still returns a proven bound. `--relative-gap g` stops the rounds once $L$ is within $g$ times the value of the master, with the status `relative gap reached`.

The same dual solution bounds each option. In it the constraint of $x_i$ has the slack $-d_i$, where

$$
d_i = \sum_{p} \lambda_{pi} - c_i \cdot \eta \;\le\; 0
$$

is the reduced cost of $x_i$, so a solution that buys $i$ scores at most $L + d_i$. `gecot optimize --reformulation path_lp` uses these bounds to [remove options](preprocessing.md#removing-options-with-the-path-relaxation) before an exact algorithm runs.

### The solution

The relaxation's $x$ is fractional. `path_lp` turns it into a solution within the budget: it takes the options by decreasing value of $x_i$ and buys each one that still fits. If a MIP start was given (`--mip-start`, `--mip-start-algorithm`) and scores more, the start is returned instead. The result is a solution with a guarantee, the Lagrangian bound: on a 40 × 40 raster landscape of the repository (`test/instances/Aix/instance_40x40`) at budget 8, `path_lp` returns in 4 s, with Gurobi on an 8-core desktop computer, a solution proved to reach 98.5 % of the best possible gain.

## The tree formulation

`tree_lp` solves the same kind of relaxation with coarser columns: one per *target* and set of options. A column of the target $t$ is worth

$$
v_t(S) = q^S_t \cdot \sum_{s} w^S_s \cdot \Pi^S(s,t)\,,
$$

the value of all the pairs of target $t$ at once, their most reliable paths forming a tree rooted at $t$. The program is the one above with $t$ in place of $p$, without the constant $C_k$. All the sources of a target must now agree on the options, which the path formulation does not require: the tree relaxation is at least as tight as the path relaxation.

Its pricing is harder: for a target $t$, find the set of options that maximizes $\gamma_k \cdot v_t(S) - \sigma_t - \sum_{i \in S} \lambda_{ti}$. `tree_lp` solves it as a small MILP, the [flow formulation](flow.md) restricted to the target $t$ on its [reduced graph](preprocessing.md#the-graph-of-a-target) $G_t$, with the prices $\lambda_{ti}$ in the objective and the budget as a constraint. The pricing problems of a round are solved in parallel.

!!! warning "`tree_lp` returns the relaxation, not a solution within the budget"

    `tree_lp` is a tool to study the tree relaxation. The options it prints are all those that the relaxation buys, even fractionally, and they can cost more than the budget: GECOT then warns that the solution is over the budget. It reports no upper bound either: its pricing is a MILP solved up to the solver's tolerances, which can miss an improving column, so the value of its master, the `relaxation_value` of its report, is not a proven bound. Use `path_lp` for a bound and a solution, and the [exact algorithms](../usage/choosing.md) for a proof of optimality.

## Parameters

Both take the [common parameters](../reference/parameters.md#common-to-the-milp-based-algorithms). The master is a linear program: `--solver` selects the solver that handles it, and for `tree_lp` the pricing MILPs too.

| Parameter | Algorithm | Meaning |
|---|---|---|
| `--pricing search` or `enumerate` | `path_lp` | The pricing, see above |
| `--relative-gap g` | `path_lp` | Stops once the Lagrangian bound is within $g$ times the value of the master (0: at convergence only) |
| `--max-activations n` | `path_lp` | At most $n$ columns enter the master per round, the best ones (0: every improving column) |
| `--evict-age n` | both | A column that has been in the master for more than $n$ rounds and is no longer attractive goes back to a pool (0: never) |
| preprocessing parameters | `tree_lp` | The reduced graphs of its pricing, see [Preprocessing](../reference/parameters.md#preprocessing) |

`--solver` can be any of the nine solvers for `path_lp`, and any but `glpk` for `tree_lp`. With `--time-limit` or Ctrl-C, both stop at the end of the current round, with the status `time limit reached` or `interrupted`.

Their report gives `iterations`, the number of rounds, `relaxation_value`, the value of the master at the end, the size of the master (`variables`, `constraints`), `pool_columns` and `evicted_columns`; `path_lp` adds `pairs`, `searches` or `columns` depending on its pricing, and `best_bound`.
