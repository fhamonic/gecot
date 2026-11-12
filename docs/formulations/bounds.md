# Bounds

An upper bound $b$ on the score of every solution within the budget turns any solution into a statement of quality. If a solution scores $s$ and buying nothing scores $s_0$, then

$$
\frac{s - s_0}{b - s_0}
$$

is a share of the best possible gain that the solution is *proved* to reach. This is the percentage that `gecot optimize` prints next to its upper bound, where $s_0$ is the score with only the options of cost $0$ bought. `gecot bound` computes such bounds on their own, to certify a solution obtained otherwise, a heuristic one for instance. Its methods go from free and loose to costly and exact.

## `all_options`

The score with every option bought, whatever its cost:

$$
b = \mathcal{C}\Big( \big( PC_{\text{num}}(G^{\overline{x}}_k) \big)_{k \in K} \Big)\,, \qquad \overline{x} = (1, \dots, 1)\,.
$$

Buying more options never lowers the score of a case, so this bounds every scenario as long as the criterion is non-decreasing in the scores of the cases. It ignores the budget: it is tight only when the budget buys almost everything, but it costs one evaluation of the score, and `gecot bound` prints it next to the bound of every other method as a reference.

## `qkp_planes`

This method takes the budget into account for the quality gains. Every probability of connection is at most its value $\overline{\Pi}$ with every option bought, and the qualities are linear in $x$, so the score of a case is at most a quadratic function of $x$:

$$
PC_{\text{num}}(G^x) \;\le\; H + \sum_{i \in \Phi} l_i \cdot x_i + \sum_{i \in \Phi} \sum_{j \in \Phi} Q_{ij} \cdot x_i \cdot x_j\,,
$$

with

$$
\begin{aligned}
H &= \sum_{s,t} w_s \cdot \overline{\Pi}(s,t) \cdot q_t\,, \\
l_i &= \sum_{s,t} \big( w^i_s \cdot \overline{\Pi}(s,t) \cdot q_t + w_s \cdot \overline{\Pi}(s,t) \cdot q^i_t \big)\,, \\
Q_{ij} &= \sum_{s,t} w^i_s \cdot \overline{\Pi}(s,t) \cdot q^j_t\,.
\end{aligned}
$$

Maximizing the right-hand side within the budget is a *quadratic knapsack problem*, itself hard, which is bounded by the upper planes of Caprara, Pisinger and Toth (1999). Split each pair of quadratic terms between its two options, $Q_{ij} + Q_{ji} = \theta_{ij} + \theta_{ji}$ with $\theta \ge 0$. Then what the option $i$ can bring is at most

$$
u_i = l_i + Q_{ii} + \max_{y \in [0,1]^\Phi} \Big\{ \sum_{j \neq i} \theta_{ij} \cdot y_j \;:\; \sum_{j \neq i} c_j \cdot y_j \le \beta - c_i \Big\}\,,
$$

and the bound is

$$
b = H + \max_{x \in [0,1]^\Phi} \Big\{ \sum_{i \in \Phi} u_i \cdot x_i \;:\; \sum_{i \in \Phi} c_i \cdot x_i \le \beta \Big\}\,,
$$

both knapsacks being continuous ones, solved by sorting the options by ratio.

- `--theta` chooses the split: `half` gives half of each pair to each option, `target` gives all of it to the option that has the target quality gain, `source` to the one that has the source quality gain, and `best`, the default, keeps the best of the three bounds.
- `--iterations n` then improves the split by $n$ subgradient steps, which aim at the score given by `--target`, that of a known solution for instance.

The probabilities stay at their best value: without quality gains, the bound is the one of `all_options`.

## `split_lagrange`

This method also takes the budget into account for the arcs. It splits the score into the part due to the quality gains, bounded by the $u_i$ of `qkp_planes`, and the part of the base qualities, a sum over the pairs $p = (s,t)$:

$$
A(x) = \sum_{p} w_s \cdot q_t \cdot \Pi^x(s,t)\,.
$$

A pair benefits from an option only if the option is bought. Relaxing this link with a price $\nu_{pi} \ge 0$ for each pair and each option gives, for every scenario $x$,

$$
A(x) \;\le\; \sum_{p} R_p(\nu) + \sum_{i \in \Phi} \Big( \sum_{p} \nu_{pi} \Big) \cdot x_i\,,
$$

where $R_p(\nu)$ is the best value of a path from $s$ to $t$ that pays the price of each option it uses:

$$
R_p(\nu) = \max_{P\,:\,st\text{-path}\,,\; S \subseteq \Phi} \Big( w_s \cdot q_t \cdot \prod_{a \in P} \pi^S_a - \sum_{i \in S} \nu_{pi} \Big)\,.
$$

The bound adds a continuous knapsack over the options, each worth its quality gains and the prices it collects:

$$
b = \sum_{p} R_p(\nu) + \max_{x \in [0,1]^\Phi} \Big\{ \sum_{i \in \Phi} \Big( u_i + \sum_{p} \nu_{pi} \Big) \cdot x_i \;:\; \sum_{i \in \Phi} c_i \cdot x_i \le \beta \Big\}\,.
$$

It holds for any prices, and `--iterations` subgradient steps (30 by default) lower it, each step pricing every pair of habitats. `--time-limit` stops them and returns the best bound of the complete steps, and `--target` is the score that the steps aim at.

To price a pair exactly, the method needs the options to have a simple shape, which it checks on each case, among the options that the budget can afford:

1. an arc is improved by at most one option;
2. the arcs improved by an option share a vertex, so that a simple path uses them consecutively;
3. the improved arcs that enter a vertex belong to one option.

Otherwise it stops with an error that names the first arc or vertex at fault. Options that each improve one arc, both ways or not, or the arcs around one vertex, satisfy the first two; the third fails when two options improve arcs entering the same vertex, as on the [three woods](../getting-started/first-run.md).

## Bounds of the algorithms

The exact algorithms and the path relaxation prove a bound as they run, and `gecot bound` can return it:

| Method | Bound |
|---|---|
| `flow_mip` | The dual bound of the branch and bound on the [flow formulation](flow.md) |
| `multicut_benders`, `flow_benders`, `multiflow_benders` | The dual bound of the search on the master of the [decomposition](benders.md) |
| `path_lp` | The [Lagrangian bound](relaxations.md#the-lagrangian-bound) of the path relaxation |

They take the parameters of the algorithm, `--time-limit` among them. The first four are exact when the run ends with the status `optimal`: the bound is then the optimum, up to the tolerances. Stopped earlier, they return the bound proved so far, provided that the search found a solution; without one the method fails. The bound of `path_lp` is the value of a relaxation, above the optimum in general, but it is proved from the first complete round and is often tight: see the [example](../usage/bound.md).

## Bounds and criterion

The three methods of this page that are not algorithms bound the weighted sums of the scores of the cases. For a criterion that is such a sum with non-negative weights, one computation gives the bound. For another non-decreasing criterion, a minimum for instance, each case that the criterion reads is bounded separately and the criterion is evaluated on these bounds, which is valid because it is non-decreasing. Like the algorithms, these methods [refuse a criterion](../usage/instance.md#criterion) that is not non-decreasing in the score of every case.

## Reference

A. Caprara, D. Pisinger and P. Toth (1999). Exact solution of the quadratic knapsack problem. *INFORMS Journal on Computing*, 11(2), 125–137.
