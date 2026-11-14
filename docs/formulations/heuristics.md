# Heuristics

The heuristics build a solution by ranking the options by what they gain per unit of cost. They need no solver, evaluate any criterion, and run on landscapes that no exact algorithm can handle, but they give **no guarantee**: their solution can be far from the optimum, and nothing in their output tells how far. A [bound](bounds.md) tells it afterwards.

Their incremental direction handles [conflict sets and several costs](#conflict-sets-and-several-costs); `--decremental` does not.

## Gain and loss of an option

Let $\mathcal{C}(x)$ be the score of a scenario $x$, and $e_i$ the scenario that buys the option $i$ alone. In a scenario $x$ that does not buy $i$, the *gain* of $i$ per unit of cost is

$$
\Delta^+_i(x) = \frac{\mathcal{C}(x + e_i) - \mathcal{C}(x)}{c_i}\,,
$$

and in a scenario that buys it, its *loss* per unit of cost is

$$
\Delta^-_i(x) = \frac{\mathcal{C}(x) - \mathcal{C}(x - e_i)}{c_i}\,.
$$

Each of these ratios costs one evaluation of the score, that is, one run of Dijkstra's algorithm from every vertex of positive source quality in every case. The evaluations of a step are done in parallel.

## Static and greedy, incremental and decremental

There are two heuristics, `static` and `greedy`, and each works in two directions. By default it is *incremental*: it starts from the landscape as it is and buys options. With `--decremental` it starts from the scenario that buys every option, removes options until the cost fits the budget, and ends with its incremental steps among the options removed, to spend what is left of the budget.

**`static`** starts from the landscape as it is, computes the gain $\Delta^+_i(0)$ of every option once, and goes through the options by decreasing gain, buying each one that fits in what is left of the budget. It ranks the options as if they did not interact: $|\Phi|$ evaluations.

**`greedy`** starts from the landscape as it is and repeats: compute the gain $\Delta^+_i(x)$ of each option that still fits in the budget, in the current scenario $x$, and buy the best one. It stops when no option fits. It sees the options that become worth more once others are bought, but only one step ahead: about $|\Phi|^2 / 2$ evaluations at most.

**`static --decremental`** starts from the scenario $\overline{x}$ that buys every option, computes the loss $\Delta^-_i(\overline{x})$ of every option once, and removes the options by increasing loss until the cost fits the budget. The last removal can leave some budget unused: the options removed that fit in it are then bought back, by decreasing gain in the scenario reached.

**`greedy --decremental`** starts from the scenario that buys every option and repeats: compute the loss $\Delta^-_i(x)$ of each option bought, and remove the one whose loss is the smallest, until the cost fits the budget. It then spends what is left of the budget as `greedy` does, among the options removed. This is the principle of the Zonation algorithm.

With `--decremental`, `--only-dec` skips these final purchases.

The incremental steps, these final purchases included, buy as long as an option fits, even one that gains nothing.

When several options are tied for the best gain or the smallest loss, `greedy` takes the first of them in the order of the instance. Tied means that their gains, or losses, differ by less than $10^{-12}$ of the score per unit of cost: within the rounding of the scores, which changes from one run to the next and would otherwise choose among them.

The releases of GECOT before 2.0 name these four `static_incr`, `greedy_incr`, `static_decr` and `greedy_decr`.

## Why they can miss the optimum

A ranking cannot see that two options are worth buying together, nor that two options do the same job. On the [three woods](../getting-started/first-run.md) with a budget of 3, the gains per unit of cost in the initial landscape are

| Option | Cost | Score alone | Gain per unit of cost |
|---|---:|---:|---:|
| `plantation` | 2 | 24.6 | 3.1 |
| `underpass` | 3 | 25.6 | 2.4 |
| `hedgerow` | 2 | 20.96 | 1.28 |

so the incremental direction buys `plantation` first, after which nothing fits, and the decremental one ends on the same solution: a score of 24.6, with a third of the budget unused, while the optimum is `underpass`, at 25.6. On the 40 × 40 raster landscape of the repository (`test/instances/Aix/instance_40x40`, 27 options) at budget 8, the two heuristics reach, in their two directions, 75 % to 98.7 % of the optimal gain (one run each on an 8-core desktop computer, the optimum with Gurobi):

| Algorithm | Score | Gain | Time |
|---|---:|---:|---:|
| `static --decremental` | 418,296.9 | +17.42 % | 42 ms |
| `static` | 432,401.8 | +21.38 % | 28 ms |
| `greedy` | 437,866.4 | +22.91 % | 0.2 s |
| `greedy --decremental` | 437,866.4 | +22.91 % | 0.5 s |
| optimum (`multiflow_benders`) | 438,893.1 | +23.20 % | 1.2 s |

`greedy` is often close, as here in both directions, and sometimes far: Hamonic et al. (2023) compare it with the optimum on real landscapes. A cheap way to use it safely is to pass its solution to an exact algorithm, with `--mip-start-algorithm greedy`: the exact algorithm starts from it and, if it runs out of time, returns the best solution it has with a proven bound.

## Conflict sets and several costs

On an instance with [conflict sets](../usage/instance.md#conflict-sets) or [several costs](../usage/instance.md#several-costs), the incremental `static` and `greedy` change in four ways.

- **The cost of an option** that divides its gain is
  $$
  D_i = \beta_{r_0} \sum_{r \in R,\ \beta_r > 0} \frac{c_{i,r}}{\beta_r}\,,
  $$
  its costs relative to their budgets, in the unit of $r_0$, the first dimension of positive budget. With a single cost, $D_i = c_i$. An option fits when it fits in what is left of every budget.
- **Conflicts.** Once an option is bought, the other options of its conflict sets are no longer candidates.
- **Free options.** An option of cost 0 in a conflict set, which stays a choice, comes before every paid option if it gains something, after them if it loses, and otherwise counts as a gain of 0.
- **Swaps.** When nothing fits any more, `greedy` swaps a bought option for another of its conflict sets, the best swap that fits and scores more, then buys again what fits, and so on until no swap scores more. `static` and `greedy --worst` make no swap.

On [`three-woods-works.json`](../examples/three-woods-works.json) at a budget of 7, both heuristics buy the free `wood_c_intact` first, then the hedgerow and the plantation (28.76). `static` stops there. `greedy` then swaps `wood_c_intact` for the underpass and reaches the optimum, 31.36.

The swaps change one option at a time. An option in a conflict set with each of two bought options is never reached: with [harms of different extents](../usage/modeling.md#actions-that-harm-the-landscape), the heuristics keep both layers of the wood and never buy the action that destroys them. `greedy --worst` is not a lower reference either: on the conflicts instance of the repository, `test/instances/conflicts`, it scores 28.76 at `money=7,area=5`, more than `greedy` (25.6), which the swaps do not get out of.

`--decremental` starts from every option bought, which buys every conflict set twice over, and does not handle these instances: it stops with the error `greedy --decremental: conflict sets and several cost dimensions are not supported, run greedy without --decremental`, and the same for `static`. The check is made on the instance once the options over the budget are removed, so a budget that leaves no conflict set with two affordable options still runs.

## Worst cases

`greedy --worst` is the mirror image of the greedy heuristic: it buys at each step the option of *smallest* gain and, with `--decremental`, removes at each step the option of *largest* loss. It spends the budget while gaining as little as possible, and serves as a reference: the gap between its score and the optimum is the range within which the choice of the options matters.

## Parameters

The heuristics take `-t`, the tolerance used when comparing a cost with the budget ($10^{-7}$ by default, in every dimension), `--decremental` and, with it, `--only-dec`, both with a single cost and no conflict set; `greedy` also takes `--worst`. They always end with the status `completed`: Ctrl-C does not interrupt them.

## Reference

F. Hamonic, B. Couëtoux, Y. Vaxès and C. H. Albert (2023). Cumulative effects on habitat networks: how greedy should we be? *Biological Conservation*, 282, 110066. [doi:10.1016/j.biocon.2023.110066](https://doi.org/10.1016/j.biocon.2023.110066)
