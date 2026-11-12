# Heuristics

The heuristics build a solution by ranking the options by what they gain per unit of cost. They need no solver, evaluate any criterion, and run on landscapes that no exact algorithm can handle, but they give **no guarantee**: their solution can be far from the optimum, and nothing in their output tells how far. A [bound](bounds.md) tells it afterwards.

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

## The four heuristics

**`static_incr`** starts from the landscape as it is, computes the gain $\Delta^+_i(0)$ of every option once, and goes through the options by decreasing gain, buying each one that fits in what is left of the budget. It ranks the options as if they did not interact: $|\Phi|$ evaluations.

**`greedy_incr`** starts from the landscape as it is and repeats: compute the gain $\Delta^+_i(x)$ of each option that still fits in the budget, in the current scenario $x$, and buy the best one. It stops when no option fits. It sees the options that become worth more once others are bought, but only one step ahead: about $|\Phi|^2 / 2$ evaluations at most.

**`static_decr`** starts from the scenario $\overline{x}$ that buys every option, computes the loss $\Delta^-_i(\overline{x})$ of every option once, and removes the options by increasing loss until the cost fits the budget. The last removal can leave some budget unused: the options removed that fit in it are then bought back, by decreasing gain in the scenario reached.

**`greedy_decr`** starts from the scenario that buys every option and repeats: compute the loss $\Delta^-_i(x)$ of each option bought, and remove the one whose loss is the smallest, until the cost fits the budget. It then spends what is left of the budget as `greedy_incr` does, among the options removed. This is the principle of the Zonation algorithm.

`--only-dec` skips the final purchases of the two decremental heuristics.

The incremental heuristics, and these final purchases, buy as long as an option fits, even one that gains nothing.

## Why they can miss the optimum

A ranking cannot see that two options are worth buying together, nor that two options do the same job. On the [three woods](../getting-started/first-run.md) with a budget of 3, the gains per unit of cost in the initial landscape are

| Option | Cost | Score alone | Gain per unit of cost |
|---|---:|---:|---:|
| `plantation` | 2 | 24.6 | 3.1 |
| `underpass` | 3 | 25.6 | 2.4 |
| `hedgerow` | 2 | 20.96 | 1.28 |

so the incremental heuristics buy `plantation` first, after which nothing fits, and the decremental ones end on the same solution: a score of 24.6, with a third of the budget unused, while the optimum is `underpass`, at 25.6. On the 40 × 40 raster landscape of the repository (`test/instances/Aix/instance_40x40`, 27 options) at budget 8, the four heuristics reach 75 % to 98.7 % of the optimal gain (one run each on an 8-core desktop computer, the optimum with Gurobi):

| Algorithm | Score | Gain | Time |
|---|---:|---:|---:|
| `static_decr` | 418,296.9 | +17.42 % | 42 ms |
| `static_incr` | 432,401.8 | +21.38 % | 28 ms |
| `greedy_incr` | 437,866.4 | +22.91 % | 0.2 s |
| `greedy_decr` | 437,866.4 | +22.91 % | 0.5 s |
| optimum (`multiflow_benders`) | 438,893.1 | +23.20 % | 1.2 s |

The greedy heuristics are often close, as here, and sometimes far: Hamonic et al. (2023) compare them with the optimum on real landscapes. A cheap way to use them safely is to pass their solution to an exact algorithm, with `--mip-start-algorithm greedy_incr`: the exact algorithm starts from it and, if it runs out of time, returns the best solution it has with a proven bound.

## Worst cases

`worst_greedy_incr` and `worst_greedy_decr` are the mirror images of the greedy heuristics: the first buys at each step the option of *smallest* gain, the second removes at each step the option of *largest* loss. They spend the budget while gaining as little as possible, and serve as a reference: the gap between their score and the optimum is the range within which the choice of the options matters.

## Parameters

The heuristics take `-t`, the tolerance used when comparing a cost with the budget ($10^{-7}$ by default), and the two decremental ones take `--only-dec`. They always end with the status `completed`: Ctrl-C does not interrupt them.

## Reference

F. Hamonic, B. Couëtoux, Y. Vaxès and C. H. Albert (2023). Cumulative effects on habitat networks: how greedy should we be? *Biological Conservation*, 282, 110066. [doi:10.1016/j.biocon.2023.110066](https://doi.org/10.1016/j.biocon.2023.110066)
