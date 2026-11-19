# `onaga` for melon

**Done, 2026-11-19:** melon 1.1.0 has `melon::onaga` (melon's commit 6b9704b), with the sparse reset and the flow lists as switches of its traits (`sparse_reset`, `flowing_arc_lists`), on by default. gecot uses it: its own header and tests are removed, and the paths below are those of before the move.

For the maintainer of melon: what `include/gecot/algorithm/onaga.hpp` is, what moving it into melon takes, and the choices left open. The header was written in melon's style for that move; gecot uses it in `multiflow_target_block.hpp`, and its tests are `test/onaga_test.cpp`.

## What it computes

A generalized maximum flow in a lossy network: each arc a = (u, v) has a gain 0 < g_a ≤ 1 and a capacity c_a, and carries 0 ≤ f_a ≤ c_a measured at u, of which g_a · f_a arrives at v. The source s sends at most its supply, the other vertices may keep flow but not create it, and the value is the flow that reaches t:

$$
\begin{aligned}
\max_{f} \quad & \sum_{a \in \delta^-(t)} g_a f_a - \sum_{a \in \delta^+(t)} f_a \\
\text{s.t.} \quad & \sum_{a \in \delta^+(v)} f_a \le \sum_{a \in \delta^-(v)} g_a f_a && \forall v \ne s, t, \\
& \sum_{a \in \delta^+(s)} f_a - \sum_{a \in \delta^-(s)} g_a f_a \le \text{supply}, \\
& 0 \le f_a \le c_a && \forall a.
\end{aligned}
$$

The algorithm is Onaga's (1966): augment along a highest-gain residual path from s to t until the supply is sent or no path remains. A residual arc taken backward has the gain 1/g_a > 1, so each path is found by Dijkstra's algorithm on gains relabeled by vertex labels p, g_a · p_v / p_u ≤ 1, as Tardos and Wayne (1998) describe it; after a search the settled vertices take p_u · D_t / D_u, which keeps the labels feasible and makes the path tight. A last Dijkstra from t gives `potential(v)`, the gain of a best residual path from v to t. With the arc duals (g_a · potential(v) − potential(u))^+ on the arcs of finite capacity, these form the least optimal dual solution once the run has converged, and a feasible one after a run cut short, whose value bounds the maximum.

- K. Onaga (1966). Dynamic programming of optimum flows in lossy communication nets. *IEEE Transactions on Circuit Theory*, 13(3), 282–287. [doi:10.1109/TCT.1966.1082612](https://doi.org/10.1109/TCT.1966.1082612)
- É. Tardos and K. D. Wayne (1998). Simple generalized maximum flow algorithms. *IPCO 1998*, LNCS 1412, 310–324. [doi:10.1007/3-540-69346-7_24](https://doi.org/10.1007/3-540-69346-7_24)

The number of augmentations can be exponential, and infinite with irrational capacities. `advance()` makes one, so that a caller can bound them; gecot stops at 100,000 and keeps the potentials of the residual graph reached.

## The move

- **The header** becomes `include/melon/algorithm/onaga.hpp`: `namespace melon`, the `melon::` qualifiers dropped, and the first comment, which says it is a draft, removed. It already follows the conventions of `CONTRIBUTING.md`: a roles struct (`onaga_roles`) for every map, the traits concept and default traits, `heap_index_map_agrees` next to the entry-type assert, the structural dividers, `graph_all`/`mapping_all` with the deduction guides, no reserved identifiers.
- **`include/melon/all.hpp`:** the include, between `network_voronoi.hpp` and `strongly_connected_components.hpp`.
- **The tests:** `test/onaga_test.cpp` becomes `test/onaga.cpp`, with `GTEST_TEST` for `TEST` and the file added to `melon_test` in `test/CMakeLists.txt`. They use `static_digraph` and `maps::function` only. They check certificates rather than values: on random lossy networks, a feasible flow and a feasible dual of equal value; that a reused object gives a fresh one's results bit for bit, with and without start labels and with tied gains; that a run cut short bounds the maximum; that `compute_potentials()` between augmentations changes no flow; that infinity and the largest finite capacity give the same results; and that `reset()` and `set_source()` drop the state that a run leaves for its next search. Mutations of each mechanism listed below fail them, by a failure, a hang or a crash.
- **The docs:** a section "Generalized maximum flow" in `docs/algorithms/flows-and-trees.md`, after "Maximum flow": the program above, the members, and the example below. The first paragraph of "Maximum flow" says that the residual network is walked both ways; the same requirements hold here.
- **`CHANGELOG.md`:** a line under "Added".

```cpp
#include "melon/algorithm/onaga.hpp"

std::vector<double> gain = ...;       // one per arc, in (0, 1]
std::vector<double> capacity = ...;   // one per arc, infinity for none

onaga alg(graph, gain, capacity, s, t, supply);
alg.run();

std::println("flow reaching t = {}", alg.flow_value());
for(auto && v : vertices(graph)) std::print(" {}", alg.potential(v));
```

| Member | Effect |
| --- | --- |
| `onaga(graph, gains, capacities)`, `onaga(graph, gains, capacities, s, t, supply)` | construct; the gain and capacity maps are read live |
| `set_source(s, supply)`, `set_target(t)` | change the terminals; they keep the flow |
| `set_start_labels(labels)` | labels to start each run from, 1 by default; O(n) |
| `reset()` | zero the flow, keep the graph, maps and start labels; costs what the last run reached |
| `finished()`, `advance()` | whether the run is over; one augmentation, or none and `finished()` |
| `run()` | augment until finished, then compute the potentials if they are stale; idempotent |
| `compute_potentials()` | the last search, for a run driven by `advance()` |
| `flow_value()`, `remaining_supply()` | the flow that reached t, the supply not sent |
| `flow(a)`, `flows_map()` | the flow of an arc, measured at its source |
| `potential(v)`, `potentials_map()` | the dual; valid until the next `advance()` or `reset()`, as asserted |

The lifecycle is melon's (`docs/algorithms/index.md`): `reset()` gives a fresh object's results, the setters and `reset()` come in either order, and `run()` drains and is idempotent; `dinitz`'s loop `alg.reset().set_source(s, supply).set_target(t).run();` is the pattern.

## Contract

- **Requirements:** a `graph_view` with `outward_incidence_graph`, `inward_incidence_graph`, `has_arc_source`, `has_arc_target`, `has_vertex_map` and `has_arc_map`; gain and capacity maps modelling `mapping_view` over the arcs, whose common value type is a floating-point type. Flows are fractional whatever the capacities, so integral types are rejected at the constraint.
- **Preconditions no concept can check**, in the header's comment: gains in (0, 1] (a gain above 1 can close a flow-generating cycle that augmenting paths do not cancel: the flow is then not maximum, silently); start labels feasible, label(t) = 1 and label(u) ≥ g_a · label(v) on every arc of positive capacity; maps unchanged during a run.
- **Capacities:** infinity is unbounded and 0 removes the arc, so a run on a subgraph needs no copy of the graph. This is the convention of `dinitz` and `edmonds_karp`, whose capacity type must have a usable infinity.
- **Traits:** the heap (melon's 4-ary updatable heap, a max-heap on the relabeled gains, since a min-heap would settle the worst paths first), `arc_full` and `negligible_flow`. The last two are relative: an arc is full within `arc_full` of its capacity, and a flow under `negligible_flow` times the supply is none. At 0, the residue of a push that fills an arc would leave it open and the searches would keep augmenting along it.

## What a reviewer needs to know of the implementation

The cost of a run is what its searches reach, not the size of the graph. Each of the five mechanisms below keeps it so, and a comment at each states the invariant it relies on.

- **Status epochs.** A vertex is in the heap when its status is the epoch of the search, settled at the epoch + 1, and unreached below. Each search takes the epoch 2 above the last instead of clearing the statuses, which happens once in 127 searches; the gains from s and the potentials are read only through the statuses.
- **Sparse reset.** `reset()` restores the labels and clears the chains of the vertices the searches settled (`_changed`), and the flows of the arcs that took flow (`_flowing`). The start labels are kept in their own map for that restore.
- **Chains of the arcs that carry flow.** An arc that takes flow is linked into the chain of its target's arcs in and of its source's arcs out. An arc can be taken backward only if it carries flow, so a search scans these chains instead of every in-arc (from s) or out-arc (from t) of a vertex.
- **Reuse of the last tree.** After the rescale, every arc of the tree of the last search has relabeled gain 1, the largest there is, and the augmentation closes only its bottleneck. The next search would therefore pop the surviving tree first, and settles it directly, without the heap, in its previous order. `reset()`, the setters and a failed search drop the tree.
- **A cancelled flow clamped at 0.** An arc taken backward as the bottleneck gives up its whole flow, less the rounding of the gains along the path, which can leave it an ulp below 0.

The chains and the tree reuse change the order in which ties are broken, compared with scanning every arc and with a heap holding the whole tree. The flow value and the potentials of a converged run are the same up to the rounding of the ties; a run cut short can follow another of the best paths.

## Measured in gecot

In gecot's Benders separation, which runs one source after the other against a target, the three changes of 2026-11-19 (the chains, the tree reuse, the open test in one comparison) took 14.3 % less separation CPU and 5.0 % less time on its benchmark suite (31 runs, three interleaved passes of timed builds, the same scores), and nothing on its largest instance, whose searches from s are 1 % of the separation (+0.3 % per source). The earlier sections of `misc/design/multiflow_benders_separation.md` record the epochs, the sparse reset and the alternatives that were measured and not kept: prefetches, a flat array for `_changed`, a fill of the flows, other heaps, an array-of-structs record.

## Left open for the maintainer

- **Names:** `potential` for the dual, against `label` internally; `onaga` for the class, which names the algorithm as `dinitz` and `edmonds_karp` do.
- **`compute_potentials()` public**, for a run cut short by `advance()` calls. `run()` calls it.
- **No `&&` overloads** of `flows_map()` and `potentials_map()`, which `dinitz` and `network_simplex` have.
- **The heap is constructed with `std::greater<value_t>{}`**, whatever comparator the traits' heap declares: a traits heap with another comparator does not compile.
- **One source.** Several sources would need a super source, which the gains make awkward: the supply of each source is in its own units.
- **Tolerances in the traits**, compile-time constants, rather than set at construction.
- **A cheaper dual, not provided:** the labels left by the searches, with those of a failed search's reach set to 0, are an optimal dual too, and need no last search. They are not the least dual, and in gecot their cuts were weaker, so the header does not expose them.
