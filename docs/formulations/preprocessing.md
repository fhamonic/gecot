# Preprocessing

The [flow formulation](flow.md) has one flow per target vertex, each on the whole graph: on a landscape of a few thousand vertices it has millions of variables. GECOT reduces it in two stages, without changing the optimum:

1. the **instance** is simplified once, for every algorithm;
2. each target $t$ gets its own **reduced graph** $G_t$, for the algorithms that route flows: `flow_mip`, `multicut_benders`, `flow_benders`, `multiflow_benders` and `tree_lp`.

Every step can be switched off to measure what it brings: the [last section](#what-each-step-brings) maps the steps to their switches.

## The instance given to the algorithms

Before any algorithm runs, `gecot optimize` and `gecot bound` rewrite the instance. The results are always reported for the instance as it was given: every one of its options is listed, and the scores are in its unit.

**Options of cost 0 are bought.** They are applied to the graphs, and the solution lists them as bought. The *initial score* is still the score of the landscape without them.

**Options that cost more than the budget are removed.**

**Vertices linked by arcs of probability 1 are merged.** If two vertices $u$ and $v$ are connected with probability $\Pi(u,v) = \Pi(v,u) = 1$, the flow passes from one to the other without loss, so $f_u = f_v$ in every scenario. The same holds for a set $T$ of vertices that are pairwise connected with probability $1$: it is contracted into a single vertex $m$ with

$$
\begin{aligned}
w_m &= \sum_{u \in T} w_u\,, & q_m &= \sum_{u \in T} q_u\,, \\
w^i_m &= \sum_{u \in T} w^i_u\,, & q^i_m &= \sum_{u \in T} q^i_u \qquad (i \in \Phi)\,,
\end{aligned}
$$

and the score is unchanged in every scenario. These sets are the strongly connected components of the graph of the arcs of probability $1$, found in $O(|V| + |A|)$ time. This is what makes raster landscapes tractable: the cells of a habitat patch, linked by arcs of probability $1$, become one vertex. The arcs that a free option raises to probability $1$ count; the arcs that only a paid option raises to $1$ do not, as the merge must hold in every scenario. `gecot optimize --no-merging` keeps the vertices apart.

**Qualities are rescaled.** Solvers work with absolute tolerances, which are too coarse for scores of $10^{11}$ and too fine for scores of $10^{-2}$. All the qualities are divided by a power of two chosen so that the largest initial score of a case lies in $[10^5, 4 \cdot 10^5)$; an instance whose cases score less is left as it is. The scores are multiplied back before they are printed. Only the solver's own log (`-vv`) is in the scaled unit.

If no option is left, there is nothing to decide: the run ends with the status `no option left`.

### Removing options with the path relaxation

`gecot optimize --reformulation path_lp` goes one step further. It first runs the [path relaxation](relaxations.md), which returns a solution of score $s$ and, for each option $i$, an upper bound $b_i$ on the score of every solution that buys $i$. An option with $b_i \le s$ is in no solution better than the one in hand, so it is removed. The algorithm then runs on the options left, starting from the solution of the relaxation, and the result is the better of the two solutions. When no option is left, or when the bound of the relaxation already equals $s$, the solution is optimal and the algorithm is not run. `--reformulation-params` passes parameters to this `path_lp`, as in `--reformulation-params "time-limit=60 pricing=enumerate"`; its MIP start is the solution of `greedy` unless told otherwise.

## The graph of a target

The rest of this page considers one case and one vertex $t$ of its graph $G = (V,A)$, and the flow that computes $f_t$ in every scenario. The aim is a smaller graph $G_t$ on which this flow has the same optimum, for every scenario within the budget.

### Vertices that are never targets

If $\overline{q}_t = 0$, the vertex $t$ has no target quality and no option gives it any: $q^x_t \cdot f^x_t = 0$ in every scenario, and $t$ needs no flow at all. On a raster landscape most vertices are in this case: they are only places that individuals cross. `--all-targets` gives them a flow anyway.

### Strong and useless arcs

In every scenario $x$, an optimal flow toward $t$ sends the quality of each vertex along a most reliable path. Some arcs are never on such a path, and some always are:

- an arc $(u,v)$ is **$t$-useless** if, in every scenario, every path from $u$ to $t$ that starts with $(u,v)$ is strictly less reliable than a most reliable one;
- an arc $(u,v)$ is **$t$-strong** if, in every scenario, some most reliable path from $u$ to $t$ starts with $(u,v)$.

With $\Pi^x$ the probability of connection in the scenario $x$, the sets of these arcs are

$$
\begin{aligned}
\mathbb{W}(t) &= \big\{ (u,v) \in A \;:\; \forall x \in \{0,1\}^\Phi\,,\;\; \Pi^x(u,t) > \pi^x_{uv} \cdot \Pi^x(v,t) \big\}\,, \\
\mathbb{S}(t) &= \big\{ (u,v) \in A \;:\; \forall x \in \{0,1\}^\Phi\,,\;\; \Pi^x(u,t) = \pi^x_{uv} \cdot \Pi^x(v,t) \big\}\,.
\end{aligned}
$$

<figure class="gecot-figure">
<svg viewBox="0 0 560 170" width="560" role="img" aria-label="Two graphs on the vertices u, v and t. On the left the arc from u to v is useless for t, on the right it is strong for t.">
  <defs>
    <marker id="su-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0 L10 5 L0 10 z" fill="currentColor"/></marker>
    <marker id="su-arrow-accent" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path class="improved-fill" d="M0 0 L10 5 L0 10 z"/></marker>
  </defs>
  <g font-size="13" fill="currentColor" stroke="none">
    <!-- (a) useless -->
    <line class="improved" x1="52" y1="109" x2="117" y2="52" stroke="currentColor" stroke-width="1.6" marker-end="url(#su-arrow-accent)"/>
    <line x1="143" y1="52" x2="208" y2="109" stroke="currentColor" stroke-width="1.6" marker-end="url(#su-arrow)"/>
    <line x1="57" y1="120" x2="202" y2="120" stroke="currentColor" stroke-width="1.6" marker-end="url(#su-arrow)"/>
    <circle class="habitat" cx="40" cy="120" r="16" stroke-width="1.5"/>
    <circle class="habitat" cx="130" cy="40" r="16" stroke-width="1.5"/>
    <circle class="habitat" cx="220" cy="120" r="16" stroke-width="1.5"/>
    <text class="habitat-label" x="40" y="124.5" text-anchor="middle" font-style="italic">u</text>
    <text class="habitat-label" x="130" y="44.5" text-anchor="middle" font-style="italic">v</text>
    <text class="habitat-label" x="220" y="124.5" text-anchor="middle" font-style="italic">t</text>
    <text x="70" y="70" text-anchor="end">[0.5, 0.8]</text>
    <text x="190" y="70" text-anchor="start">0.9</text>
    <text x="130" y="139" text-anchor="middle">0.9</text>
    <text x="130" y="164" text-anchor="middle">(a)</text>
    <!-- (b) strong -->
    <line x1="352" y1="109" x2="417" y2="52" stroke="currentColor" stroke-width="1.6" marker-end="url(#su-arrow)"/>
    <line x1="443" y1="52" x2="508" y2="109" stroke="currentColor" stroke-width="1.6" marker-end="url(#su-arrow)"/>
    <line class="improved" x1="357" y1="120" x2="502" y2="120" stroke="currentColor" stroke-width="1.6" marker-end="url(#su-arrow-accent)"/>
    <circle class="habitat" cx="340" cy="120" r="16" stroke-width="1.5"/>
    <circle class="habitat" cx="430" cy="40" r="16" stroke-width="1.5"/>
    <circle class="habitat" cx="520" cy="120" r="16" stroke-width="1.5"/>
    <text class="habitat-label" x="340" y="124.5" text-anchor="middle" font-style="italic">u</text>
    <text class="habitat-label" x="430" y="44.5" text-anchor="middle" font-style="italic">v</text>
    <text class="habitat-label" x="520" y="124.5" text-anchor="middle" font-style="italic">t</text>
    <text x="370" y="70" text-anchor="end">0.8</text>
    <text x="490" y="70" text-anchor="start">0.8</text>
    <text x="430" y="139" text-anchor="middle">[0.2, 0.5]</text>
    <text x="430" y="164" text-anchor="middle">(b)</text>
  </g>
</svg>
<figcaption>The orange arcs can be improved, within the interval shown. (a) The arc from u to v is t-useless: through it u reaches t with probability at most 0.8 × 0.9 = 0.72, less than the 0.9 of the direct arc. (b) The arc from u to v is t-strong: through it u reaches t with probability 0.64, never less than the direct arc.</figcaption>
</figure>

Deciding these properties over the scenarios of $\{0,1\}^\Phi$ is hard, because an option improves several arcs at once and several options improve the same arc. GECOT relaxes the scenarios: the probability of each arc $a$ may take any value of $[\pi_a, \overline{\pi}_a]$, independently of the other arcs. A property that holds for all these *continuous scenarios* holds for the scenarios of $\{0,1\}^\Phi$, which are among them, so the sets $W(t)$ and $S(t)$ defined with the continuous scenarios satisfy

$$
W(t) \subseteq \mathbb{W}(t)\,, \qquad S(t) \subseteq \mathbb{S}(t)\,.
$$

!!! note "Difference with the thesis"

    The thesis writes the definition of $\mathbb{W}(t)$ with the inequality $\Pi^x(u,t) < \pi^x_{uv} \cdot \Pi^x(v,t)$, which no arc can satisfy. The definition above is the one its text and its algorithm use: the paths through $(u,v)$ are strictly less reliable than the best.

### Interval lengths and fibers

Most reliable paths are shortest paths for the lengths $-\log \pi$. Give each arc $a$ the interval of lengths

$$
[\,l^-_a\,,\, l^+_a\,] = [\,-\log \overline{\pi}_a\,,\, -\log \pi_a\,]\,,
$$

and call scenario any choice of a length $l^x_a$ in the interval of each arc, with $d^x(s,t)$ the length of a shortest path from $s$ to $t$ under it. The **fiber** of an arc $(u,v)$ under the scenario $x$ is the set of vertices that $u$ reaches by a shortest path through $(u,v)$,

$$
\begin{aligned}
F^x(u,v) &= \big\{ t \in V \;:\; d^x(u,t) = l^x_{uv} + d^x(v,t) \big\}\,, \\
F(u,v) &= \bigcap_{x} F^x(u,v)\,,
\end{aligned}
$$

so that $(u,v)$ is $t$-strong exactly when $t \in F(u,v)$. It is enough to look at the scenarios where each length is at one end of its interval, and in fact at a single one.

!!! abstract "Lemma"

    $F(u,v) = F^y(u,v)$ for the scenario $y$ in which the arc $(u,v)$ and the arcs that leave a vertex of $F(u,v)$ have their upper length $l^+$, and every other arc has its lower length $l^-$.

The scenario $y$ is the worst one for the paths through $(u,v)$: they are as long as possible, and the paths that avoid them as short as possible. It is defined from the fiber it yields, and both are computed together by an adaptation of Dijkstra's algorithm that colours the vertices as it settles them, by increasing distance from $u$:

- $v$ starts **blue** at distance $l^+_{uv}$, and the other vertices that an arc of $u$ enters start **red**, at the lower length of this arc;
- a blue vertex relaxes its outgoing arcs with their upper lengths and colours blue the vertices it improves, a red vertex relaxes them with their lower lengths and colours them red;
- when a blue and a red vertex are at the same distance, the blue one is settled first.

The blue vertices are $F(u,v)$. A symmetric version computes the vertices $t$ for which $(u,v)$ is $t$-useless: the arc $(u,v)$ is red with its lower length, the other arcs that leave $u$ are blue with their upper lengths, and ties go to red, so that blue vertices are those that a path avoiding $(u,v)$ always reaches strictly sooner. Each run takes $O(|A| + |V| \log |V|)$ time. GECOT runs both versions for every arc, in parallel, hence

$$
O\big( |A|^2 + |A| \cdot |V| \log |V| \big)
$$

for all the sets $S(t)$ and $W(t)$ of a case, which is $O(|V|^2 \log |V|)$ on the planar graphs that landscapes usually are. Only the options that the budget can afford count in $\overline{\pi}$.

### Resolution

The lengths are integers: with $r$ the value of `--resolution` ($10^{-8}$ by default),

$$
l = \left\lfloor \frac{-\ln \max\{\pi, r\}}{\ln(1 + r)} \right\rfloor\,.
$$

A probability below $r$ is thus treated as $r$, and two paths are told apart only if their probabilities differ by a factor of about $1 + h \cdot r$, for paths of $h$ arcs. Integer lengths make the comparisons of the labelling exact, at the price of this resolution: an arc can be labelled against a path that beats it by less. The optimum that the algorithms prove is therefore exact *up to the resolution*. This matters only when the gains at stake are of that order, for instance a budget that only buys options that are negligible at the scale of the landscape: then compare with `--preprocessing none`, which uses no label.

The resolution must lie strictly between 0 and 1, and GECOT also refuses one so small that the lengths of a path could overflow their 64-bit integers: from about $5 \cdot 10^{-12}$ down on a graph of a million vertices, $5 \cdot 10^{-14}$ on one of ten thousand. Between these limits any value is accepted, however coarse: keep it well below the probabilities of the instance, which the labels otherwise no longer tell apart.

### Removing the useless arcs

!!! abstract "Lemma"

    Let $(u,v) \in \mathbb{W}(t)$ and let $G'$ be $G$ without the arc $(u,v)$. For every scenario $x$, $f^x_t(G) = f^x_t(G')$.

No most reliable path to $t$ uses $(u,v)$, so removing it changes no probability of connection to $t$. $G_t$ is built without the arcs of $W(t)$, which also removes their improved copies and big-M constraints.

### Contracting the strong arcs

Let $(u,v) \in \mathbb{S}(t)$ be an arc that no option improves. All the quality that reaches $u$ can go on to $v$ through $(u,v)$ in every scenario, so $u$ is merged into $v$:

- each arc $(s,u)$ entering $u$ becomes an arc $(s,v)$ of probability $\pi_{su} \cdot \pi_{uv}$, and so do its improved copies, with $\pi^i_{su} \cdot \pi_{uv}$;
- the source quality of $u$ moves to $v$, discounted by the arc: $w'_v = w_v + \pi_{uv} \cdot w_u$, and $w'^{\,i}_v = w^i_v + \pi_{uv} \cdot w^i_u$ for each option $i$;
- $u$ and the arcs that leave it are removed.

!!! abstract "Lemma"

    Let $(u,v) \in \mathbb{S}(t)$ and let $G'$ be the graph obtained by contracting $(u,v)$. For every scenario $x$, $f^x_t(G) = f^x_t(G')$.

For a vertex $s \notin \{u, v\}$ the distance to $t$ is the same in both graphs, and for $u$ and $v$

$$
\begin{aligned}
w^x_u \cdot \Pi^x(u,t) + w^x_v \cdot \Pi^x(v,t)
&= \big( \pi_{uv} \cdot w^x_u + w^x_v \big) \cdot \Pi^x(v,t) \\
&= w'^{\,x}_v \cdot \Pi^x(v,t)\,.
\end{aligned}
$$

On a landscape, most arcs far from the options are strong for most targets: whole regions collapse onto the few vertices where the options leave a choice.

### Pruning

Two more removals cost nothing:

- the arcs that leave $t$, since no optimal flow leaves its target;
- the vertices that no quality can reach: a vertex carries flow only if it is on a path from a vertex $s$ with $\overline{w}_s > 0$, which a graph traversal from these vertices finds in $O(|V| + |A|)$ time.

`--no-pruning` keeps both.

### Taking the budget into account

The labels above hold for every scenario, including those that buy every option. Yet only the scenarios within the budget matter: an arc $(u,v)$ is **$t$-$\beta$-strong** if it is on a most reliable path from $u$ to $t$ in every scenario of cost at most $\beta$, and more arcs are. Recognizing them exactly is NP-hard, as it contains the constrained shortest path problem, but a Lagrangian relaxation finds some of them.

Let $A_i$ be the set of arcs that the option $i$ improves, and $l^i_a = -\log \pi^i_a$. For a multiplier $\mu \ge 0$, charge each option its cost, shared among its arcs, in the lengths:

$$
\lambda^\mu_a = \min\Big\{ l^+_a\,,\; \min_{i \,:\, a \in A_i} \Big( l^i_a + \mu \cdot \frac{c_i}{|A_i|} \Big) \Big\}\,.
$$

For a scenario $x$ of cost at most $\beta$ and a simple path $Q$, the options that $x$ buys pay at most $\mu \cdot \beta$ along $Q$, so $l^x(Q) \ge \lambda^\mu(Q) - \mu \cdot \beta$. The colouring algorithm run with the lower lengths $\lambda^\mu$, the paths through $(u,v)$ being handicapped by $\mu \cdot \beta$, therefore yields labels that are valid within the budget. Each multiplier gives its own labels, and they accumulate: $\mu = 0$ gives the labels of the previous sections, and larger multipliers add to them.

`--nb-mus N` turns this on, with $N + 1$ positive multipliers in geometric progression up to the largest useful one, after $\mu = 0$. It is experimental, and off by default ($N = 0$):

- it helps only when the budget is small in absolute terms, so that few options fit together; where it finds nothing it only adds preprocessing time, since each multiplier costs one more pass;
- `--nb-mus 20` suits graphs where a pass takes well under half a second, `--nb-mus 9` larger ones; compare with `--nb-mus 0` on your instance before relying on it;
- `--mus-sweep decades`, the default, spreads the multipliers over at most the three decades below the largest useful one and tests $\mu = 0$ once more at the end; `--mus-sweep full` starts from the inverse of the largest cost per arc.

## Big-M constants

The constant $M^t_u$ of the constraints $\phi^t_{a^i} \le M^t_u \cdot x_i$ and $f^i_t \le M^t_t \cdot x_i$ must be at least the flow that can reach the vertex $u$ on its way to $t$. The smaller it is, the tighter the linear relaxation, and the faster the search: with a constant $100$ times too large, $x_i = 0.01$ is enough to open an arc completely. `--big-m` chooses among three constants, all computed on the reduced graph $G_t$, where $\overline{\Pi}_{G_t}(s,u)$ is the probability of connection in $G_t$ with every option bought.

**`plain`** counts every quality gain:

$$
M^t_u = \sum_{s \in V(G_t)} \overline{w}_s \cdot \overline{\Pi}_{G_t}(s,u)\,.
$$

It is one run of Dijkstra's algorithm from $u$ on the reversed graph. As $G_t$ has fewer arcs than $G$ and keeps every most reliable path to $t$, this bound is no larger than the same sum on $G$.

**`knapsack`**, the default, counts only the gains that the budget can buy together. With $g_i = \sum_{s} w^i_s \cdot \overline{\Pi}_{G_t}(s,u)$ the most that the option $i$ can bring to $u$,

$$
\begin{aligned}
M^t_u = {} & \sum_{s \in V(G_t)} w_s \cdot \overline{\Pi}_{G_t}(s,u) \\
& + \max\Big\{ \sum_{i \in \Phi} g_i \cdot x_i \;:\; \sum_{i \in \Phi} c_i \cdot x_i \le \beta\,,\; x \in \{0,1\}^\Phi \Big\}\,,
\end{aligned}
$$

a knapsack problem solved exactly by branch and bound for each vertex that needs a constant.

**`tight`** (experimental) lowers the knapsack constant of each improved copy $a^i$ of an arc $(u,v)$, by keeping only the sources that can use it, those whose best path through $a^i$ with every option is at least as reliable as their best path without any option, and by bounding the probabilities within the budget left by $i$ with a Lagrangian relaxation.

`--big-m` is a parameter of `flow_mip`, `multicut_benders` and `flow_benders`. `multiflow_benders` has its own constants, one per source and arc, and `tree_lp` uses the plain ones.

## What each step brings

| Step | Switch that disables it | Where |
|---|---|---|
| Merging the vertices linked with probability 1 | `--no-merging` | `gecot optimize`, every algorithm |
| Skipping the vertices that are never targets | `--all-targets` | `flow_mip`, the three Benders decompositions, `tree_lp` |
| Removing the useless arcs | `--preprocessing strong` or `none` | same |
| Contracting the strong arcs | `--preprocessing useless` or `none` | same |
| Pruning | `--no-pruning` | same |
| Budget-aware labels | off unless `--nb-mus N` | same |
| Big-M constants within the budget | `--big-m plain` | `flow_mip`, `multicut_benders`, `flow_benders` |

`--preprocessing` takes `all` (the default), `strong` (only the contraction), `useless` (only the removal) or `none`. Both kinds of labels are computed together, so `strong` and `useless` measure the size of the model and the time to solve it, not the time to label.

As an order of magnitude, here is `flow_mip` on a 30 × 30 raster landscape of the repository (`test/instances/Aix/instance_30x30`, 2,700 vertices, 5,280 arcs, 18 options) at budget 8, with Gurobi on an 8-core desktop computer, one run each. Every run returns the same optimum.

| Setting | Variables | Constraints | Time |
|---|---:|---:|---:|
| default | 22,817 | 11,116 | 1.0 s |
| `--big-m plain` | 22,817 | 11,116 | 0.9 s |
| `--no-pruning` | 29,839 | 14,003 | 0.9 s |
| `--preprocessing strong` | 43,209 | 13,469 | 1.8 s |
| `--preprocessing useless` | 55,564 | 39,149 | 1.3 s |
| `--preprocessing none` | 179,471 | 86,124 | 6.3 s |
| `--no-merging` | 481,084 | 182,938 | 4.6 s |
| `--all-targets` | 814,021 | 422,885 | 5.2 s |

The instance is small enough for every setting to finish, which larger ones do not allow. The effect of each step depends on the instance: merging matters on rasters, labels where the options are few and local, and the big-M constants when many options carry quality gains.
