# Problem and notation

This page states the optimization problem that GECOT solves and fixes the notation of the other pages. It follows the formalism of the thesis and of the article cited on the [home page](../index.md#citation), extended in two directions: a vertex has one quality as a source and another as a target, and several landscape graphs (the *cases*, one per species for instance) share the same options. The [notation page](../reference/notation.md) lists every symbol next to the file column or the command line parameter that sets it.

## Landscape graph

A landscape is modelled, for one species, by a directed graph $G=(V,A)$. A vertex is a habitat patch, or a cell of a raster, and an arc $(u,v)$ is a move that an individual can attempt from $u$ to $v$.

- Each arc $a \in A$ has a **probability** $\pi_a \in [0,1]$, the probability that the move succeeds.
- Each vertex $u \in V$ has a **source quality** $w_u \ge 0$ and a **target quality** $q_u \ge 0$: what $u$ is worth as a place that individuals leave, and as a place that they reach.

The probability of an $st$-path is the product of the probabilities of its arcs, and $\Pi(s,t)$ denotes the probability of a *most reliable path* from $s$ to $t$:

$$
\Pi(s,t) = \max_{P\,:\,st\text{-path}} \; \prod_{a \in P} \pi_a\,,
$$

with $\Pi(s,s) = 1$, and $\Pi(s,t) = 0$ when $t$ cannot be reached from $s$. With the lengths $l_a = -\log \pi_a$ a most reliable path is a shortest path, and $\Pi(s,t) = e^{-d(s,t)}$.

The **score** of a landscape graph adds, over every ordered pair of vertices, the two qualities times the probability of connection:

$$
PC_{\text{num}}(G) = \sum_{s \in V} \sum_{t \in V} w_s \cdot q_t \cdot \Pi(s,t)\,.
$$

The pairs $s = t$ count, with $\Pi(s,s) = 1$: a patch is connected to itself.

!!! note "Relation with the Probability of Connectivity"

    When both qualities of a vertex are its habitat area, $q_u = w_u$, the score is the numerator of the *Probability of Connectivity* indicator of Saura and Pascual-Hortal (2007), $PC = PC_{\text{num}} / \mathcal{A}^2$, where $\mathcal{A}$ is the area of the study region, and its square root is the *Equivalent Connected Area*. Dividing by a constant does not change which options are the best, so GECOT works with $PC_{\text{num}}$ and reports it as `pc_num`.

    The usual way to set the probabilities is $\pi_{uv} = \exp(-\alpha \cdot l_{uv})$, where $l_{uv}$ is the distance, or the cost of the least-cost path, between $u$ and $v$, and $\alpha = \ln 2 / \theta$ for a species of median dispersal distance $\theta$.

## Options

The decisions are taken among a set $\Phi$ of **options**. An option $i \in \Phi$ stands for one conservation or restoration action. It has a **cost** $c_i \ge 0$ and it improves some elements of the graph:

- a **source quality gain** $w^i_u \ge 0$ and a **target quality gain** $q^i_u \ge 0$ for each vertex $u$ (both $0$ where the option does nothing);
- an **improved probability** $\pi^i_a$ for each arc $a$ (equal to $\pi_a$ where the option does nothing).

A *scenario* is a vector $x \in \{0,1\}^\Phi$, with $x_i = 1$ when the option $i$ is bought. It turns $G$ into the graph $G^x$, with the same vertices and arcs and

$$
\begin{aligned}
w^x_u &= w_u + \sum_{i \in \Phi} w^i_u \cdot x_i\,, \\
q^x_u &= q_u + \sum_{i \in \Phi} q^i_u \cdot x_i\,, \\
\pi^x_a &= \max\Big\{ \pi_a\,,\; \max_{i \in \Phi}\, \pi^i_a \cdot x_i \Big\}\,.
\end{aligned}
$$

The gains of several options on the same vertex add up, whereas their improvements of the same arc compete: only the best probability counts. To make options exclusive instead, put them in a [conflict set](#several-costs-and-conflict-sets); see the [modeling page](../usage/modeling.md). $\Pi^x(s,t)$ denotes the probability of a most reliable path in $G^x$. The scenario that buys every option gives the largest values, written

$$
\begin{aligned}
\overline{w}_u &= w_u + \sum_{i \in \Phi} w^i_u\,, \\
\overline{q}_u &= q_u + \sum_{i \in \Phi} q^i_u\,, \\
\overline{\pi}_a &= \max\Big\{ \pi_a\,,\; \max_{i \in \Phi}\, \pi^i_a \Big\}\,.
\end{aligned}
$$

A set of options $S \subseteq \Phi$ is identified with the scenario that buys exactly its options, as in $G^S$ or $\Pi^S(s,t)$.

!!! tip "Two options that add up on one arc"

    To let two options $i$ and $j$ cumulate on an arc $(u,v)$, replace it with two arcs $(u,m)$ and $(m,v)$ through a new vertex $m$ of quality $0$, with $\pi_{um} \cdot \pi_{mv} = \pi_{uv}$, where $i$ improves $(u,m)$ and $j$ improves $(m,v)$.

The same statement covers restoration and conservation:

- **Restoration.** $G$ is the landscape as it is, and buying $i$ raises qualities and probabilities.
- **Conservation.** $G$ is the landscape as it would be if nothing were protected, every threatened element degraded. Buying $i$ keeps the quality $w^i_u$ of each vertex from being lost and keeps the probability of each arc from dropping below $\pi^i_a$.

## Cases and criterion

GECOT optimizes several landscape graphs at once. An instance has a set $K$ of **cases**, typically one per species. Each case $k \in K$ has its own graph $G_k = (V_k, A_k)$ with its qualities and probabilities, and the options are shared: one option can improve vertices and arcs of several cases, as a wildlife crossing serves several species.

The **criterion** $\mathcal{C}$ combines the scores of the cases into the single value that is maximized. It is a formula built from

- the score of a case,
- constants,
- sums, products and minimums of such formulas.

When an instance gives no criterion, $\mathcal{C}$ is the sum of the scores of all the cases. A weighted sum balances species, and a minimum asks for a solution that is good for the worst-off case:

$$
\begin{aligned}
\mathcal{C} &= PC_{\text{num}}(G_{\text{squirrel}}) + \tfrac{1}{2} \cdot PC_{\text{num}}(G_{\text{hedgehog}})\,, \\
\mathcal{C} &= \min\big\{ PC_{\text{num}}(G_{\text{squirrel}})\,,\; 3 \cdot PC_{\text{num}}(G_{\text{hedgehog}}) \big\}\,.
\end{aligned}
$$

The [instance file page](../usage/instance.md#criterion) gives the syntax, and which formulas each family of algorithms accepts.

## The optimization problem

Given the cases, the options, the criterion and a **budget** $\beta \ge 0$, GECOT looks for a scenario within the budget that maximizes the criterion:

$$
\begin{aligned}
\max_{x \in \{0,1\}^\Phi} \quad & \mathcal{C}\Big( \big( PC_{\text{num}}(G^x_k) \big)_{k \in K} \Big) \\
\text{s.t.} \quad & \sum_{i \in \Phi} c_i \cdot x_i \le \beta\,.
\end{aligned}
$$

The value of the criterion in a scenario is called its **score**. The *initial score* is the score of the scenario that buys nothing, and the *gain* of a solution is its score minus the initial score.

With a single case and $q = w$ this is the *Budget-Constrained PC Optimization* problem (BC-PC-Opt) of the thesis, called *Max-PC* in the article. It is NP-hard, and unless $P = NP$ no polynomial algorithm guarantees more than a fraction $1 - \frac{1}{e} \simeq 0.632$ of the optimum, already when each option improves a single arc. This is why GECOT offers both [exact algorithms](flow.md), which prove how good their solution is, and [heuristics](heuristics.md), which do not.

### Several costs and conflict sets

Two generalizations add constraints of the same kind, rows on $x$ with non-negative coefficients; not every algorithm [supports them](../usage/choosing.md#conflict-sets-and-several-costs) yet.

- **Several costs.** Each option has a cost $c_{i,r} \ge 0$ in each dimension $r \in R$, money and area for instance, and each dimension a budget $\beta_r$: the single budget row becomes $\sum_{i \in \Phi} c_{i,r} \cdot x_i \le \beta_r$ for every $r \in R$.
- **Conflict sets.** Sets $\Omega_l \subseteq \Phi$ of alternative options, of which at most one is bought: $\sum_{i \in \Omega_l} x_i \le 1$ for every set $l$. The sets may overlap.

Buying nothing stays feasible, and buying more never lowers a score, so every bound computed while ignoring these constraints stays an upper bound, only looser. The instance file page gives the syntax of [several costs](../usage/instance.md#several-costs) and of [conflict sets](../usage/instance.md#conflict-sets), and the [modeling page](../usage/modeling.md) what they can express.

## Why the options interact

The score is not a sum of independent contributions of the options, which is what makes the problem hard and ranking methods unreliable:

- **Synergy.** Two options that each improve one half of a corridor are worth little alone and much together.
- **Redundancy.** Two options that open two parallel corridors between the same patches are each worth much alone, and together hardly more than one of them.

The [first run](../getting-started/first-run.md) shows both effects on three patches, with numbers that can be checked by hand.

## References

- S. Saura and L. Pascual-Hortal (2007). A new habitat availability index to integrate connectivity in landscape conservation planning: comparison with existing indices and application to a case study. *Landscape and Urban Planning*, 83, 91–103.
- F. Hamonic, C. H. Albert, B. Couëtoux and Y. Vaxès (2022). Optimizing the ecological connectivity of landscapes. *Networks*. [doi:10.1002/net.22131](https://doi.org/10.1002/net.22131)
- F. Hamonic, B. Couëtoux, Y. Vaxès and C. H. Albert (2023). Cumulative effects on habitat networks: how greedy should we be? *Biological Conservation*, 282, 110066. [doi:10.1016/j.biocon.2023.110066](https://doi.org/10.1016/j.biocon.2023.110066)
- F. Hamonic (2023). PhD thesis, Aix-Marseille Université. [theses.fr](https://theses.fr/api/v1/document/2023AIXM9063)
- F. Hamonic, Y. Vaxès, B. Couëtoux and C. H. Albert (2025). GECOT: Graph-based ecological connectivity optimization tool. *Methods in Ecology and Evolution*. [doi:10.1111/2041-210X.70055](https://doi.org/10.1111/2041-210X.70055)
