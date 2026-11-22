# Benders decompositions

Most of the [flow formulation](flow.md) is flows: once the options $x$ are fixed, everything else is a set of independent linear programs, one per target, whose optimum a shortest path algorithm gives without any solver. A Benders decomposition exploits this. It keeps only the options in a small *master* program, replaces each flow by the linear inequalities, the *cuts*, that its dual solutions provide, and adds these cuts only when the search needs them.

GECOT has three such decompositions, which differ in what the cuts bound: `flow_benders`, `multicut_benders` and `multiflow_benders`. All three are exact, like `flow_mip`, and use the same [reduced graphs](preprocessing.md#the-graph-of-a-target) $G_t$.

## The cut of a target

Fix a target $t$, its reduced graph $G_t$ with the improved copies $A'$, and a scenario $x$. With $w^x_u = w_u + \sum_{i \in \Phi} w^i_u \cdot x_i$, the flow that reaches $t$ is the optimum of

$$
\begin{aligned}
f_t(x) = \max \;\; & f \\
\text{s.t.} \;\;
& \sum_{a \in \delta^{\text{out}}_u} \phi_a - \sum_{b \in \delta^{\text{in}}_u} \pi_b \cdot \phi_b \le w^x_u && u \neq t && (\rho_u) \\
& f + \sum_{a \in \delta^{\text{out}}_t} \phi_a - \sum_{b \in \delta^{\text{in}}_t} \pi_b \cdot \phi_b \le w^x_t && && (\rho_t) \\
& \phi_{a^i} \le M^t_u \cdot x_i && a^i \in A' && (\sigma_{a^i}) \\
& \phi \ge 0\,, \; f \ge 0\,,
\end{aligned}
$$

where $u$ is the vertex that the improved copy $a^i$ leaves, and the dual variable of each constraint is written on its right. The dual program is

$$
\begin{aligned}
\min \;\; & \sum_{u} w^x_u \cdot \rho_u + \sum_{a^i \in A'} M^t_u \cdot x_i \cdot \sigma_{a^i} \\
\text{s.t.} \;\;
& \rho_t \ge 1\,, \\
& \rho_u \ge \pi_a \cdot \rho_v && a = (u,v) \in A\,, \\
& \rho_u + \sigma_{a^i} \ge \pi^i_a \cdot \rho_v && a^i = (u,v) \in A'\,, \\
& \rho \ge 0\,, \; \sigma \ge 0\,.
\end{aligned}
$$

The scenario appears only in its objective. A dual solution $(\rho, \sigma)$ is therefore feasible whatever the scenario, and by weak duality it bounds the flow of *every* scenario for which the constants $M^t_u$ are valid, that is, with the [default constants](preprocessing.md#big-m-constants), every scenario within the budget:

$$
f_t(x) \;\le\; \sum_{u} \Big( w_u + \sum_{i \in \Phi} w^i_u \cdot x_i \Big) \cdot \rho_u \;+\; \sum_{a^i \in A'} M^t_u \cdot \sigma_{a^i} \cdot x_i\,,
$$

an inequality that is linear in $x$.

The dual variable $\rho_u$ is a *potential*: what a unit of quality at $u$ is worth at $t$. Given a reference scenario $y$, take the probabilities of connection to $t$ in $G_t$ under $y$, and let each closed copy pay for what it would bring:

$$
\rho_u = \Pi^y(u,t)\,, \qquad \sigma_{a^i} = \big( \pi^i_a \cdot \rho_v - \rho_u \big)^+ \quad \text{for } a^i = (u,v)\,,
$$

where $(\cdot)^+ = \max\{\cdot, 0\}$. This solution is feasible, since a most reliable path from $u$ is at least as reliable as the one that starts with any open arc, and $\sigma_{a^i} = 0$ for the options that $y$ buys. It gives the **cut of $t$ at $y$**:

$$
\begin{aligned}
f_t(x) \;\le\; \operatorname{cut}_{t,y}(x) = {} & \sum_{u} \Big( w_u + \sum_{i \in \Phi} w^i_u \cdot x_i \Big) \cdot \rho_u \\
& + \sum_{i \,:\, y_i = 0} \;\; \sum_{a^i = (u,v) \in A'} M^t_u \cdot \big( \pi^i_a \cdot \rho_v - \rho_u \big)^+ \cdot x_i\,.
\end{aligned}
$$

At $x = y$ the second sum vanishes and the first is $\sum_u w^y_u \cdot \Pi^y(u,t) = f_t(y)$: the cut is tight where it was built. Its coefficients read as an estimate of what each option would add to the flow of $y$: its quality gains at their current worth, and its arcs for the flow $M^t_u$ that they could carry at most. Computing it takes one run of Dijkstra's algorithm from $t$ on the reversed graph: no linear program is solved.

## `flow_benders`: cuts on the flows

The master of `flow_benders` keeps the variables $x$, the score $z_k$ of each case, and for each target the flow $f_t$ and the variables $f^i_t$ that linearize its target quality gains:

$$
\begin{aligned}
\max \;\; & \mathcal{C}(z) \\
\text{s.t.} \;\;
& z_k \le \sum_{t \in V_k} \Big( q_t \cdot f_t + \sum_{i \in \Phi} q^i_t \cdot f^i_t \Big) && k \in K\,, \\
& f^i_t \le f_t\,, \quad f^i_t \le M^t_t \cdot x_i && t\,,\; i \in \Phi\,, \\
& f_t \le \operatorname{cut}_{t,y}(x) && t\,,\; y \in Y_t\,, \\
& \sum_{i \in \Phi} c_i \cdot x_i \le \beta\,, \quad x \in \{0,1\}^\Phi\,,
\end{aligned}
$$

where $Y_t$ is the set of scenarios at which a cut of $t$ has been added so far. With all the scenarios in $Y_t$ the master would be equivalent to the flow formulation; the search adds only the few that matter.

`--indicator-constraints` writes the big-M constraint of each target quality gain as an *indicator constraint*, $x_i = 0 \Rightarrow f^i_t \le 0$, as [in `flow_mip`](flow.md#what-flow_mip-solves). The cuts keep their constants $M^t_u$, which are coefficients and not constraints. It is an experimental switch, for the solvers that have such constraints (`gurobi`, `cplex`).

With [several costs or conflict sets](problem.md#several-costs-and-conflict-sets), the budget row of this master, and of the masters of `multicut_benders` and `multiflow_benders` below, becomes the rows of the [flow formulation](flow.md#cases-and-criterion): one per cost dimension and one per conflict set. The cuts are the same.

## `multicut_benders`: cuts on groups of targets

`multicut_benders` also removes the variables of the targets. The value of a target in a scenario is

$$
v_t(x) = q^x_t \cdot f_t(x) = q^x_t \cdot \sum_{s} w^x_s \cdot \Pi^x(s,t)\,.
$$

In the flow formulation it is $q_t \cdot f_t + \sum_i q^i_t \cdot f^i_t$ with $f^i_t \le f_t$ and $f^i_t \le M^t_t \cdot x_i$. Using the first bound for the options that $y$ buys and the second for the others gives $v_t(x) \le q^y_t \cdot f_t(x) + \sum_{i : y_i = 0} q^i_t \cdot M^t_t \cdot x_i$, hence, with the potentials $\rho_u = q^y_t \cdot \Pi^y(u,t)$ that already count the target quality:

$$
\begin{aligned}
v_t(x) \;\le\; \operatorname{cut}^v_{t,y}(x) = {} & \sum_{u} \Big( w_u + \sum_{i \in \Phi} w^i_u \cdot x_i \Big) \cdot \rho_u \\
& + \sum_{i \,:\, y_i = 0} \;\; \sum_{a^i = (u,v) \in A'} M^t_u \cdot \big( \pi^i_a \cdot \rho_v - \rho_u \big)^+ \cdot x_i \\
& + \sum_{i \,:\, y_i = 0} M^t_t \cdot q^i_t \cdot x_i\,,
\end{aligned}
$$

again with equality at $x = y$. The targets of each case are split into **groups**, and the master has one variable $\theta_g$ per group $g$:

$$
\begin{aligned}
\max \;\; & \mathcal{C}(z) \\
\text{s.t.} \;\;
& z_k \le \sum_{g \in \mathcal{G}_k} \theta_g && k \in K\,, \\
& \theta_g \le \sum_{t \in g} \operatorname{cut}^v_{t,y}(x) && g\,,\; y \in Y_g\,, \\
& \sum_{i \in \Phi} c_i \cdot x_i \le \beta\,, \quad x \in \{0,1\}^\Phi\,.
\end{aligned}
$$

`--target-groups` sets the number of groups per case:

- `--target-groups 1` is the classical single-cut decomposition: one cut bounds the whole score of a case;
- `--target-groups 0` gives each target its own group, the multi-cut decomposition;
- in between (2 by default), fewer groups mean a smaller master with fewer but weaker cuts.

`--target-grouping` decides which targets go together. With `voronoi`, the default, a group is a cell of a network Voronoi diagram: its kernels are targets of high value that are far from each other in the landscape without options, and each target joins the kernel that reaches it with the most reliable path, so that a group holds targets that the same options serve. `consecutive` cuts the list of targets into consecutive runs, and `random` draws the groups at random (`--target-grouping-seed`), as a control.

## `multiflow_benders`: cuts per source

The cut of a target has one weakness. Its big-M constant $M^t_u$ is the flow that *all* the sources together can bring to $u$, and an option is charged for all of it even when it only helps a few sources. `multiflow_benders` decomposes the flow of a target further, into one flow per source, as in a multicommodity flow:

$$
f_t(x) = w^x_t + \sum_{s \neq t} w^x_s \cdot \Pi^x(s,t)\,.
$$

The part of a source $s \neq t$ is the optimum of the program of [the cut of a target](#the-cut-of-a-target) with the quality of $s$ alone, restricted to the set $A_s$ of the arcs that a most reliable path from $s$ to $t$ can use in some scenario, and with its own constants $m_{sa}$:

$$
\begin{aligned}
\max \;\; & f \\
\text{s.t.} \;\;
& \sum_{a \in \delta^{\text{out}}_u \cap A_s} \phi_a - \sum_{b \in \delta^{\text{in}}_u \cap A_s} \pi_b \cdot \phi_b \le [u = s] \cdot w^x_s && u \neq t && (\rho^s_u) \\
& f + \sum_{a \in \delta^{\text{out}}_t \cap A_s} \phi_a - \sum_{b \in \delta^{\text{in}}_t \cap A_s} \pi_b \cdot \phi_b \le 0 && && (\rho^s_t) \\
& \phi_{a^i} \le m_{sa} \cdot x_i && a^i \in A_s \cap A' && (\sigma^s_{a^i}) \\
& \phi \ge 0\,, \; f \ge 0\,,
\end{aligned}
$$

where $[u = s]$ is $1$ at $s$ and $0$ elsewhere, and

$$
m_{sa} = \overline{w}_s \cdot \overline{\Pi}(s,u) \qquad \text{for an improved copy } a \text{ leaving } u\,,
$$

what $s$ alone can bring to $u$ with every option. An arc or an improved copy $a = (u,v)$ is in $A_s$ when the best path of $s$ through it with every option, of probability $\overline{\Pi}(s,u) \cdot \pi_a \cdot \overline{\Pi}(v,t)$ with the probability of $a$ itself, is at least as reliable as the best path of $s$ without any option, up to a relative $10^{-9}$: in every scenario a most reliable path of $s$ is at least that reliable, so it uses no other arc. Only the sources that can bring something to $t$ get a flow, those with $\overline{w}_s > 0$ and $\overline{\Pi}(s,t) > 0$. Any potentials $\rho^s$ with $\rho^s_t = 1$ and $\rho^s_u \ge \pi_a \cdot \rho^s_v$ on the arcs $a = (u,v)$ of $A_s \cap A$ then give

$$
\begin{aligned}
f_t(x) \;\le\; w^x_t + \sum_{s \neq t} \Big[ \; & \rho^s_s \cdot w^x_s \\
& + \sum_{a^i = (u,v) \in A_s \cap A'} m_{sa} \cdot \big( \pi^i_a \cdot \rho^s_v - \rho^s_u \big)^+ \cdot x_i \; \Big]\,.
\end{aligned}
$$

An option now pays, for each source, only what this source can bring to its arcs. [`multiflow_benders` step by step](multiflow_benders.md) follows a run of it in order, on an example. The master is the one of `flow_benders` with these cuts in place of $\operatorname{cut}_{t,y}$, and with each $f_t$ bounded by $M^t_t$: without this bound its relaxation is unbounded until $t$ receives a cut. The constant $M^t_t$ is the one of `--big-m`, `knapsack_bnb` by default; `tight` and `exact` lower that of the $f^i_t$ only, as the constant of a scenario that buys $i$ does not bound $f_t$. Only the gains $q^i_t > 0$ get a variable $f^i_t$. Its linear relaxation is much closer to the optimum.

### Separation

The cut of a point $\hat{x}$, which may be fractional, takes for each source the potentials of its largest flow at $\hat{x}$, where an improved copy carries at most $m_{sa} \cdot \hat{x}_i$. The program of a source is a generalized maximum flow problem, a flow with gains (Jewell, 1962), in a lossy network: no arc has a gain above $1$. Its largest flow is computed by Onaga's algorithm (1966), which augments along the paths of highest gain, with Dijkstra's algorithm on relabeled gains, as Tardos and Wayne (1998) describe it. In the residual graph an arc that carries flow can be taken backward, with a gain $1/\pi$ above $1$, so Dijkstra's algorithm runs on gains reduced by potentials $p$, with $p_t = 1$ and $p_u \ge g \cdot p_v$ on every residual arc of gain $g$ from $u$ to $v$: the reduced gain $g \cdot p_v / p_u$ is at most $1$.

- One search from $t$ over every arc that is open at $\hat{x}$, made once for all the sources of the target, gives the first potentials of each source, whose arcs are among these.
- Each augmenting path is found by a search from the source, stopped once it settles $t$. With $D_u$ the reduced gain of the best path from the source to $u$, the potentials of the vertices it settled become $p_u \cdot D_t / D_u$. They stay feasible and are tight on the path, so the arcs that the augmentation opens backward have a reduced gain of $1$. The potentials lead the search toward $t$: it settles the vertices that a path as good as the best one could cross, not all of $A_s$.
- When the quality of the source is sent, or when the source no longer reaches $t$, one search from $t$ on the last residual graph gives $\rho^s$, the gains of the best residual paths to $t$: the inverses of what Tardos and Wayne call the canonical labels. They are an optimal dual solution, and the least one: every optimal dual solution is feasible on the residual graph of a largest flow, so it is at least these gains at every vertex.

At a scenario, one path takes the whole quality of the source, and $\rho^s$ holds the probabilities of connection to $t$ within $A_s$. There is one exception: when the source has its whole quality $\overline{w}_s$ and the path is its best one to an improved copy, the copy carries exactly $m_{sa}$ and leaves the residual graph, and the cut then charges its option for what it brings. Onaga's algorithm can need exponentially many augmentations. If this computation needs more than $100\,000$ augmentations, it stops there and takes $\rho^s$ on the residual graph it reached. These potentials are feasible too, so the cut stays valid. The flow of a candidate never depends on this limit: it comes from one search from $t$, step 2 of [the search](#the-search).

The sets $A_s$ and the constants $m_{sa}$ come from one run of Dijkstra's algorithm from each source, which does not go on from a vertex $u$ with $\overline{\Pi}(s,u) \cdot \overline{\Pi}(u,t)$ under the threshold of the test: no arc of $A_s$ leaves such a vertex or one reached through it. They are computed once and kept. Their sizes add up to at most the arcs of $G_t$ times its sources, for every target, which exceeds the memory on large graphs that the reduction barely shrinks. `--no-stored-arcs` then keeps none and rebuilds those of each source at each separation: the same cuts, at the price of one more search per source and separation.

### Cuts on the relaxation

Before the search, `multiflow_benders` solves the linear relaxation of its master, with $x \in [0,1]^\Phi$, and separates cuts until the optimum of the relaxation violates none. It does so *in-out*: the point that is separated is a center, which starts at the first optimum of the relaxation and moves toward each new one by `--in-out-ratio` of the way (0.9 of it, by default), and whose cuts are deeper than those of the optimum itself; when the center yields no violated cut, the optimum takes over. `--in-out-separation` separates instead the point that share of the way from the old center to the new optimum, before the center moves; by default it is the new center itself. On 18 runs of the benchmark, a step of 0.9, or a separation at 0.9 with a step of 0.5, shortened the rounds by 13 to 15 %, but not the runs of `multiflow_benders`, whose time is in the search. `--in-out-lift δ` raises each option of the point separated to at least $\delta$, the "2ε trick" of Fischetti, Ljubić and Sinnl (2017): an option at $0$ then opens its improved copies, with a capacity of $m_{sa} \cdot \delta$, in the largest flow of each source, which changes the potentials of the cut. The cut stays valid, its potentials being feasible at every point, but the lifted point can be over the budget: the lifted rounds give no `fractional gain`, and `--root-cut-gap` can end only the last ones. Once a lifted round adds no cut, the lift ends with the center, and the optimum itself is separated. The lift is $10^{-7}$ by default. On 31 runs of the benchmark at 60 s, the short suite without its dks, cover and faircover instances, with Aix 70 × 70 at 8 and 12 and Quebec at 1000 and 10000, this lift with a step of 0.9 took about 11 % less time than the step of 0.9 alone, on two seeds of the solver, and 14 % less than a step of 0.5 without the lift, the defaults until then, in geometric mean over the 28 runs that every setting solved. With a step of 0.5, the lift took as long as without it. Against the step of 0.9 alone, it took 39 to 51 % less time on the five bike runs that every setting solved and 15 to 23 % less on the species, and up to 10 % more on the landscapes, Aix and Quebec, where each separation was up to about 3.5 times slower: urban-greening-large at 1600, the worst run, took 5.8 s instead of 3.2 s. It left the dks, cover and faircover instances as they were, and $10^{-5}$, about the value of Fischetti, Ljubić and Sinnl, did as well as $10^{-7}$. On the bike rounds that the time share stops, at budgets of 5000 and 10000, the lift took back most of what the step of 0.9 loses there: at 45 s, their bounds were 0.0006 and 0.0012 % above the best one, against 0.0034 and 0.0029 % at 0.9 alone and 0.0003 % at 0.5. A cut is added when the value of $f_t$ at the optimum exceeds the cut by more than `--root-cut-tolerance` of it ($10^{-7}$, the default, at least) and by $10^{-7}$ absolute; neither follows `-t`. The rounds end when no cut is added: a larger tolerance ends them sooner, with a relaxation above its value with all the cuts by about that share at most, unless the criterion subtracts a constant. On large instances the last rounds often move the bound only in its fourth decimal: on the bike instances of the benchmark, `--root-cut-tolerance 1e-3` ended the rounds in about half the time, for a bound higher by 0.02 to 0.09 points of gain. The [table of the rounds](../usage/results.md#progress) shows how far they still are from their end: each row gives the bound, `max gain`; the `fractional gain`, the best value so far of a point separated with its flows, which the relaxation with all the cuts reaches at least; and the `violation`, the smallest tolerance with which the round adds no cut. `--root-cut-gap g` also ends the rounds once the bound is within $g$ of the fractional gain, relative to its score, as `--relative-gap` does for `path_lp`: on the same bike instances, `--root-cut-gap 1e-3` ended the rounds 55 to 69 % sooner, for a bound higher by about 0.04 points of gain. Both were measured at a step of 0.5 without the lift. The lifted rounds give no fractional gain: on four bike runs with the lift, only the last round had one, and `--root-cut-gap 1e-3` changed nothing, where with `--in-out-lift 0` it ended the rounds 42 to 68 % sooner, for a bound 0.0005 to 0.033 % higher. With a time limit, this phase takes at most half of it. A Ctrl-C ends it at the end of the current round, and the search goes on from there; the next Ctrl-C stops the search. The value of the relaxation with its cuts is an upper bound on the optimum, reported as `root_relaxation_value`, and the bound of the run is the lower of this value and the solver's bound. `--no-root-cuts` skips the phase, for comparison, and `--in-out-ratio 1` separates the optimum from the start.

### Slack cuts removed

The rounds add about one cut per target and round, and the relaxation keeps them all, although an optimal basis needs at most as many of its rows as the relaxation has columns: on a hexagon grid of 1,456 targets and 1,386 options, at 5 % of their cost and with `--no-stored-arcs`, 256,000 rows after 181 rounds. Warm started from the last basis, the dual simplex makes about as many pivots each round, but each pivot costs in proportion to the rows. `--root-cut-purge k` removes the row of a cut once it is slack at $k$ optima in a row, by `--root-cut-tolerance` of its terms there and by $10^{-7}$; a part of a [group](#groups-of-sources) then leaves the envelope of its group too. The slack of such a row is basic: the basis of the optimum stays one, and the value of the relaxation does not change. A removed cut can be needed again at a later optimum, where the separation adds it back. So that the rounds cannot cycle through the same rows, rows leave only at a value below that of the last removal, by the same tolerances: that happens finitely often, and from the last removal on the rows only grow, as without it. Rounds that converge thus end at the same value within `--root-cut-tolerance`, since the separation of their last optimum finds no cut, removed ones included; rounds cut short, by the time share, `--root-cut-gap` or Ctrl-C, can end higher. Only Gurobi, CPLEX and HiGHS remove rows through mippp: with the other solvers the rows stay.

`multiflow_rounding`, which reads only the optimum and the value of the relaxation, removes them with $k = 10$ by default. On the 42 runs of the short suite of the benchmark, at 60 s, where the rounds are short and the separation most of them, the runs took 64.3 s at $k = 10$, against 66.3 s without removal, 65.8 s at 5, 70.6 s at 3 and 64.2 to 64.9 s from 15 to 50, for the same values of the relaxation to $10^{-6}$: a small $k$ adds back more cuts, on cover-80 at 24 8,873 rounds at 3 against 814 without removal. Their optima differ, and so do the scenarios drawn from them: at $k = 10$ five runs ended with a better solution and none with a worse one. On the hexagon grid above, the rounds alone, $k = 10$ reached round 80 in 2,909 s against 3,592 s without removal, for a bound within 0.04 points of gain of it at that round, with 15,756 rows of cuts left at round 89 out of 129,052 added; $k = 3$, on an instrumented build, reached round 69 in 1,847 s against 2,481 s, and $k = 5$, 20 and 40 reached round 61 in 1,445 s, 1,668 s and 1,784 s, against 1,530 s at 10 and 1,833 s without removal. A small $k$ makes a round faster but brings back more cuts, which a round then moves the bound less: at equal times, $k$ from 3 to 40 kept the bounds within 0.02 points of gain of each other over the first 1,800 s, while at 3,600 s $k = 10$ was at 102.60 % against 102.77 % without removal. Most of what is left of a round is the separation, which grows too, from 10 s at the first round to 55 s at the 69th. `multiflow_benders` removes none by default, its search then having only the rows left: at $k = 3$, the 42 runs took 651 s against 666 s but solved 33 against 35, faster on most landscape and species runs (species-periurban at 6000, 22.9 s against 34.3 s), slower on bike-squirrel-suburb (at 200, 12.8 s against 5.5 s; at 500, unsolved in 60 s against 54.4 s) and on faircover-40 at 16, unsolved against 58.1 s.

### Groups of sources

`--source-group-size n`, experimental and off by default (`0`), gives the sources of a target variables of their own in the master. The $n_t$ sources of $t$ whose $A_s$ holds an improved copy go in $\lceil n_t / n \rceil$ groups, each with a variable $f_{tg}$ within $\sum_{s \in g} \overline{w}_s \cdot \overline{\Pi}(s,t)$, and

$$
f_t \le w^x_t + \sum_{s \in F_t} w^x_s \cdot \Pi(s,t) + \sum_g f_{tg}\,,
$$

where $F_t$ holds the other sources, whose flow is this affine function of $x$ at every point. Of a cut, a group receives the sum of the parts of its sources, $f_{tg} \le \sum_{s \in g} \big[ \rho^s_s \cdot w^x_s + \dots \big]$, when the point exceeds it or when it lies below the group's bound and earlier parts there; the cut on $f_t$ is added too when these rows leave the point above it. A target left with a single group keeps the cuts on $f_t$, as tight. Each group keeps the least of its own parts, where $f_t$ keeps the least of their sums, so the groups combine the parts of different cuts, which a cut on $f_t$ cannot, at the price of a variable per group and of more rows; at each point separated, the master is as tight as with the cut on $f_t$. Once every point is separated, both give the same relaxation. With `--root-cut-purge`, a part whose row the rounds [remove](#slack-cuts-removed) leaves the envelope of its group too, which then only holds the rows still in the master. With `--no-stored-arcs` the sets $A_s$ are not kept, and every source is in a group.

`--source-grouping` decides which sources go together. With `voronoi`, the default, a group is a cell of a network Voronoi diagram of $G_t$ without $t$, its arcs taken both ways at their best probability, the ways compared by probability then by their number of arcs: its kernels are chosen as those of `multicut_benders`, by the bounds $\overline{w}_s \cdot \overline{\Pi}(s,t)$ of the sources, and a source that reaches no kernel joins the smallest cell. `consecutive` cuts the list of the sources into consecutive runs, and `random` draws the groups at random (`--source-grouping-seed`), as a control. On the benchmark, a variable per source makes the master of the coverage instances the coverage MIP, solved in a second where the default does not end in 60 s, but makes the landscape and bike runs 8 to 14 times slower, or unsolved in 60 s; groups of 16 by `voronoi` are 1.0 to 2.6 times slower than the default there.

## The search

The three masters are solved by the branch and bound of a MILP solver, with the cuts added *lazily*:

1. Before the search, `flow_benders` and `multicut_benders` add the cuts of the scenario that buys nothing, all three add the cuts of the MIP start if there is one, and `multiflow_benders` adds its cuts on the relaxation.
2. Each time the solver finds a candidate scenario $y$, GECOT computes the true flows $f_t(y)$ by one shortest path computation per target.
3. For each target, or each group, that the master overestimates by more than the tolerance `-t` (relative to its true value), the cut at $y$ is added as a lazy constraint. `multiflow_benders` builds it by its [separation](#separation) at $y$, for these targets only.
4. A candidate that receives a cut is rejected by the solver, although $y$ is a perfectly valid solution whose score is now known. GECOT hands it back with its true values, so that no solution found is lost.

The search ends when the solver has proved its best solution optimal for the master, whose cuts then describe the score exactly where it matters. A candidate is accepted when the master overestimates none of its targets by more than `-t`, so the result is exact up to this tolerance, to the solver's own, and to the [resolution](preprocessing.md#resolution) of the preprocessing.

### Without lazy constraints

Only Gurobi, CPLEX and COPT take lazy constraints through mippp. With the other solvers, GECOT solves the master again instead. It solves the master, computes the cuts at its solution $y$ as in steps 2 and 3, adds them to the master as constraints, and solves it again, until a solution receives no cut. The true score of each solution is then known: GECOT keeps the best one, starting from the MIP start, and gives it to the next solve as its start. The bound is the lowest of the bounds of the masters. The search also ends when the bound is within the solver's gap of the best score. A scenario that the master proposes a second time receives the cuts of all its targets or groups, not only of those that the master overestimates: the first time, a target that the master did not overestimate received none, and a later master can overestimate it. Proposed a third time, the scenario ends the search, since the master then overestimates it only by the solver's tolerances.

Each solve starts a new branch and bound, so this search is slower than one with lazy constraints. `--no-lazy-constraints` makes Gurobi, CPLEX and COPT search this way too, for comparison. `flow_benders` and `multiflow_benders` cannot use SCIP, whose model in mippp cannot add columns.

## `multiflow_rounding`: the relaxation, rounded

`multiflow_rounding` solves the linear relaxation of `multiflow_benders` with its [cuts on the relaxation](#cuts-on-the-relaxation), the same rounds with the same parameters but no lift by default, `--in-out-lift 0`, and with its [slack cuts removed](#slack-cuts-removed), `--root-cut-purge 10`, and stops there: no search. The rounds are most of its run: on 18 runs of the benchmark, the lift made them 31 % longer for solutions as good, and a step of 0.9 shortened them by about 13 % against 0.5, for solutions as good on average. It then rounds the optimum $x^*$ of the last relaxation into scenarios, a randomized rounding (Raghavan and Thompson, 1987):

1. The first scenario buys the options by decreasing $x^*_i$, each one that still fits in the budgets and its conflict sets, as `path_lp` rounds its relaxation.
2. Each other one draws each option $i$ with probability $x^*_i$, tries the options drawn in a random order, each where it fits, then the other options by decreasing $x^*_i$. Buying more never lowers the score, so a scenario ends with no option left that fits.
3. Each scenario is scored exactly, by the flows of the blocks, as a [candidate of the search](#the-search) is, and the same scenario only once. A MIP start is scored with them.

It returns the best scenario, with the value of the relaxation as its upper bound. The draws stop after `--roundings` of them, 1000 by default, at `--time-limit`, on Ctrl-C, or after a batch of 64 draws that brings no new scenario; the status is then `completed`, or `optimal` once the best scenario reaches the bound. `--roundings 0` draws until `--time-limit`, which it then requires. The draws follow `--rounding-seed` and are scored by batches in parallel: the same seed gives the same solution on any number of threads. With a time limit, the rounds on the relaxation take at most half of it, as for `multiflow_benders`, and the draws the rest; if no round ends in time, $x^* = 0$, and the scenarios buy the options in their order.

## Parameters

The three decompositions take any solver, but `flow_benders` and `multiflow_benders` not SCIP. With `--solver gurobi` (the default), `cplex` or `copt` the cuts are lazy constraints, with the others the master is [solved again](#without-lazy-constraints). They take the [common parameters](../reference/parameters.md#common-to-the-milp-based-algorithms) and the [preprocessing parameters](../reference/parameters.md#preprocessing), plus:

| Algorithm | Parameters |
|---|---|
| `multicut_benders` | `--target-groups`, `--target-grouping`, `--target-grouping-seed`, `--big-m` |
| `flow_benders` | `--big-m`, `--indicator-constraints` |
| `multiflow_benders` | `--big-m`, `--root-cut-tolerance`, `--root-cut-gap`, `--root-cut-purge`, `--no-root-cuts`, `--in-out-ratio`, `--in-out-separation`, `--in-out-lift`, `--no-stored-arcs`, `--source-group-size`, `--source-grouping`, `--source-grouping-seed` |

and `--no-lazy-constraints`, for comparison. Their report adds `lazy_constraints`, the number of cuts added during the search, `master_solves` when the master is solved again, and for `multiflow_benders` `sources`, `root_rounds`, `root_cuts` and `root_relaxation_value`, with `--root-cut-purge` `root_cuts_purged`, with `--source-group-size` `source_groups` and `affine_sources`, the variables of the groups and the sources written exactly.

`multiflow_rounding` takes any solver but SCIP, which it uses for linear programs only, the common and preprocessing parameters, and those of the rounds of `multiflow_benders`: `--big-m`, `--root-cut-tolerance`, `--root-cut-gap`, `--root-cut-purge`, `--in-out-ratio`, `--in-out-separation`, `--in-out-lift`, `--no-stored-arcs`, `--source-group-size`, `--source-grouping` and `--source-grouping-seed`. It adds `--roundings` and `--rounding-seed`. Its report has the fields of `multiflow_benders` but `lazy_constraints`, and `roundings` and `scenarios`, the draws made and the distinct scenarios scored.

## References

- P. Raghavan and C. D. Thompson (1987). Randomized rounding: a technique for provably good algorithms and algorithmic proofs. *Combinatorica*, 7(4), 365–374. [doi:10.1007/BF02579324](https://doi.org/10.1007/BF02579324)
- W. S. Jewell (1962). New methods in mathematical programming: optimal flow through networks with gains. *Operations Research*, 10(4), 476–499. [doi:10.1287/opre.10.4.476](https://doi.org/10.1287/opre.10.4.476)
- K. Onaga (1966). Dynamic programming of optimum flows in lossy communication nets. *IEEE Transactions on Circuit Theory*, 13(3), 282–287. [doi:10.1109/TCT.1966.1082612](https://doi.org/10.1109/TCT.1966.1082612)
- É. Tardos and K. D. Wayne (1998). Simple generalized maximum flow algorithms. *Integer Programming and Combinatorial Optimization (IPCO 1998)*, Lecture Notes in Computer Science 1412, 310–324. [doi:10.1007/3-540-69346-7_24](https://doi.org/10.1007/3-540-69346-7_24)
- M. Fischetti, I. Ljubić and M. Sinnl (2017). Redesigning Benders decomposition for large-scale facility location. *Management Science*, 63(7), 2146–2162. [doi:10.1287/mnsc.2016.2461](https://doi.org/10.1287/mnsc.2016.2461)
