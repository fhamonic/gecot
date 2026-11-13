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

The scenario appears only in its objective. A dual solution $(\rho, \sigma)$ is therefore feasible whatever the scenario, and by weak duality it bounds the flow of *every* scenario:

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

With [several costs or conflict sets](problem.md#several-costs-and-conflict-sets), the budget row of this master, and of the master of `multicut_benders` below, becomes the rows of the [flow formulation](flow.md#cases-and-criterion): one per cost dimension and one per conflict set. The cuts are the same.

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

`--groups` sets the number of groups per case:

- `--groups 1` is the classical single-cut decomposition: one cut bounds the whole score of a case;
- `--groups 0` gives each target its own group, the multi-cut decomposition;
- in between (2 by default), fewer groups mean a smaller master with fewer but weaker cuts.

`--grouping` decides which targets go together. With `voronoi`, the default, a group is a cell of a network Voronoi diagram: its kernels are targets of high value that are far from each other in the landscape without options, and each target joins the kernel that reaches it with the most reliable path, so that a group holds targets that the same options serve. `consecutive` cuts the list of targets into consecutive runs, and `random` draws the groups at random (`--grouping-seed`), as a control.

## `multiflow_benders`: cuts per source

The cut of a target has one weakness. Its big-M constant $M^t_u$ is the flow that *all* the sources together can bring to $u$, and an option is charged for all of it even when it only helps a few sources. `multiflow_benders` decomposes the flow of a target further, into one flow per source, as in a multicommodity flow:

$$
f_t(x) = w^x_t + \sum_{s \neq t} w^x_s \cdot \Pi^x(s,t)\,.
$$

The part of a source $s$ is the optimum of the program above with the quality of $s$ alone, restricted to the set $A_s$ of the arcs that a most reliable path from $s$ to $t$ can use in some scenario, and with its own constants:

$$
m_{sa} = \overline{w}_s \cdot \overline{\Pi}(s,u) \qquad \text{for an improved copy } a \text{ leaving } u\,,
$$

what $s$ alone can bring to $u$ with every option. An arc $(u,v)$ is in $A_s$ when the best path of $s$ through it with every option, of probability $\overline{\Pi}(s,u) \cdot \overline{\pi}_{uv} \cdot \overline{\Pi}(v,t)$, is at least as reliable as the best path of $s$ without any option. Any potentials $\rho^s$ with $\rho^s_t = 1$ and $\rho^s_u \ge \pi_a \cdot \rho^s_v$ on the arcs $a = (u,v)$ of $A_s \cap A$ then give

$$
\begin{aligned}
f_t(x) \;\le\; w^x_t + \sum_{s \neq t} \Big[ \; & \rho^s_s \cdot w^x_s \\
& + \sum_{a^i = (u,v) \in A_s \cap A'} m_{sa} \cdot \big( \pi^i_a \cdot \rho^s_v - \rho^s_u \big)^+ \cdot x_i \; \Big]\,.
\end{aligned}
$$

An option now pays, for each source, only what this source can bring to its arcs. The master is the one of `flow_benders` with these cuts in place of $\operatorname{cut}_{t,y}$, and its linear relaxation is much closer to the optimum. `multiflow_benders` does not accept [several costs or conflict sets](problem.md#several-costs-and-conflict-sets) yet.

**Separation.** The cut of a point $\bar{x}$, which may be fractional, takes for each source the potentials of its largest flow at $\bar{x}$, where an improved copy carries at most $m_{sa} \cdot \bar{x}_i$. This flow is computed by augmenting along the paths of highest gain, and the potentials of the last step are an optimal dual solution. At a scenario, one path takes the whole quality of the source and the potentials are the probabilities of connection.

**Cuts on the relaxation.** Before the search, `multiflow_benders` solves the linear relaxation of its master, with $x \in [0,1]^\Phi$, and separates cuts until the optimum of the relaxation violates none. It does so *in-out*: the point that is separated is a center that moves halfway toward each new optimum of the relaxation, whose cuts are deeper than those of the optimum itself, and when the center yields no violated cut the optimum takes over. With a time limit, this phase takes at most half of it. The value of the relaxation with its cuts is an upper bound on the optimum, reported as `root_relaxation_value`, and the bound of the run is the lower of this value and the solver's bound. `--no-root-cuts` skips the phase and `--no-in-out` separates the optimum from the start, both for comparison.

## The search

The three masters are solved by the branch and bound of a MILP solver, with the cuts added *lazily*:

1. Before the search, `flow_benders` and `multicut_benders` add the cuts of the scenario that buys nothing, all three add the cuts of the MIP start if there is one, and `multiflow_benders` adds its cuts on the relaxation.
2. Each time the solver finds a candidate scenario $y$, GECOT computes the true flows $f_t(y)$, by one shortest path computation per target.
3. For each target, or each group, that the master overestimates by more than the tolerance `-t` (relative to its true value), the cut at $y$ is added as a lazy constraint.
4. A candidate that receives a cut is rejected by the solver, although $y$ is a perfectly valid solution whose score is now known. GECOT hands it back with its true values, so that no solution found is lost.

The search ends when the solver has proved its best solution optimal for the master, whose cuts then describe the score exactly where it matters. A candidate is accepted when the master overestimates none of its targets by more than `-t`, so the result is exact up to this tolerance, to the solver's own, and to the [resolution](preprocessing.md#resolution) of the preprocessing.

## Parameters

The three decompositions need a solver that accepts lazy constraints in a callback: `--solver gurobi` (the default), `cplex` or `copt`. They take the [common parameters](../reference/parameters.md#common-to-the-milp-based-algorithms) and the [preprocessing parameters](../reference/parameters.md#preprocessing), plus:

| Algorithm | Parameters |
|---|---|
| `multicut_benders` | `--groups`, `--grouping`, `--grouping-seed`, `--big-m` |
| `flow_benders` | `--big-m` |
| `multiflow_benders` | `--no-root-cuts`, `--no-in-out` |

Their report adds `lazy_constraints`, the number of cuts added during the search, and for `multiflow_benders` `sources`, `root_rounds`, `root_cuts` and `root_relaxation_value`.
