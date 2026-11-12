# The flow formulation

`flow_mip` solves the [problem](problem.md) as a mixed integer linear program (MILP) in which the score of each case is computed by generalized flows, one per target vertex. The three [Benders decompositions](benders.md) solve the same program without writing its flows, and the [preprocessing](preprocessing.md) shrinks it. This page builds the formulation in three steps, as the thesis does: a linear program for the score of a fixed graph, the options, then the cases and the criterion.

## A linear program for the score

Take one landscape graph $G = (V, A)$ and write its score target by target:

$$
PC_{\text{num}}(G) = \sum_{t \in V} q_t \cdot f_t\,, \qquad f_t = \sum_{s \in V} w_s \cdot \Pi(s,t)\,.
$$

$f_t$ is the quantity of source quality that reaches $t$ when every vertex $s$ sends its $w_s$ along a most reliable path, each arc $a$ keeping only the fraction $\pi_a$ of what enters it. It is the optimum of a *generalized flow* problem, a flow problem in which an arc multiplies the flow it carries. Let $\delta^{\text{out}}_u$ and $\delta^{\text{in}}_u$ be the sets of arcs that leave and enter a vertex $u$, and let $\phi_a \ge 0$ be the flow that enters the arc $a$:

$$
\begin{aligned}
(\mathcal{P}_t) \qquad \max \quad & f_t \\
\text{s.t.} \quad
& \sum_{a \in \delta^{\text{out}}_u} \phi_a - \sum_{b \in \delta^{\text{in}}_u} \pi_b \cdot \phi_b \le w_u && u \in V \setminus \{t\}\,, \\
& f_t + \sum_{a \in \delta^{\text{out}}_t} \phi_a - \sum_{b \in \delta^{\text{in}}_t} \pi_b \cdot \phi_b \le w_t\,, \\
& \phi_a \ge 0 && a \in A\,.
\end{aligned}
$$

The first constraints say that what leaves $u$ is at most its own quality plus what arrives there. The second defines $f_t$ as what is available at $t$ and does not leave it.

!!! abstract "Lemma"

    The optimal value of $(\mathcal{P}_t)$ is $f_t = \sum_{s \in V} w_s \cdot \Pi(s,t)$.

Sending $w_s$ units from $s$ along a path $P$ brings $w_s \cdot \prod_{a \in P} \pi_a$ to $t$. A solution that routes some flow along a path that is not a most reliable one is improved by moving this flow to a most reliable one, so an optimal solution sends the quality of every vertex along most reliable paths, and nothing leaves $t$.

The score of the whole graph is the optimum of the linear program that puts the programs $(\mathcal{P}_t)$ side by side, with a copy $\phi^t$ of the flow variables for each target $t$, and maximizes $\sum_{t \in V} q_t \cdot f_t$.

## The options

Let $x_i \in \{0,1\}$ tell whether the option $i \in \Phi$ is bought. Three things change in the program above.

**Improved arcs.** For each arc $a = (u,v)$ and each option $i$ that improves it, add an *improved copy* $a^i = (u,v)$ of probability $\pi_{a^i} = \pi^i_a$, which can carry flow only if $i$ is bought. Let $A'$ be the set of the improved copies and $\Psi = A \cup A'$; from now on $\delta^{\text{out}}_u$, $\delta^{\text{in}}_u$ and the flows are those of $\Psi$. A *big-M constraint* closes the copy when the option is not bought:

$$
\phi^t_{a^i} \le M^t_u \cdot x_i\,,
$$

where the constant $M^t_u$ is an upper bound on the flow that can reach $u$ on its way to $t$. A flow prefers the copy of largest probability among those that are open, so the arc behaves as if it had the probability $\pi^x_a$.

**Source quality gains.** The quality available at $u$ becomes $w_u + \sum_{i \in \Phi} w^i_u \cdot x_i$, which is linear.

**Target quality gains.** The contribution of a target becomes $\big(q_t + \sum_{i \in \Phi} q^i_t \cdot x_i\big) \cdot f_t$, a product of variables. It is linearized with one variable $f^i_t$ per gain, equal to $f_t$ if $i$ is bought and to $0$ otherwise (a McCormick linearization):

$$
f^i_t \le f_t\,, \qquad f^i_t \le M^t_t \cdot x_i\,,
$$

so that the contribution reads $q_t \cdot f_t + \sum_{i \in \Phi} q^i_t \cdot f^i_t$. As the program maximizes and the coefficients $q^i_t$ are positive, $f^i_t$ takes the largest value that the two constraints allow.

## Cases and criterion

Each case $k \in K$ gets its own flows on its own graph, and a variable $z_k$ for its score. In the complete program below, every constraint that mentions $t$ is written for each case $k \in K$ and each vertex $t \in V_k$, $u$ ranges over the vertices of the same case, and in the constraint of an improved copy $a^i$ it is the vertex that the copy leaves:

$$
\begin{aligned}
\max \;\; & \mathcal{C}(z) \\
\text{s.t.} \;\;
& z_k \le \sum_{t \in V_k} \Big( q_t \cdot f_t + \sum_{i \in \Phi} q^i_t \cdot f^i_t \Big) && k \in K\,, \\
& \sum_{a \in \delta^{\text{out}}_u} \phi^t_a - \sum_{b \in \delta^{\text{in}}_u} \pi_b \cdot \phi^t_b \le w_u + \sum_{i \in \Phi} w^i_u \cdot x_i && u \neq t\,, \\
& f_t + \sum_{a \in \delta^{\text{out}}_t} \phi^t_a - \sum_{b \in \delta^{\text{in}}_t} \pi_b \cdot \phi^t_b \le w_t + \sum_{i \in \Phi} w^i_t \cdot x_i\,, \\
& \phi^t_{a^i} \le M^t_u \cdot x_i && a^i \in A'\,, \\
& f^i_t \le f_t\,, \quad f^i_t \le M^t_t \cdot x_i && i \in \Phi\,, \\
& \sum_{i \in \Phi} c_i \cdot x_i \le \beta\,, \\
& x \in \{0,1\}^\Phi\,, \quad \phi\,,\, f\,,\, z \ge 0\,.
\end{aligned}
$$

For a single case with $q = w$ and $\mathcal{C}(z) = z$, this is the formulation of BC-PC-Opt in the thesis. It has $|\Phi|$ binary variables and, for each case, about $|V_k| \cdot |\Psi_k|$ flow variables and $|V_k|^2$ flow constraints: its size is what limits it, and what the [preprocessing](preprocessing.md) attacks.

### Linearizing the criterion

The objective $\mathcal{C}(z)$ is written with one variable $v$ per sub-formula of the criterion:

| Sub-formula | Constraint |
|---|---|
| a constant $c$ | $v = c$ |
| the score of the case $k$ | $v = z_k$ |
| a sum of formulas of variables $v_1, \dots, v_n$ | $v = v_1 + \dots + v_n$ |
| the product of a constant $c$ by a formula of variable $v_1$ | $v = c \cdot v_1$ |
| the minimum of formulas of variables $v_1, \dots, v_n$ | $v \le v_j$ for $j = 1, \dots, n$ |

and the objective maximizes the variable of the whole formula. A product of two formulas that both depend on the scores is not linear, and the algorithms built on this program refuse it.

!!! note "The criterion must be non-decreasing in every case score"

    The program bounds each score $z_k$ from above by the flows, and each minimum from above by its terms. This is exact only if the objective pushes them up, that is, if the criterion never decreases when the score of a case increases: sums, products by non-negative constants and minimums. With a criterion such as $-z_k$, the solver would simply set $z_k = 0$. `flow_mip`, the Benders decompositions and the relaxations therefore refuse such a criterion: see [which formulas each algorithm accepts](../usage/instance.md#criterion).

## What `flow_mip` solves

`flow_mip` builds this program and hands it to a MILP solver through [mippp](https://github.com/fhamonic/mippp), with a few differences that do not change its optimum:

- the instance is first [reformulated](preprocessing.md#the-instance-given-to-the-algorithms): options of cost $0$ are bought, options over the budget are removed, vertices linked by arcs of probability $1$ are merged, and qualities are rescaled;
- the flows of a target $t$ live on its own reduced graph $G_t$, without the arcs that are useless for $t$ and with its strong arcs contracted, and vertices that can never be targets ($\overline{q}_t = 0$) have no flow at all;
- the constants $M^t_u$ are computed on $G_t$ [within the budget](preprocessing.md#big-m-constants).

The solver's relative optimality gap is set to $10^{-10}$, and `-t` sets its feasibility tolerance. The status `optimal` therefore means that the solution is proved optimal up to these tolerances and up to the [resolution](preprocessing.md#resolution) of the preprocessing. When the run stops earlier (`--time-limit`, Ctrl-C), the solution is the best one found and the upper bound is the solver's dual bound: see [Results and logs](../usage/results.md).

`--lp-relaxation` relaxes $x \in \{0,1\}^\Phi$ to $x \in [0,1]^\Phi$ and solves the linear program, whose value bounds the optimum from above. It is an experimental switch, meant to measure how tight the formulation is: the value of the relaxation is the upper bound of the result, and what `gecot bound flow_mip` returns with it. The options printed are only those that the relaxation buys entirely, which always fit in the budget.

## Parameters

`flow_mip` takes the [common parameters](../reference/parameters.md#common-to-the-milp-based-algorithms) of the MILP-based algorithms and the [preprocessing parameters](../reference/parameters.md#preprocessing). Its `--solver` can be `gurobi` (the default), `cplex`, `copt`, `xpress`, `mosek`, `highs`, `scip`, `cbc` or `glpk`.
