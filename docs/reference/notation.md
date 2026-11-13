# Notation

The symbols of the [formulation pages](../formulations/problem.md), next to the column of the [instance file](../usage/instance.md), the parameter or the output field they correspond to. The notation follows the thesis and the article cited on the [home page](../index.md#citation); the symbols marked ★ are those that GECOT 2.0 adds to it.

## Instance

| Symbol | Meaning | In GECOT |
|---|---|---|
| $K$ ★ | the cases, one landscape graph each | the keys of `cases` |
| $G = (V, A)$, $G_k = (V_k, A_k)$ | the landscape graph (of the case $k$) | the tables `vertices` and `arcs` |
| $w_u$ | source quality of the vertex $u$ (the *weight* of the thesis) | column `source_quality` |
| $q_u$ ★ | target quality of the vertex $u$ | column `target_quality` |
| $\pi_a$, $\pi_{uv}$ | probability of the arc $a = (u,v)$ | column `probability` |
| $\Phi$ | the options | the table `options` |
| $c_i$ | cost of the option $i$ | column `cost` |
| $w^i_u$ | source quality gain of $u$ with the option $i$ | column `source_quality_gain` |
| $q^i_u$ ★ | target quality gain of $u$ with the option $i$ | column `target_quality_gain` |
| $\pi^i_a$ | probability of $a$ with the option $i$ | column `improved_probability` |
| $\beta$ | the budget | argument `<budget>` |
| $\mathcal{C}$ ★ | the criterion, a function of the scores of the cases | key `criterion` |

## Scenarios and score

| Symbol | Meaning | In GECOT |
|---|---|---|
| $x \in \{0,1\}^\Phi$ | a scenario: $x_i = 1$ if the option $i$ is bought | field `solution` |
| $G^x$, $w^x_u$, $q^x_u$, $\pi^x_a$ | the graph, the qualities and the probabilities in the scenario $x$ | |
| $\overline{w}_u$, $\overline{q}_u$, $\overline{\pi}_a$ | their values with every option bought | |
| $S \subseteq \Phi$ | a set of options, identified with the scenario that buys it | |
| $\Pi(s,t)$, $\Pi^x(s,t)$ | probability of a most reliable path from $s$ to $t$ (in the scenario $x$) | |
| $\overline{\Pi}(s,t)$ | the same with every option bought | |
| $PC_{\text{num}}(G)$ | score of a landscape graph, $\sum_{s,t} w_s \cdot q_t \cdot \Pi(s,t)$ | fields `initial_pc_num`, `solution_pc_num` |
| $\mathcal{C}(x)$ | score of a scenario: the criterion applied to the scores of the cases | fields `initial_score`, `solution_score` |

## Flow formulation

| Symbol | Meaning | In GECOT |
|---|---|---|
| $\delta^{\text{out}}_u$, $\delta^{\text{in}}_u$ | the arcs that leave and enter the vertex $u$ | |
| $a^i$, $A'$, $\Psi = A \cup A'$ | the improved copy of the arc $a$ for the option $i$, the set of the copies, all the arcs | |
| $f_t$ | flow of source quality that reaches the target $t$, $\sum_s w_s \cdot \Pi(s,t)$ | |
| $\phi^t_a$ | flow that enters the arc $a$ on its way to $t$ | |
| $f^i_t$ | the variable equal to $f_t$ if $i$ is bought and to 0 otherwise | |
| $z_k$ ★ | score of the case $k$ | |
| $M^t_u$ | big-M constant: a bound on the flow that reaches $u$ on its way to $t$ | parameter `--big-m` |

## Preprocessing

| Symbol | Meaning | In GECOT |
|---|---|---|
| $\mathbb{S}(t)$, $\mathbb{W}(t)$ | the $t$-strong and the $t$-useless arcs | |
| $S(t)$, $W(t)$ | the subsets that GECOT computes, with continuous scenarios | parameter `--preprocessing` |
| $[\,l^-_a, l^+_a\,]$ | interval of the length $-\log \pi$ of the arc $a$ | |
| $l^x_a$, $d^x(s,t)$ | length of an arc and distance in the scenario $x$ | |
| $F^x(u,v)$, $F(u,v)$ | fiber of the arc $(u,v)$ in the scenario $x$, and in every scenario | |
| $r$ | resolution of the integer lengths | parameter `--resolution` |
| $G_t$ | reduced graph of the target $t$ | |
| $\mu$, $\lambda^\mu_a$ | Lagrange multiplier of the budget-aware labelling, and the lengths it penalizes | parameters `--nb-mus`, `--mus-sweep` |
| $A_i$ | the arcs that the option $i$ improves | |

## Decompositions and relaxations

| Symbol | Meaning | In GECOT |
|---|---|---|
| $\rho_u$ | potential of the vertex $u$: dual variable of its flow conservation | |
| $\sigma_{a^i}$ | dual variable of the big-M constraint of an improved copy | |
| $y$ | reference scenario of a cut | |
| $\operatorname{cut}_{t,y}(x)$ | cut of the target $t$ at $y$, a linear function of $x$ | field `lazy_constraints` |
| $v_t(x)$ | value of the target $t$, $q^x_t \cdot f_t(x)$ | |
| $\theta_g$, $\mathcal{G}_k$ | value of the group of targets $g$, and the groups of the case $k$ | parameter `--groups` |
| $A_s$, $m_{sa}$, $\rho^s$ | arcs, big-M constants and potentials of the source $s$ in `multiflow_benders` | field `sources` |
| $p = (s,t)$ | a pair of vertices | field `pairs` |
| $v_p(S)$, $v_t(S)$ | value of a pair, of a target, when the set $S$ is bought | |
| $y_{pS}$, $y_{tS}$ | columns of the path and tree formulations | |
| $\gamma_k$, $\sigma_p$, $\lambda_{pi}$, $\eta$ | dual variables of the contribution, uniqueness, purchase and budget constraints | |
| $r_{pS}$, $r_p$ | reduced cost of a column, best reduced cost of a pair | |
| $L$ | Lagrangian bound | field `best_bound` of `path_lp` |
| $d_i$ | reduced cost of the option $i$ in the relaxation | option `--reformulation path_lp` |

## Heuristics and bounds

| Symbol | Meaning | In GECOT |
|---|---|---|
| $e_i$ | the scenario that buys the option $i$ alone | |
| $\Delta^+_i(x)$, $\Delta^-_i(x)$ | gain and loss of the option $i$ per unit of cost, in the scenario $x$ | |
| $b$ | an upper bound on the score of the scenarios within the budget | field `bound`, line `Upper bound` |
| $s_0$ | score with only the options of cost 0 | |
| $H$, $l_i$, $Q_{ij}$ | constant, linear and quadratic terms of the bound of `qkp_planes` | |
| $\theta_{ij}$, $u_i$ | split of the quadratic terms, and the resulting bound on what the option $i$ brings | parameter `--theta` |
| $\nu_{pi}$, $R_p(\nu)$ | prices of `split_lagrange`, and the best priced value of a pair | |
