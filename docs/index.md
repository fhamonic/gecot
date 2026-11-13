---
# The H1 below is the hero wordmark; without this the tab reads "GECOT - GECOT".
# An empty title makes the template fall through to the bare site_name.
title: ""
---

<div class="gecot-hero" markdown>

![GECOT logo](assets/gecot.png)

# GECOT

**Graph-based Ecological Connectivity Optimization Tool.**

[Get started](getting-started/installation.md){ .md-button .md-button--primary }
[View on GitHub](https://github.com/fhamonic/gecot){ .md-button }

---

</div>

GECOT is an open-source command line tool for systematic conservation and restoration planning. Given a graph of the landscape for each species of interest, a list of actions that improve the patches and the corridors of these graphs, and a budget, it selects the actions to fund: those whose total cost fits the budget and that connect the landscape best, as measured by the Probability of Connectivity indicator.

```console
$ gecot optimize flow_mip three-woods.json 4
Status          optimal
Initial score   18.4
Solution score  28.76 (+56.30 %)
Upper bound     28.76 (at least 100.0 % of the best possible gain)
Cost            4 of 4
Time            2 ms
Options bought  2 of 3:
    hedgerow
    plantation
```

Three things set it apart from ranking the actions one by one:

- **It optimizes the actions together.** Two actions can be worth much more together than alone, or hardly more than one of them. GECOT's exact algorithms account for these cumulative effects and prove that their solution is optimal, or how far from the optimum it can be.
- **It scales.** A preprocessing of the landscape graphs shrinks the optimization models by orders of magnitude, and decompositions solve landscapes of thousands of patches with dozens of actions in seconds.
- **It handles several species.** Each species has its own graph, the actions are shared, and the criterion to maximize combines the species: a weighted sum, or the score of the worst-off one.

## Where to start

<div class="grid cards" markdown>

- **[Getting started](getting-started/installation.md)**

    Install GECOT and a solver, then [run it](getting-started/first-run.md) on a landscape of three woods whose numbers can be checked by hand.

- **[Using GECOT](usage/instance.md)**

    Describe a landscape and its actions in an [instance file](usage/instance.md), [model](usage/modeling.md) budgets of several kinds and exclusive actions, [optimize](usage/optimize.md) it, read the [results](usage/results.md), and [choose an algorithm](usage/choosing.md).

- **[Formulations](formulations/problem.md)**

    The [problem](formulations/problem.md), its [mixed integer formulation](formulations/flow.md), the [preprocessing](formulations/preprocessing.md), the [decompositions](formulations/benders.md), the [relaxations](formulations/relaxations.md) and the [bounds](formulations/bounds.md).

- **[Reference](reference/parameters.md)**

    Every [parameter](reference/parameters.md) of every algorithm, and the [notation](reference/notation.md) next to the columns of the instance files.

</div>

!!! info "Version"

    This documentation describes GECOT 2.0, the version of the `main` branch of the repository. The command of the 1.0 release was `gecot_optimize <algorithm> ...`: it is now `gecot optimize <algorithm> ...`, its algorithm `prep_mip` is `flow_mip`, and its `mip` is `flow_mip --preprocessing none`.

## Citation

Please cite GECOT when you use it in a publication:

> Hamonic, F., Vaxès, Y., Couëtoux, B., & Albert, C. H. (2025). GECOT: Graph-based ecological connectivity optimization tool. *Methods in Ecology and Evolution*, 00, 1–9. <https://doi.org/10.1111/2041-210X.70055>

The releases are archived on Zenodo: <https://doi.org/10.5281/zenodo.15295914>.

## Acknowledgments

This work has been funded by [Région Sud](https://www.maregionsud.fr/) and [Natural Solutions](https://www.natural-solutions.eu/) ([PhD thesis](https://theses.fr/api/v1/document/2023AIXM9063) of François Hamonic), and by the ERC project [SCALED](https://www.scaled-erc.eu/), grant n°949812 (postdoctoral position).

GECOT is free software, distributed under the [GNU General Public License v3](https://www.gnu.org/licenses/gpl-3.0.html).
