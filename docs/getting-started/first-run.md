# A first run

This page runs GECOT on a landscape small enough to check every number by hand. It shows the two commands, `gecot optimize` and `gecot bound`, how to read their output, and why optimizing the actions together is not the same as ranking them.

## Three woods

A squirrel population lives in three woods. Wood A and wood B are close to each other, wood C is further away, beyond a road:

<figure class="gecot-figure">
<svg viewBox="0 0 560 150" width="560" role="img" aria-label="Three woods in a row. Wood A has quality 2, wood B quality 1 and wood C quality 3. A and B are linked with probability 0.5, raised to 0.9 by the hedgerow. B and C are linked with probability 0.2, raised to 0.8 by the underpass. The plantation adds 1 to the quality of wood B.">
  <defs>
    <marker id="tw-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10 z" fill="currentColor"/></marker>
  </defs>
  <g font-size="13" fill="currentColor" stroke="none">
    <line x1="110" y1="62" x2="250" y2="62" stroke="currentColor" stroke-width="1.6" marker-start="url(#tw-arrow)" marker-end="url(#tw-arrow)"/>
    <line x1="310" y1="62" x2="450" y2="62" stroke="currentColor" stroke-width="1.6" marker-start="url(#tw-arrow)" marker-end="url(#tw-arrow)"/>
    <circle class="habitat" cx="80" cy="62" r="26" stroke-width="1.5"/>
    <circle class="habitat" cx="280" cy="62" r="26" stroke-width="1.5"/>
    <circle class="habitat" cx="480" cy="62" r="26" stroke-width="1.5"/>
    <text class="habitat-label" x="80" y="67" text-anchor="middle" font-size="15">2</text>
    <text class="habitat-label" x="280" y="67" text-anchor="middle" font-size="15">1</text>
    <text class="habitat-label" x="480" y="67" text-anchor="middle" font-size="15">3</text>
    <text x="80" y="108" text-anchor="middle">wood_a</text>
    <text x="280" y="108" text-anchor="middle">wood_b</text>
    <text x="480" y="108" text-anchor="middle">wood_c</text>
    <text x="180" y="52" text-anchor="middle">0.5</text>
    <text x="380" y="52" text-anchor="middle">0.2</text>
    <text class="improved-fill" x="180" y="82" text-anchor="middle">hedgerow: 0.9</text>
    <text class="improved-fill" x="380" y="82" text-anchor="middle">underpass: 0.8</text>
    <text class="improved-fill" x="280" y="128" text-anchor="middle">plantation: +1</text>
  </g>
</svg>
<figcaption>The qualities of the woods are in the circles, the probabilities of moving between two woods above the arcs, and what each option changes in orange.</figcaption>
</figure>

- The **quality** of a wood stands for its area: 2, 1 and 3.
- The **probability** that a squirrel succeeds in moving between A and B is 0.5 in each direction, and 0.2 between B and C. From A to C it must go through B: $0.5 \times 0.2 = 0.1$.

Three **options** can be funded:

| Option | Cost | Effect |
|---|---:|---|
| `hedgerow` | 2 | the probability between A and B rises to 0.9 |
| `underpass` | 3 | the probability between B and C rises to 0.8 |
| `plantation` | 2 | the quality of wood B rises by 1 |

The whole instance is one JSON file, [`three-woods.json`](../examples/three-woods.json):

```json
{
  "options": [
    { "id": "hedgerow", "cost": 2 },
    { "id": "underpass", "cost": 3 },
    { "id": "plantation", "cost": 2 }
  ],
  "cases": {
    "squirrel": {
      "vertices": [
        { "id": "wood_a", "source_quality": 2, "target_quality": 2 },
        { "id": "wood_b", "source_quality": 1, "target_quality": 1 },
        { "id": "wood_c", "source_quality": 3, "target_quality": 3 }
      ],
      "arcs": [
        { "id": "a_b", "from": "wood_a", "to": "wood_b", "probability": 0.5 },
        { "id": "b_a", "from": "wood_b", "to": "wood_a", "probability": 0.5 },
        { "id": "b_c", "from": "wood_b", "to": "wood_c", "probability": 0.2 },
        { "id": "c_b", "from": "wood_c", "to": "wood_b", "probability": 0.2 }
      ],
      "vertices_improvements": [
        { "vertex_id": "wood_b", "option_id": "plantation",
          "source_quality_gain": 1, "target_quality_gain": 1 }
      ],
      "arcs_improvements": [
        { "arc_id": "a_b", "option_id": "hedgerow", "improved_probability": 0.9 },
        { "arc_id": "b_a", "option_id": "hedgerow", "improved_probability": 0.9 },
        { "arc_id": "b_c", "option_id": "underpass", "improved_probability": 0.8 },
        { "arc_id": "c_b", "option_id": "underpass", "improved_probability": 0.8 }
      ]
    }
  },
  "criterion": "squirrel"
}
```

A real instance keeps its tables in CSV files and has one case per species: see [Instance files](../usage/instance.md).

## The score

GECOT measures the connectivity of a landscape by adding, over every ordered pair of woods, the quality of the first times the quality of the second times the probability of the best path between them (a wood is connected to itself with probability 1). For the three woods as they are:

| Pairs | Computation | Value |
|---|---|---:|
| each wood with itself | $2 \times 2 + 1 \times 1 + 3 \times 3$ | 14 |
| A and B, both ways | $2 \times (2 \times 1 \times 0.5)$ | 2 |
| B and C, both ways | $2 \times (1 \times 3 \times 0.2)$ | 1.2 |
| A and C, both ways | $2 \times (2 \times 3 \times 0.1)$ | 1.2 |
| **Initial score** | | **18.4** |

This is the numerator of the Probability of Connectivity indicator: see [Problem and notation](../formulations/problem.md).

## Optimizing

With a budget of 4:

```console
$ gecot optimize flow_mip three-woods.json 4
[   0.0s] GECOT 2.0, algorithm flow_mip
[   0.0s] Instance /home/user/three-woods.json: 3 options, 1 species
[   0.0s]   squirrel: 3 vertices (3 habitats, 1 improvable), 4 arcs (4 improvable)
[   0.0s] Budget 4: 57.14 % of the options' total cost (7)
[   0.0s] Preprocessing the 'squirrel' graph
[   0.0s] Solving the optimization model
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

The lines that start with a time are the log, on the standard error stream; the rest is the result, on the standard output.

- `flow_mip` is an exact algorithm: its status `optimal` means that no selection of options within the budget scores more.
- The **solution score** can be checked as above. With the hedgerow and the plantation, wood B has quality 2 and the probabilities are 0.9 between A and B, 0.2 between B and C, 0.18 between A and C: $17 + 7.2 + 2.4 + 2.16 = 28.76$.
- The **upper bound** is what the algorithm proves that no solution exceeds. Here it equals the score: the solution reaches 100 % of the best possible gain.

`flow_mip` uses Gurobi by default: add `--solver highs`, or another [solver](installation.md#solvers) that you have, if needed.

## Ranking is not optimizing

Now with a budget of 3, compare the exact algorithm with `greedy`, a heuristic that repeatedly buys the option that gains the most per unit of cost (`-q` silences the log):

```console
$ gecot optimize flow_mip three-woods.json 3 -q
Status          optimal
Initial score   18.4
Solution score  25.6 (+39.13 %)
Upper bound     25.6 (at least 100.0 % of the best possible gain)
Cost            3 of 3
Time            2 ms
Options bought  1 of 3:
    underpass
```

```console
$ gecot optimize greedy three-woods.json 3 -q
Status          completed
Initial score   18.4
Solution score  24.6 (+33.70 %)
Cost            2 of 3
Time            0 ms
Options bought  1 of 3:
    plantation
```

Alone, the plantation gains $6.2$ for a cost of 2, and the underpass $7.2$ for a cost of 3. Per unit of cost the plantation is better, so the heuristic buys it, and then nothing else fits in the budget. The optimum is the underpass. The heuristic prints no upper bound: nothing in its output says that a better solution exists.

The options also interact. Alone, the hedgerow gains 2.56 and the underpass 7.2; together they gain 12.64, as they complete a corridor from A to C. No ranking of the options one by one can see it.

| Budget | Optimal options | Score |
|---:|---|---:|
| 2 | `plantation` | 24.6 |
| 3 | `underpass` | 25.6 |
| 4 | `hedgerow`, `plantation` | 28.76 |
| 5 | `underpass`, `plantation` | 35.4 |
| 7 | all | 42.44 |

Note that the optimal selections are not nested: the options of budget 3 are not kept at budget 4. This is why GECOT is run for one budget at a time.

## Bounding

On a large landscape an exact algorithm can take too long. A heuristic solution can still be given a guarantee by a bound computed separately:

```console
$ gecot bound path_lp three-woods.json 3 -q
Status             converged
Initial score      18.4
Upper bound        28.70286468 (at most +55.99 %)
All options bound  42.44
Budget             3
Time               2 ms
```

No solution of cost at most 3 scores more than 28.70, so the heuristic solution, which gains 6.2 where at most 10.30 can be gained, is proved to reach at least 60 % of the best possible gain. `path_lp` used as an algorithm gives both at once, a solution and this bound. The bound is not tight here, as the true optimum is 25.6; on real landscapes it is often much closer: see [Bounding](../usage/bound.md).

## Next steps

- Describe your own landscape: [Instance files](../usage/instance.md).
- The parameters of `gecot optimize`, time limits and solvers: [Optimizing](../usage/optimize.md).
- Which algorithm for which instance: [Choosing an algorithm](../usage/choosing.md).
