# Modeling with costs and conflicts

[Several costs](instance.md#several-costs) and [conflict sets](instance.md#conflict-sets) are two plain constraints: a budget per cost dimension, and at most one option of each set. This page shows three ways to model a landscape with them, each on a variant of the [three woods](../getting-started/first-run.md) whose numbers can be checked by hand.

!!! warning "Supported algorithms"

    Not every algorithm solves instances with several costs or conflict sets yet: see [which ones do](choosing.md#conflict-sets-and-several-costs).

## Different kinds of actions

A single budget adds up costs that are not comparable: a number of wildlife crossings that a road administration will allow, a planting area that a forest service can fund, money. Give each kind of resource its own cost dimension, and each option a cost in the dimensions it uses.

The three woods with three resources, in [`three-woods-resources.json`](../examples/three-woods-resources.json):

| Option | `money` | `area` (ha) | `crossings` |
|---|---:|---:|---:|
| `hedgerow` | 2 | 1 | 0 |
| `underpass` | 3 | 0 | 1 |
| `plantation` | 2 | 2 | 0 |

```json
"options": [
  { "id": "hedgerow", "cost": { "money": 2, "area": 1, "crossings": 0 } },
  { "id": "underpass", "cost": { "money": 3, "area": 0, "crossings": 1 } },
  { "id": "plantation", "cost": { "money": 2, "area": 2, "crossings": 0 } }
]
```

"Money for everything, one crossing and two hectares of planting" is the budget `money=7,area=2,crossings=1`:

```console
$ gecot optimize flow_mip three-woods-resources.json money=7,area=2,crossings=1
...
Status          optimal
Initial score   18.4
Solution score  35.4 (+92.39 %)
Upper bound     35.4 (at least 100.0 % of the best possible gain)
Cost area       2 of 2
Cost crossings  1 of 1
Cost money      5 of 7
Time            2 ms
Options bought  2 of 3:
    underpass
    plantation
```

The money would buy all three options, but the area allows only one of the plantings. With `area=3`, all three are bought (42.44). With `crossings=0`, no underpass can be built, and the plantation alone is the best (24.6).

- **A count is a cost.** A dimension whose costs are 0 or 1, as `crossings`, limits how many options of a kind are bought.
- **Budgets per year or per region.** One dimension per year, or per region, where an option costs 0 outside its own. The budget of a year cannot be spent in another one: solving the same instance with a single summed budget shows what this split costs.
- **Every option must cost something.** An option of cost 0 in every dimension, and in no conflict set, is always bought.
- **No weights.** The dimensions are never added together, so their units do not matter.

## Exclusive choices on one element

Some actions are alternatives: a patch can be replanted in one way or in another, a dam removed or given a fish pass, a crossing built as a bridge or as a tunnel. Without a constraint, GECOT could buy two alternatives together, and since quality gains add up, their effects would add up too. A conflict set says that at most one of them is bought.

Wood B can now be restored in two ways, in [`three-woods-alternatives.json`](../examples/three-woods-alternatives.json):

| Option | Cost | Effect |
|---|---:|---|
| `plantation` | 2 | the quality of wood B rises by 1 |
| `forest` | 3 | the quality of wood B rises by 2 |

```json
"conflicts": [
  { "set_id": "wood_b", "option_id": "plantation" },
  { "set_id": "wood_b", "option_id": "forest" }
]
```

| Budget | Without the set | With the set |
|---:|---|---|
| 5 | `plantation`, `forest`: 43 | `hedgerow`, `forest`: 38.56 |
| 7 | `hedgerow`, `plantation`, `forest`: 50.36 | `underpass`, `forest`: 47.2 |
| 8 | `underpass`, `plantation`, `forest`: 61 | `hedgerow`, `underpass`, `forest`: 55.84 |

Without the set, the best use of a budget of 5 is to plant wood B twice over: a quality of 4 that no real restoration gives.

- **Levels of one action.** Give each level its whole effect, here +1 and +2, and put the levels in one set. Increments that add up would need an order between them, which a set cannot express.
- **Alternatives on one arc.** Options that improve the same arc already compete: the arc takes the best of their probabilities, so a set does not change the best score. With budget to spare, though, GECOT may also list the alternative that adds nothing; the set keeps the plan to one. A set changes the best score when the gains add up, on vertices, and when the alternatives improve several arcs or serve different cases: two crossings at one site, one for the deer and one for the squirrels, both count if both are allowed.
- **Pairwise conflicts.** Sets may overlap. Two options that only exclude each other form a set of two.

## Actions that harm the landscape

An action that improves one part of the landscape can harm another: the works of an underpass destroy part of a wood, a plantation replaces a meadow that another species lives in. GECOT's options only improve the landscape, so the harm is modeled the other way round:

- the landscape is described as it would be **after the harm**;
- a **free option** gives back what the harm destroys;
- a **conflict set** joins the free option and the harmful action.

The free option stays a decision, because its set holds another option that the budget allows. GECOT keeps it, which costs nothing, unless the harmful action is worth more than what it destroys.

In [`three-woods-works.json`](../examples/three-woods-works.json), the works of the underpass destroy a third of wood C. Wood C has a quality of 2, the free option `wood_c_intact` gives it back its third, and the set `underpass_works` holds the two:

```json
"options": [
  { "id": "hedgerow", "cost": 2 },
  { "id": "underpass", "cost": 3 },
  { "id": "plantation", "cost": 2 },
  { "id": "wood_c_intact", "cost": 0 }
],
"conflicts": [
  { "set_id": "underpass_works", "option_id": "underpass" },
  { "set_id": "underpass_works", "option_id": "wood_c_intact" }
]
```

| Budget | Options bought | Score | Without the harm |
|---:|---|---:|---:|
| 3 | `plantation`, `wood_c_intact` | 24.6 | 25.6 |
| 5 | `hedgerow`, `plantation`, `wood_c_intact` | 28.76 | 35.4 |
| 7 | `hedgerow`, `underpass`, `plantation` | 31.36 | 42.44 |

Up to a budget of 5 the underpass is not worth a third of wood C. At 7, it is: the underpass with the hedgerow and the plantation scores 31.36, against 28.76 for keeping the wood intact. The last column is the original three woods, where the works harm nothing.

**Reading the results.** The *initial score* is the score with nothing bought, so here the landscape after the harm: 12.6. This is the convention of [conservation problems](../formulations/problem.md#options), where the landscape is described as it would be if nothing were protected, and the free option is a conservation action among restoration ones. The printed gains, `+148.89 %` at a budget of 7, are relative to the landscape after the harm, and so is the guaranteed share while the harmful action fits in the budget.

The landscape as it is scores 18.4. A run with a budget of 0 gives it: the harmful action is then over the budget, so the free option is bought statically.

```console
$ gecot optimize flow_mip three-woods-works.json 0 -q
Status          no option left
Initial score   12.6
Solution score  18.4 (+46.03 %)
...
```

```console
$ gecot optimize flow_mip three-woods-works.json 7 -q
Status          optimal
Initial score   12.6
Solution score  31.36 (+148.89 %)
...
Options bought  3 of 4:
    hedgerow
    underpass
    plantation
```

- **Several harmful actions.** If two actions harm the same element, use one set for each, $\{B, A_1\}$ and $\{B, A_2\}$. A single set $\{B, A_1, A_2\}$ would also forbid $A_1$ together with $A_2$.
- **Harms of different extents.** The landscape is described after the worst harm, and each layer of harm gets its own free option, in a set with every action that destroys it. If the underpass costs wood C a third of its area and a bridge two thirds, wood C has a quality of 1. `keep_1` (+1) is in a set with the bridge. `keep_2` (+1) is in a set with the underpass and in another with the bridge. With neither action, wood C keeps its 3; with the underpass, 2; with the bridge, 1.
- **Harm to another species.** The free option can improve another case: a plantation that replaces a meadow lowers the quality of the meadow for a butterfly, and the free option gives that quality back in the butterfly's case. The criterion then weighs the two species.
- **When the harmful action is over the budget.** The set then constrains nothing: the free option is bought statically and listed as bought.
- **With the heuristics.** `static` and `greedy` buy a free option first, as it costs nothing. Only `greedy` gives it back for the harmful action, through a [swap](../formulations/heuristics.md#conflict-sets-and-several-costs), and never when the action is in a set with two free layers. Use an exact algorithm to weigh a harm.
