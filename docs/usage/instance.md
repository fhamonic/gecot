# Instance files

An instance describes the landscape, the options and what to maximize. It is a JSON file that names the options, one or several *cases* (a landscape graph each, typically one per species) and a *criterion*. The tables themselves, vertices, arcs and improvements, are usually CSV files that the JSON file points to.

## Structure

```json
{
  "options": { "csv_file": "options.csv" },
  "cases": {
    "squirrel": {
      "vertices": { "csv_file": "squirrel/vertices.csv" },
      "arcs": { "csv_file": "squirrel/arcs.csv" },
      "vertices_improvements": { "csv_file": "squirrel/vertices_gains.csv" },
      "arcs_improvements": { "csv_file": "squirrel/arcs_gains.csv" }
    },
    "hedgehog": {
      "vertices": { "csv_file": "hedgehog/vertices.csv" },
      "arcs": { "csv_file": "hedgehog/arcs.csv" },
      "arcs_improvements": { "csv_file": "hedgehog/arcs_gains.csv" }
    }
  },
  "criterion": {
    "operation": "sum",
    "values": [ "squirrel", [0.5, "hedgehog"] ]
  }
}
```

| Key | Required | Content |
|---|---|---|
| `options` | yes | the table of the options, shared by all the cases |
| `conflicts` | no | the table of the [conflict sets](#conflict-sets): at most one option of each set is bought |
| `cases` | yes | an object with one entry per case, whose key is the name of the case |
| `criterion` | no | the formula to maximize; by default the sum of the scores of all the cases |

No other key is accepted at the top level. Each case has the tables `vertices` and `arcs`, and at least one of `vertices_improvements` and `arcs_improvements`.

## Tables

A table is either a CSV file or an array written in the JSON file itself.

**CSV file.** The table is an object with the path of the file in `csv_file`. A relative path is relative to the directory of the JSON file. The first line of the CSV file names its columns; the columns that GECOT does not read (coordinates, names...) are ignored.

**Inline.** The table is an array of objects whose keys are the column names, as in the [three woods](../getting-started/first-run.md) example:

```json
"options": [
  { "id": "hedgerow", "cost": 2 },
  { "id": "underpass", "cost": 3 }
]
```

In an inline table the identifiers must be JSON strings: `"id": "12"`, not `"id": 12`.

### Columns

| Table | Column | Meaning | Symbol |
|---|---|---|---|
| `options` | `id` | identifier of the option | $i \in \Phi$ |
| | `cost` | its cost, at least 0, or its costs in [several dimensions](#several-costs) | $c_i$, or $c_{i,r}$ |
| `conflicts` | `set_id` | identifier of a conflict set | $l$ |
| | `option_id` | an option of the set | $i \in \Omega_l$ |
| `vertices` | `id` | identifier of the vertex, unique in its case | $u \in V$ |
| | `source_quality` | quality as a source, at least 0 | $w_u$ |
| | `target_quality` | quality as a target, at least 0 | $q_u$ |
| `arcs` | `id` | identifier of the arc, unique in its case | $a \in A$ |
| | `from` | identifier of the vertex it leaves | |
| | `to` | identifier of the vertex it enters | |
| | `probability` | probability of crossing it, in $[0,1]$ | $\pi_a$ |
| `vertices_improvements` | `vertex_id` | the vertex improved | |
| | `option_id` | the option that improves it | |
| | `source_quality_gain` | added to its source quality, at least 0 | $w^i_u$ |
| | `target_quality_gain` | added to its target quality, at least 0 | $q^i_u$ |
| `arcs_improvements` | `arc_id` | the arc improved | |
| | `option_id` | the option that improves it | |
| | `improved_probability` | its probability with the option, in $[0,1]$ | $\pi^i_a$ |

The symbols are those of [Problem and notation](../formulations/problem.md).

### Columns with other names

When the columns of a CSV file have other names, `csv_columns` maps the names above to those of the file. The trout instance of the repository (`test/instances/Aude/aude.json`) reads the lengths of river stretches as both qualities, and dams as options:

```json
"vertices": {
  "csv_file": "stretches.csv",
  "csv_columns": {
    "id": "id",
    "source_quality": "length",
    "target_quality": "length"
  }
},
"arcs_improvements": {
  "csv_file": "dams_arcs.csv",
  "csv_columns": {
    "arc_id": "arc_id",
    "option_id": "dam_id",
    "improved_probability": "improved_prob"
  }
}
```

```text title="stretches.csv"
name,id,length,x,y
AUD-Puy,0,235,400,70
AUD-Car-1,1,60,385,180
AUD-Car-2,2,120,350,140
```

Only the columns that are renamed need to be listed.

## What the tables mean

**Arcs are directed.** A connection that works both ways is two arcs, one in each direction, and an option that improves both lists both.

**An option can improve many things.** The same `option_id` can appear on any number of rows, of both improvement tables, of any number of cases. This is how a single action serves several species, or how a corridor made of many arcs is funded as a whole.

**Quality gains add up, probabilities compete.** If several options improve the same vertex, their gains add up. If several options improve the same arc, the arc takes the best of their probabilities. An improved probability lower than the probability of the arc has no effect.

**Source and target qualities.** The score of a case adds, over every ordered pair of vertices $(s,t)$, the source quality of $s$ times the target quality of $t$ times the probability of the best path from $s$ to $t$. Give both columns the same value, the habitat area for instance, to obtain the Probability of Connectivity indicator. Give them different values when leaving a place and reaching it are not worth the same, and $0$ to both for a vertex that is only a place to cross.

**Free and unaffordable options.** An option that costs more than the budget, in any dimension, is ignored. An option of cost $0$ in every dimension is always bought, unless it belongs to a conflict set with another affordable option: buying it would then exclude that option, and it stays a choice. This is how [an action that harms the landscape](modeling.md#actions-that-harm-the-landscape) is modeled.

**Probability 1.** Vertices linked both ways by arcs of probability $1$ behave as a single vertex, and GECOT [merges them](../formulations/preprocessing.md#the-instance-given-to-the-algorithms). A raster landscape can thus be given cell by cell, with arcs of probability $1$ inside a habitat patch.

## Several costs

An option can have a cost in several dimensions, money and area for instance, or the budget of several years or regions, an option costing $0$ outside its own. Each dimension has its own budget, and a solution must fit in every one. The dimensions are named by an object in place of the cost: in a CSV file, `csv_columns` maps each dimension to its column,

```json
"options": {
  "csv_file": "options.csv",
  "csv_columns": { "cost": { "money": "cost_eur", "area": "area_m2" } }
}
```

and an inline table gives the costs of every option under the same names:

```json
"options": [
  { "id": "hedgerow", "cost": { "money": 2, "area": 1 } },
  { "id": "underpass", "cost": { "money": 3, "area": 0 } }
]
```

The budget then names each dimension once, as `money=5,area=3` (see [`gecot optimize`](optimize.md)). A name cannot contain `=` or `,`. A plain cost is one dimension named `cost`. The budget can give the dimensions in any order, and the outputs list them in alphabetical order. [Modeling with costs and conflicts](modeling.md#different-kinds-of-actions) shows how to give each kind of action its own budget.

## Conflict sets

A conflict set is a group of options of which at most one is bought: alternative actions at one site, a dam removed or given a fish pass, a crossing built as a bridge or as a tunnel. Each row of the `conflicts` table puts an option in a set, and a set has as many rows as options. An option can belong to several sets: a pairwise conflict is a set of two.

```json
"conflicts": [
  { "set_id": "dam_12", "option_id": "dam_12_removal" },
  { "set_id": "dam_12", "option_id": "dam_12_fish_pass" }
]
```

A set changes the best score where the effects of two options would add up. Options that improve the same arc already compete, the best probability counts, so a second one adds nothing to the score; the set then only keeps the plan from listing it when the budget allows. Their quality gains add up, though, and a set is what forbids, for instance, two restorations of one patch, or two actions on one dam that serve different species. [Modeling with costs and conflicts](modeling.md#exclusive-choices-on-one-element) shows such choices.

!!! warning "Supported algorithms"

    For now, only `flow_mip`, `multicut_benders` and `flow_benders` solve instances with conflict sets or several costs, and only the `all_options` bound and these three algorithms bound them. The other algorithms and bounds, `--mip-start-algorithm` and `--reformulation path_lp`, stop with the error `conflict sets and several cost dimensions are not supported yet`; `--mip-start <file>` works. The check is made once the options over the budget are removed: with a single cost, a budget that leaves no conflict set with two affordable options runs with every algorithm.

## Criterion

The criterion is a formula over the scores of the cases:

| JSON | Formula |
|---|---|
| `"squirrel"` | the score of the case `squirrel` |
| `2.5` | a constant |
| `[0.5, f]` | the constant times the formula `f` |
| `{"operation": "sum", "values": [f, g, ...]}` | the sum of the formulas |
| `{"operation": "min", "values": [f, g, ...]}` | the smallest of the formulas |
| `{"operation": "product", "values": [f, g, ...]}` | the product of the formulas |

For instance $PC_{\text{squirrel}} + \tfrac{1}{2} PC_{\text{hedgehog}}$, then the score of the worst-off of three species once each is given a weight:

```json
"criterion": {
  "operation": "sum",
  "values": [ "squirrel", [0.5, "hedgehog"] ]
}
```

```json
"criterion": {
  "operation": "min",
  "values": [ "squirrel", [3, "hedgehog"], [0.2, "toad"] ]
}
```

The scores of two cases can differ by orders of magnitude, since they grow with the square of the qualities: weights are what makes a minimum or a sum meaningful. The initial score of each case, printed in the `initial_pc_num` field of the [JSON output](results.md#json), is a natural basis for them.

### Which formulas the algorithms accept

| Criterion | Heuristics | `flow_mip`, Benders, `path_lp`, `tree_lp` | `all_options`, `qkp_planes`, `split_lagrange` |
|---|---|---|---|
| sums, minimums, products by non-negative constants | yes | yes | yes |
| product of formulas that depend on the scores | yes | error | yes |
| decreasing in the score of a case (a negative constant in a product, for instance) | evaluated as written | error | error |

!!! note "Keep the criterion non-decreasing"

    The exact algorithms and the relaxations [model the score of each case as an upper bound](../formulations/flow.md#linearizing-the-criterion), which is right only if a higher score of a case never lowers the criterion. They refuse the others, such as `[-1, "trout"]`, with the error `the criterion is not non-decreasing in the case scores`. In a formula made of sums and products by constants only, what matters is the total weight of each case: `{"operation": "sum", "values": [[2, "trout"], [-1, "trout"]]}` is fine. Otherwise every term must be non-decreasing: a minimum with a decreasing term is refused. The bound `all_options` is stricter, and refuses any term that decreases. A heuristic evaluates any formula, unless it is run with `--reformulation path_lp`, which goes through the path relaxation.

## Errors

GECOT checks the instance when it reads it, and stops with a message that starts with `Parsing instance:`.

| Message | Cause |
|---|---|
| `option id '...' appears multiple times` (also for vertices and arcs) | two rows with the same identifier |
| `unknown vertex id '...'`, `unknown arc id '...'`, `unknown option id '...'`, `unknown case '...'` | a row, or the criterion, refers to an identifier that no table defines |
| `'...' is not a valid probability`, `qualities must be non-negative`, `quality gains must be non-negative`, `'...' is not a valid cost` | a value out of range in a CSV file |
| `option '...' must have the cost dimensions of the first option`, `option '...' has named costs, the first option a number` | the costs of an inline table name other dimensions than its first option's |
| `option id '...' appears multiple times in conflict set '...'` | a row of the `conflicts` table is repeated |
| `the budget of '...' is missing`, `the budget of '...' is given twice`, `the budget names '...', which is not a cost dimension`, `the budget item '...' is not name=value`, `the budget of '...' must be a non-negative number`, `the options have several costs: the budget is ...` | the [budget](optimize.md) does not give each [cost dimension](#several-costs) once, as `name=value` |
| `<file> line <n>: Can't find a column named ...` | a column is missing: check `csv_columns` |
| `Cannot open file ...` | the path of a `csv_file` is wrong; remember that it is relative to the JSON file |
| `At /cases/<case>/arcs of [...] - no subschema has succeeded...` | the JSON file does not have the expected structure at this path: a missing or misspelled key, a value out of range in an inline table, an identifier that is not a string, or a case without any improvement table |

## Checking an instance

A quick way to check that an instance says what you mean is to run a heuristic on it: the first lines of the log count what was read,

```console
$ gecot optimize greedy aude.json 4
[   0.0s] GECOT 2.0, algorithm greedy
[   0.0s] Instance: /home/user/instances/Aude/aude.json
[   0.0s] Options count: 15
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s]   species   | vertices | habitats | improvable |  arcs   | improvable
[   0.0s]             |          |          |  vertices  |         |    arcs
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s]       trout |       45 |       34 |          0 |      88 |         30
[   0.0s] ------------+----------+----------+------------+---------+-----------
[   0.0s] Budget 4: 26.67 % of the options' total cost (15)
```

and the initial score of the result is the score of the landscape as it is. The heuristics do not run yet on an instance with [conflict sets or several costs](#conflict-sets): `gecot bound all_options` prints the same lines and the initial score. The directory `test/instances` of the repository holds complete instances: a river network with dams (`Aude`), raster landscapes of several sizes (`Aix`) and a patch-based landscape (`Quebec_438_RASY`).
