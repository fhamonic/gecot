#!/usr/bin/env python3
"""Instance of GECOT that encodes a maximum coverage problem.

Maximum coverage: given m subsets of a ground set of n elements and k, find k
subsets whose union is the largest. It is NP-hard, and NP-hard to approximate
within a factor better than 1 - 1/e.

Reduction (arc improvements only, one arc per option):
- one vertex per element, of source quality 1;
- one vertex per subset, of quality 0, with an arc of probability 1 from each
  of its elements;
- a vertex "hub" of source and target quality 1, with an arc of probability 0
  from each subset, and one option of cost 1 per subset that raises the
  probability of this arc to 1.

With the options S bought, the score is 1 + the number of elements that the
subsets of S cover: the optimal sets of a budget k are maximum coverages.

Each element is in `degree` subsets drawn uniformly at random. With degree 2
the elements are the edges of a random multigraph on the subsets, and the
problem is maximum k-vertex-cover. The subsets are written to sets.csv.

With --groups g > 1 the instance encodes fair maximum coverage: the elements
are split into g groups (element e in group e mod g), and the k subsets must
maximise the smallest number of covered elements over the groups. It has one
case per group, built as above with the elements of the group, the options
being shared, and its criterion is the minimum of the cases: the score is
1 + the coverage of the worst group. Each subset takes its elements from
`span` groups only, every choice of `span` groups as often as the others, so
that no subset alone raises the score when span < g.
"""
import argparse
import itertools
import json
import os
import random


def generate(m, n, degree, seed, out, groups=1, span=1):
    rng = random.Random(seed)
    choices = list(itertools.combinations(range(groups), span))
    devoted = [choices[j % len(choices)] for j in range(m)]
    memberships = [sorted(rng.sample(
        [j for j in range(m) if e % groups in devoted[j]], degree))
        for e in range(n)]
    os.makedirs(out, exist_ok=True)

    def write(name, header, rows):
        with open(os.path.join(out, name), "w") as f:
            f.write(header + "\n")
            f.writelines(row + "\n" for row in rows)

    write("sets.csv", "element,subset",
          (f"{e},{j}" for e, sets in enumerate(memberships) for j in sets))
    write("options.csv", "id,cost", (f"s{j},1" for j in range(m)))
    cases = {}
    for g in range(groups):
        name, prefix = ("coverage", "") if groups == 1 else (f"g{g}", f"g{g}_")
        subsets = [j for j in range(m) if g in devoted[j]]
        elements = range(g, n, groups)
        write(prefix + "vertices.csv", "id,source_quality,target_quality",
              ["hub,1,1"] + [f"s{j},0,0" for j in subsets] +
              [f"e{e},1,0" for e in elements])
        write(prefix + "arcs.csv", "id,from,to,probability",
              [f"s{j}_hub,s{j},hub,0" for j in subsets] +
              [f"e{e}_s{j},e{e},s{j},1" for e in elements
               for j in memberships[e]])
        write(prefix + "arcs_improvements.csv",
              "arc_id,option_id,improved_probability",
              (f"s{j}_hub,s{j},1" for j in subsets))
        cases[name] = {
            "vertices": {"csv_file": prefix + "vertices.csv"},
            "arcs": {"csv_file": prefix + "arcs.csv"},
            "arcs_improvements": {"csv_file": prefix + "arcs_improvements.csv"},
        }
    with open(os.path.join(out, "instance.json"), "w") as f:
        json.dump({
            "options": {"csv_file": "options.csv"},
            "cases": cases,
            "criterion": "coverage" if groups == 1 else
            {"operation": "min", "values": list(cases)},
        }, f, indent=4)
        f.write("\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("out", help="directory of the instance")
    parser.add_argument("-m", type=int, required=True, help="subsets")
    parser.add_argument("-n", type=int, required=True, help="elements")
    parser.add_argument("--degree", type=int, default=2,
                        help="subsets that contain each element")
    parser.add_argument("--groups", type=int, default=1,
                        help="groups of elements, one case each (default 1)")
    parser.add_argument("--span", type=int,
                        help="groups a subset takes its elements from "
                        "(default: all)")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    generate(args.m, args.n, args.degree, args.seed, args.out, args.groups,
             args.span or args.groups)
    print(f"{args.out}: {args.m} subsets, {args.n} elements, each in "
          f"{args.degree} subsets" + (f", {args.groups} groups"
                                      if args.groups > 1 else ""))
