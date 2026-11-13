#!/usr/bin/env python3
"""Instance of GECOT that encodes a densest-k-subgraph problem.

Densest-k-subgraph: given a graph G = (V, E) and k, find k vertices that
induce the most edges. It is NP-hard and has no known constant-factor
approximation.

Reduction (vertex improvements only, one vertex per option):
- one vertex per vertex of G, of quality 0, and one option of cost 1 that
  gives it a source quality and a target quality of 1;
- two opposite arcs of probability p per edge of G;
- an isolated vertex "anchor" of quality 1, so that the initial score is 1.

With the options S bought, the score is

    1 + |S| + 2 p e(S) + r(S),      0 <= r(S) <= |S| (|S| - 1) p^2

e(S) being the edges induced by S and r(S) the pairs of S at distance 2 or
more. With p = 1 / (kmax (kmax - 1)), r(S) <= p for every |S| <= kmax, less
than the 2 p of one more edge: the optimal sets of a budget k <= kmax are the
densest k-subgraphs, and

    e(S) = floor((score - 1 - k) / (2 p)).

G is an Erdos-Renyi graph G(n, density). Its edges are written to graph.csv.
"""
import argparse
import json
import os
import random


def generate(n, density, kmax, seed, out):
    rng = random.Random(seed)
    edges = [(i, j) for i in range(n) for j in range(i + 1, n)
             if rng.random() < density]
    p = 1.0 / (kmax * (kmax - 1))
    os.makedirs(out, exist_ok=True)

    def write(name, header, rows):
        with open(os.path.join(out, name), "w") as f:
            f.write(header + "\n")
            f.writelines(row + "\n" for row in rows)

    write("graph.csv", "u,v", (f"{i},{j}" for i, j in edges))
    write("options.csv", "id,cost", (f"{i},1" for i in range(n)))
    write("vertices.csv", "id,source_quality,target_quality",
          ["anchor,1,1"] + [f"{i},0,0" for i in range(n)])
    write("arcs.csv", "id,from,to,probability",
          (f"{a}_{b},{a},{b},{p!r}" for i, j in edges
           for a, b in ((i, j), (j, i))))
    write("vertices_improvements.csv",
          "vertex_id,option_id,source_quality_gain,target_quality_gain",
          (f"{i},{i},1,1" for i in range(n)))
    with open(os.path.join(out, "instance.json"), "w") as f:
        json.dump({
            "options": {"csv_file": "options.csv"},
            "cases": {"dks": {
                "vertices": {"csv_file": "vertices.csv"},
                "arcs": {"csv_file": "arcs.csv"},
                "vertices_improvements": {
                    "csv_file": "vertices_improvements.csv"},
            }},
            "criterion": "dks",
        }, f, indent=4)
        f.write("\n")
    return len(edges), p


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("out", help="directory of the instance")
    parser.add_argument("-n", type=int, required=True, help="vertices of G")
    parser.add_argument("--density", type=float, default=0.5,
                        help="probability of each edge of G")
    parser.add_argument("--kmax", type=int, required=True,
                        help="largest budget the reduction is exact for")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    m, p = generate(args.n, args.density, args.kmax, args.seed, args.out)
    print(f"{args.out}: {args.n} vertices, {m} edges, p = {p:.6g}, "
          f"exact for budgets up to {args.kmax}")
