#!/usr/bin/env python3
"""Brute-force optimum of a gecot instance, with cost dimensions and conflict sets.

usage: brute.py instance.json BUDGET            (BUDGET: '5' or 'money=5,area=3')
       brute.py instance.json BUDGET --check out.json   (check a gecot JSON output)
       brute.py instance.json BUDGET --score a,b,c       (score of a selection)
"""
import csv, itertools, json, math, os, sys
import numpy as np


def read_table(spec, base, defaults):
    if isinstance(spec, list):
        return [dict(r) for r in spec], {k: k for k in defaults}
    path = os.path.join(base, spec["csv_file"])
    cols = {k: k for k in defaults}
    cols.update(spec.get("csv_columns", {}))
    with open(path) as f:
        return list(csv.DictReader(f)), cols


def load(path):
    base = os.path.dirname(os.path.abspath(path))
    j = json.load(open(path))
    rows, cols = read_table(j["options"], base, ["id", "cost"])
    options, costs = [], []
    cost_col = cols["cost"]
    if isinstance(j["options"], list):
        first = rows[0]["cost"]
        dims = sorted(first) if isinstance(first, dict) else ["cost"]
        for r in rows:
            options.append(str(r["id"]))
            c = r["cost"]
            costs.append([float(c[d]) for d in dims] if isinstance(c, dict) else [float(c)])
    else:
        dims = sorted(cost_col) if isinstance(cost_col, dict) else ["cost"]
        for r in rows:
            options.append(r[cols["id"]])
            if isinstance(cost_col, dict):
                costs.append([float(r[cost_col[d]]) for d in dims])
            else:
                costs.append([float(r[cost_col])])
    oid = {o: i for i, o in enumerate(options)}
    sets = {}
    if "conflicts" in j:
        crow, ccols = read_table(j["conflicts"], base, ["set_id", "option_id"])
        for r in crow:
            sets.setdefault(r[ccols["set_id"]], []).append(oid[r[ccols["option_id"]]])
    cases = {}
    for name, c in j["cases"].items():
        vrows, vc = read_table(c["vertices"], base, ["id", "source_quality", "target_quality"])
        vid = {}
        w, q = [], []
        for r in vrows:
            vid[str(r[vc["id"]])] = len(vid)
            w.append(float(r[vc["source_quality"]]))
            q.append(float(r[vc["target_quality"]]))
        arows, ac = read_table(c["arcs"], base, ["id", "from", "to", "probability"])
        arcs, aid = [], {}
        for r in arows:
            aid[str(r[ac["id"]])] = len(arcs)
            arcs.append((vid[str(r[ac["from"]])], vid[str(r[ac["to"]])], float(r[ac["probability"]])))
        vimp, aimp = [], []
        if "vertices_improvements" in c:
            rr, ic = read_table(c["vertices_improvements"], base,
                                ["vertex_id", "option_id", "source_quality_gain", "target_quality_gain"])
            for r in rr:
                vimp.append((vid[str(r[ic["vertex_id"]])], oid[str(r[ic["option_id"]])],
                             float(r[ic["source_quality_gain"]]), float(r[ic["target_quality_gain"]])))
        if "arcs_improvements" in c:
            rr, ic = read_table(c["arcs_improvements"], base, ["arc_id", "option_id", "improved_probability"])
            for r in rr:
                aimp.append((aid[str(r[ic["arc_id"]])], oid[str(r[ic["option_id"]])],
                             float(r[ic["improved_probability"]])))
        cases[name] = (np.array(w), np.array(q), arcs, vimp, aimp)
    crit = j.get("criterion", {"operation": "sum", "values": list(cases)})
    return dict(dims=dims, options=options, costs=np.array(costs), sets=sets, cases=cases, crit=crit)


def case_score(case, X):
    w, q, arcs, vimp, aimp = case
    w, q = w.copy(), q.copy()
    for v, o, sg, tg in vimp:
        if X[o]:
            w[v] += sg
            q[v] += tg
    n = len(w)
    P = np.zeros((n, n))
    np.fill_diagonal(P, 1.0)
    best = {}
    for a, (u, v, p) in enumerate(arcs):
        best[a] = p
    for a, o, p in aimp:
        if X[o]:
            best[a] = max(best[a], p)
    for a, (u, v, p) in enumerate(arcs):
        P[u, v] = max(P[u, v], best[a])
    for k in range(n):  # max-product closure
        P = np.maximum(P, np.outer(P[:, k], P[k, :]))
    return float(w @ P @ q)


def eval_crit(f, scores):
    if isinstance(f, str):
        return scores[f]
    if isinstance(f, (int, float)):
        return float(f)
    if isinstance(f, list):
        return f[0] * eval_crit(f[1], scores)
    vals = [eval_crit(g, scores) for g in f["values"]]
    return {"sum": sum, "min": min, "product": math.prod}[f["operation"]](vals)


def score(inst, X):
    return eval_crit(inst["crit"], {n: case_score(c, X) for n, c in inst["cases"].items()})


def parse_budget(inst, text):
    if "=" not in text:
        assert len(inst["dims"]) == 1
        return np.array([float(text)])
    d = dict(item.split("=") for item in text.split(","))
    return np.array([float(d[k]) for k in inst["dims"]])


def feasible(inst, X, B, tol=1e-6):
    c = inst["costs"].T @ np.array(X, dtype=float)
    if np.any(c > B + tol * np.maximum(1, B)):
        return False
    return all(sum(X[o] for o in s) <= 1 for s in inst["sets"].values())


def optimum(inst, B):
    n = len(inst["options"])
    best = (-1, None)
    for mask in range(1 << n):
        X = [(mask >> i) & 1 for i in range(n)]
        if not feasible(inst, X, B, 0.0):
            continue
        s = score(inst, X)
        if s > best[0]:
            best = (s, [inst["options"][i] for i in range(n) if X[i]])
    return best


if __name__ == "__main__":
    inst = load(sys.argv[1])
    B = parse_budget(inst, sys.argv[2])
    if len(sys.argv) > 3 and sys.argv[3] == "--score":
        names = [x for x in sys.argv[4].split(",") if x]
        X = [int(o in names) for o in inst["options"]]
        print(f"{score(inst, X):.10g} feasible={feasible(inst, X, B)}")
    elif len(sys.argv) > 3 and sys.argv[3] == "--check":
        out = json.load(open(sys.argv[4]))
        X = [int(out["solution"][o]) for o in inst["options"]]
        s = score(inst, X)
        print(f"score {s:.10g} reported {out['solution_score']:.10g} feasible={feasible(inst, X, B)}")
    else:
        s, sel = optimum(inst, B)
        print(f"{s:.10g} {','.join(sel)}")
