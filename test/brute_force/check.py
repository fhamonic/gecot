#!/usr/bin/env python3
"""Checks gecot's algorithms against brute force on small instances with conflict
sets and several cost dimensions, run by hand like test/parity/capture.py.

usage: check.py GECOT_BIN [ALGORITHM_CONFIG ...]

Each config is an algorithm and its parameters in one argument, as
"flow_mip --big-m plain", or "bound" and a method of gecot bound, as
"bound qkp_planes --iterations 20"; by default every supported algorithm and
method with their main options. A run passes if its selection is feasible, its
reported score is the brute-force score of that selection, its bound, if any,
is at least the brute-force optimum, and, for an exact algorithm, its status
is optimal and its score the optimum. A bound passes if it is at least the
optimum, each up to 1e-6 of it; split_lagrange refusing the shape of the options is no failure. The instances are variants of the repository's
three woods, Aude, conflicts and two species instances, written to a temporary
directory. The two species come with criteria that gecot simplifies, leaving a
case out, and with weights of 1e-8 to 1e12 on copies of the squirrel whose
scores differ as much, which the scale of the criterion must serve. Three
oracles bypass the new rows: a second dimension equal to the first, an
all-zero one, and renamed dimensions, each with the same optimum as its plain
twin. Needs numpy.
"""
import copy, csv, json, os, random, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True  # no __pycache__ in the repository
import brute  # noqa: E402

CONFIGS = [
    "flow_mip", "flow_mip --big-m knapsack_dantzig", "flow_mip --big-m plain",
    "flow_mip --big-m tight", "flow_mip --nb-mus 3", "flow_mip --preprocessing none",
    "flow_mip --solver highs", "flow_mip --lp-relaxation",
    "flow_mip --indicator-constraints",
    "multicut_benders", "multicut_benders --big-m tight", "multicut_benders --groups 0",
    "flow_benders", "flow_benders --big-m knapsack_dantzig", "flow_benders --nb-mus 3",
    "flow_benders --indicator-constraints",
    "multiflow_benders", "multiflow_benders --no-root-cuts", "multiflow_benders --no-in-out",
    "path_lp", "path_lp --pricing enumerate", "flow_mip --reformulation path_lp",
    "multicut_benders --reformulation path_lp --reformulation-params pricing=enumerate",
    "static", "greedy", "greedy --worst",
    "bound all_options", "bound qkp_planes", "bound qkp_planes --iterations 20",
    "bound split_lagrange", "bound path_lp",
]


def relaxed(config):
    """Not exact: the relaxations, the heuristics and the bounds."""
    words = config.split()
    return words[0] in ("static", "greedy", "path_lp", "bound") or "--lp-relaxation" in words


def write_instances(out):
    """The instances and budgets of the check, as (path, budgets)."""
    tw = json.load(open(f"{REPO}/docs/examples/three-woods.json"))
    c = copy.deepcopy(tw)
    c["options"][2]["cost"] = 0  # a free plantation in conflict with the underpass
    c["conflicts"] = [{"set_id": "site", "option_id": o} for o in ("plantation", "underpass")]
    json.dump(c, open(f"{out}/tw_conflict.json", "w"))
    d = copy.deepcopy(tw)
    for o, cost in zip(d["options"], ({"money": 2, "area": 1}, {"money": 3, "area": 0},
                                      {"money": 2, "area": 4})):
        o["cost"] = cost
    json.dump(d, open(f"{out}/tw_dims.json", "w"))
    cases = [(f"{out}/tw_conflict.json", ["0", "2", "3", "5", "7"]),
             (f"{out}/tw_dims.json", ["money=4,area=5", "money=7,area=4", "money=2,area=0"]),
             (f"{REPO}/test/instances/conflicts/instance.json",
              ["money=4,area=5", "money=5,area=1", "money=7,area=9"])]
    for name in ("three-woods-alternatives", "three-woods-works"):
        cases.append((f"{REPO}/docs/examples/{name}.json", ["3", "5", "7"]))
    cases.append((f"{REPO}/docs/examples/three-woods-resources.json",
                  ["money=7,area=2,crossings=1", "money=7,area=3,crossings=0"]))
    # Aude with overlapping sets, a dam free in money and in a set, one free and in none
    A = f"{REPO}/test/instances/Aude/"
    aude = json.load(open(A + "aude.json"))
    for case in aude["cases"].values():
        for table in case.values():
            if isinstance(table, dict) and "csv_file" in table:
                table["csv_file"] = A + table["csv_file"]
    dams = [r["id"] for r in csv.DictReader(open(A + "dams.csv"))]
    rng = random.Random(3)
    area = [rng.randint(0, 3) for _ in dams]
    sets = [[0, 3, 5], [5, 8], [2, 9, 11], [1, 2]]
    money = lambda i: 0 if i in (6, 8) else 1

    def aude_variant(name, cost):
        j = copy.deepcopy(aude)
        j["options"] = [{"id": d, "cost": cost(i)} for i, d in enumerate(dams)]
        j["conflicts"] = [{"set_id": f"s{k}", "option_id": dams[i]}
                          for k, s in enumerate(sets) for i in s]
        json.dump(j, open(f"{out}/{name}.json", "w"))
        return f"{out}/{name}.json"
    cases.append((aude_variant("aude_cd", lambda i: {"money": money(i), "area": 0 if i == 6 else area[i]}),
                  ["money=2,area=2", "money=5,area=4", "money=8,area=6"]))
    cases.append((aude_variant("aude_c1", money), ["2", "3", "5"]))
    cases.append((star(out), ["money=3,area=2", "money=5,area=3", "money=8,area=1"]))
    # criteria that gecot simplifies to one that does not read the hedgehog, its first case:
    # the terms of a sum that add up to the squirrel, then such a sum inside a minimum
    species = json.load(open(f"{REPO}/docs/examples/two-species.json"))
    term = lambda *values: {"operation": "sum", "values": list(values)}
    for name, criterion in (
            ("species_sum", term([2, "squirrel"], [-1, "squirrel"], [0, "hedgehog"])),
            ("species_min", {"operation": "min", "values": [
                term("squirrel", "hedgehog", [-1, "hedgehog"]), [3, "squirrel"]]})):
        j = copy.deepcopy(species)
        j["criterion"] = criterion
        json.dump(j, open(f"{out}/{name}.json", "w"))
        cases.append((f"{out}/{name}.json", ["3", "5", "7"]))
    # a second squirrel whose scores are 1e8 and 1e10 times larger, weighted by 1e-8 and 1e-10:
    # without the scale of the criterion, bounds under the optimum and a wrong optimum
    # and both 1e5 times larger, under a minimum held by its term of weight 1e-10
    def scaled(case, factor):
        case = copy.deepcopy(case)
        for v in case["vertices"]:
            v["source_quality"] *= factor
            v["target_quality"] *= factor
        for g in case.get("vertices_improvements", []):
            g["source_quality_gain"] *= factor
            g["target_quality_gain"] *= factor
        return case
    min_of = lambda *values: {"operation": "min", "values": list(values)}
    for name, factors, criterion in (
            ("weighted_sum", (1, 1e4), term("squirrel", [1e-8, "copy"])),
            ("weighted_min", (1, 1e5), min_of("squirrel", [1e-10, "copy"])),
            ("weighted_alike", (1e5, 1e5), min_of("squirrel", [1e-10, "copy"]))):
        j = copy.deepcopy(species)
        del j["cases"]["hedgehog"]
        twin = scaled(j["cases"]["squirrel"], factors[1])
        j["cases"]["squirrel"] = scaled(j["cases"]["squirrel"], factors[0])
        j["cases"]["copy"] = twin
        j["criterion"] = criterion
        json.dump(j, open(f"{out}/{name}.json", "w"))
        cases.append((f"{out}/{name}.json", ["2", "4", "5", "7"]))
    # the squirrel with scores near 1e-9, which the qualities are scaled up for; and a criterion
    # with a product of constants in a product, which the reformulation folds for the MIP solvers
    j = copy.deepcopy(species)
    del j["cases"]["hedgehog"]
    j["cases"]["squirrel"] = scaled(j["cases"]["squirrel"], 1e-5)
    j["criterion"] = "squirrel"
    json.dump(j, open(f"{out}/tiny.json", "w"))
    cases.append((f"{out}/tiny.json", ["3", "5", "7"]))
    # sources and targets apart, linked by arcs of probability 0 that options raise to 0.9, and
    # a vertex of qualities 1e-3: the score goes from 1e-6 to tens, which bounds the scale-up
    vertices = [{"id": f"s{i}", "source_quality": q, "target_quality": 0} for i, q in enumerate((3, 1, 2, 2))]
    vertices += [{"id": f"t{i}", "source_quality": 0, "target_quality": q} for i, q in enumerate((2, 3, 1, 1))]
    vertices.append({"id": "w", "source_quality": 1e-3, "target_quality": 1e-3})
    links = [(i, k) for i in range(4) for k in range(4) if (i + k) % 2 == 0]
    j = {"options": [{"id": f"o{i}{k}", "cost": 1 + (i + 2 * k) % 3} for i, k in links],
         "cases": {"c": {"vertices": vertices,
                         "arcs": [{"id": f"a{i}{k}", "from": f"s{i}", "to": f"t{k}", "probability": 0}
                                  for i, k in links],
                         "arcs_improvements": [{"arc_id": f"a{i}{k}", "option_id": f"o{i}{k}",
                                                "improved_probability": 0.9} for i, k in links]}},
         "criterion": "c"}
    json.dump(j, open(f"{out}/rising.json", "w"))
    cases.append((f"{out}/rising.json", ["3", "5", "8"]))
    j = copy.deepcopy(species)
    j["criterion"] = min_of("hedgehog", {"operation": "product", "values": [
        {"operation": "product", "values": [2, 3]}, "squirrel"]})
    json.dump(j, open(f"{out}/folded.json", "w"))
    cases.append((f"{out}/folded.json", ["3", "5", "7"]))
    # a case weighted by 1e12, at 0 unless an option over every budget is bought
    j = copy.deepcopy(species)
    del j["cases"]["hedgehog"]
    j["options"].append({"id": "bridge", "cost": 100})
    j["cases"]["ghost"] = {
        "vertices": [{"id": "a", "source_quality": 6, "target_quality": 0},
                     {"id": "b", "source_quality": 0, "target_quality": 6}],
        "arcs": [{"id": "a_b", "from": "a", "to": "b", "probability": 0}],
        "arcs_improvements": [{"arc_id": "a_b", "option_id": "bridge", "improved_probability": 1}]}
    j["criterion"] = term("squirrel", [1e12, "ghost"])
    json.dump(j, open(f"{out}/weighted_out_of_reach.json", "w"))
    cases.append((f"{out}/weighted_out_of_reach.json", ["3", "5", "7"]))
    # (instance, its budget, twin, the twin's budget, budget values)
    same = lambda b: b
    oracles = [(aude_variant("aude_dup", lambda i: {"money": money(i), "copy": money(i)}),
                lambda b: f"money={b},copy={b}", f"{out}/aude_c1.json", same, ["2", "3", "5"]),
               (aude_variant("aude_zero", lambda i: {"money": money(i), "zero": 0}),
                lambda b: f"money={b},zero=0", f"{out}/aude_c1.json", same, ["2", "3", "5"]),
               (aude_variant("aude_ren", lambda i: {"zz": money(i), "aa": 0 if i == 6 else area[i]}),
                lambda b: "zz={},aa={}".format(*b), f"{out}/aude_cd.json",
                lambda b: "money={},area={}".format(*b), [(2, 2), (5, 4), (8, 6)])]
    return cases, oracles


def star(out):
    """An instance of the shape split_lagrange prices: each option improves the
    arcs entering one vertex, with two costs and overlapping sets."""
    rng = random.Random(5)
    names = [f"v{i}" for i in range(7)]
    vertices = [{"id": v, "source_quality": rng.randint(1, 3), "target_quality": rng.randint(1, 3)}
                for v in names]
    edges = [(i, (i + 1) % 7) for i in range(7)] + [(0, 3), (2, 5), (4, 6)]
    arcs = [{"id": f"{names[u]}_{names[v]}", "from": names[u], "to": names[v],
             "probability": round(rng.uniform(0.2, 0.6), 2)}
            for a, b in edges for u, v in ((a, b), (b, a))]
    options = [{"id": f"enter_{v}", "cost": {"money": rng.randint(1, 3), "area": rng.randint(0, 2)}}
               for v in names[1:]]
    options += [{"id": f"restore_{v}", "cost": {"money": rng.randint(1, 2), "area": rng.randint(0, 1)}}
                for v in ("v0", "v3")]
    improvements = [{"arc_id": a["id"], "option_id": f"enter_{a['to']}",
                     "improved_probability": min(1.0, a["probability"] + 0.35)}
                    for a in arcs if a["to"] != "v0"]
    gains = [{"vertex_id": v, "option_id": f"restore_{v}", "source_quality_gain": 2,
              "target_quality_gain": 1} for v in ("v0", "v3")]
    gains.append({"vertex_id": "v2", "option_id": "enter_v2", "source_quality_gain": 1,
                  "target_quality_gain": 1})
    sets = {"a": ["enter_v1", "enter_v4"], "b": ["enter_v2", "restore_v3", "enter_v5"],
            "c": ["enter_v5", "enter_v6"], "d": ["restore_v0", "enter_v6"]}
    j = {"options": options,
         "conflicts": [{"set_id": k, "option_id": o} for k, l in sets.items() for o in l],
         "cases": {"star": {"vertices": vertices, "arcs": arcs, "vertices_improvements": gains,
                            "arcs_improvements": improvements}},
         "criterion": "star"}
    json.dump(j, open(f"{out}/star.json", "w"))
    return f"{out}/star.json"


# split_lagrange refusing the shape of the options
SHAPE = ("is improved by options", "is entered by arcs improved by", "share no vertex")


def run(binary, path, budget, config, out):
    words = config.split()
    bound = words[0] == "bound"
    command, params = (words[:2], words[2:]) if bound else (["optimize", words[0]], words[1:])
    result = os.path.join(out, "result.json")
    if os.path.exists(result):
        os.remove(result)
    p = subprocess.run([binary, *command, path, budget, *params, "-q", "-o", result],
                       capture_output=True, text=True, timeout=600)
    if p.returncode != 0:
        return None, p.stderr.strip()
    return json.load(open(result)), ""


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    binary = os.path.abspath(sys.argv[1])
    configs = sys.argv[2:] or CONFIGS
    failures, runs, refused = [], 0, 0
    with tempfile.TemporaryDirectory() as out:
        cases, oracles = write_instances(out)
        for path, budgets in cases:
            instance = brute.load(path)
            for b in budgets:
                B = brute.parse_budget(instance, b)
                optimum, selection = brute.optimum(instance, B)
                print(f"{os.path.basename(path)} {b}: optimum {optimum:.10g} {selection}")
                for config in configs:
                    runs += 1
                    result, error = run(binary, path, b, config, out)
                    if config.startswith("bound "):
                        if result is None and any(m in error for m in SHAPE):
                            refused += 1
                            print(f"  {config:42s} refused: {error[:80]}")
                            continue
                        ok = result is not None \
                            and result["bound"] >= optimum - 1e-6 * abs(optimum)
                        print(f"  {config:42s} {result['bound'] if result else error[:80]} "
                              f"{'ok' if ok else 'FAIL'}")
                        if not ok:
                            failures.append(f"{os.path.basename(path)} {b} {config}: "
                                            f"{result['bound'] if result else error[:200]}, "
                                            f"optimum {optimum}")
                        continue
                    if result is None:
                        failures.append(f"{os.path.basename(path)} {b} {config}: {error[:200]}")
                        continue
                    X = [int(result["solution"][o]) for o in instance["options"]]
                    score = brute.score(instance, X)
                    status = result["algorithm_report"]["status"]
                    bound = result["algorithm_report"].get("best_bound", float("inf"))
                    ok = brute.feasible(instance, X, B) and result.get("feasible", True) \
                        and abs(score - result["solution_score"]) <= 1e-6 * abs(score) \
                        and bound >= optimum - 1e-6 * abs(optimum)
                    if not relaxed(config):
                        ok = ok and status in ("optimal", "no option left") \
                            and abs(score - optimum) <= 1e-6 * abs(optimum)
                    print(f"  {config:42s} {status:15s} {score:.10g} {'ok' if ok else 'FAIL'}")
                    if not ok:
                        failures.append(f"{os.path.basename(path)} {b} {config}: score {score}, "
                                        f"optimum {optimum}, status {status}")
        for path, budget_of, twin, twin_budget_of, budgets in oracles:
            for b in budgets:
                for config in configs:
                    if relaxed(config):
                        continue
                    runs += 1
                    first, e1 = run(binary, path, budget_of(b), config, out)
                    second, e2 = run(binary, twin, twin_budget_of(b), config, out)
                    if first is None or second is None:
                        failures.append(f"oracle {os.path.basename(path)} {b} {config}: {e1 or e2}")
                        continue
                    s1, s2 = first["solution_score"], second["solution_score"]
                    ok = abs(s1 - s2) <= 1e-6 * abs(s2)
                    print(f"  oracle {os.path.basename(path)} {b} {config}: {s1:.10g} | {s2:.10g} "
                          f"{'ok' if ok else 'FAIL'}")
                    if not ok:
                        failures.append(f"oracle {os.path.basename(path)} {b} {config}: {s1} != {s2}")
    print(f"{runs} runs, {len(failures)} failures, {refused} bounds refused by the shape check")
    for f in failures:
        print("FAIL", f)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
