#!/usr/bin/env python3
"""Checks gecot's algorithms against brute force on small instances with conflict
sets and several cost dimensions, run by hand like test/cli_parity.sh.

usage: check.py GECOT_BIN [ALGORITHM_CONFIG ...]

Each config is an algorithm and its parameters in one argument, as
"flow_mip --big-m plain"; by default every supported algorithm with its main
options. A run passes if its selection is feasible, its reported score is the
brute-force score of that selection, its bound, if any, is at least the
brute-force optimum, and, for an exact algorithm, its status is optimal and
its score the optimum. The instances are variants of the repository's
three woods, Aude and conflicts instances, written to a temporary directory,
and include three oracles that bypass the new rows: a second dimension equal
to the first, an all-zero one, and renamed dimensions, each with the same
optimum as its plain twin. Needs numpy.
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
    "multicut_benders", "multicut_benders --big-m tight", "multicut_benders --groups 0",
    "flow_benders", "flow_benders --big-m knapsack_dantzig", "flow_benders --nb-mus 3",
    "multiflow_benders", "multiflow_benders --no-root-cuts", "multiflow_benders --no-in-out",
    "path_lp", "path_lp --pricing enumerate", "flow_mip --reformulation path_lp",
    "multicut_benders --reformulation path_lp --reformulation-params pricing=enumerate",
    "static", "greedy", "greedy --worst",
]


def relaxed(config):
    """Not exact: the relaxations and the heuristics."""
    words = config.split()
    return words[0] in ("static", "greedy", "path_lp") or "--lp-relaxation" in words


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


def run(binary, path, budget, config, out):
    algorithm, *params = config.split()
    result = os.path.join(out, "result.json")
    if os.path.exists(result):
        os.remove(result)
    p = subprocess.run([binary, "optimize", algorithm, path, budget, *params, "-q", "-o", result],
                       capture_output=True, text=True, timeout=600)
    if p.returncode != 0:
        return None, p.stderr.strip()
    return json.load(open(result)), ""


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    binary = os.path.abspath(sys.argv[1])
    configs = sys.argv[2:] or CONFIGS
    failures, runs = [], 0
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
                    if result is None:
                        failures.append(f"{os.path.basename(path)} {b} {config}: {error[:200]}")
                        continue
                    X = [int(result["solution"][o]) for o in instance["options"]]
                    score = brute.score(instance, X)
                    status = result["algorithm_report"]["status"]
                    bound = result["algorithm_report"].get("best_bound", float("inf"))
                    ok = brute.feasible(instance, X, B) and result.get("feasible", True) \
                        and abs(score - result["solution_score"]) <= 1e-6 * max(1, score) \
                        and bound >= optimum - 1e-6 * max(1, optimum)
                    if not relaxed(config):
                        ok = ok and status in ("optimal", "no option left") \
                            and abs(score - optimum) <= 1e-6 * max(1, optimum)
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
                    ok = abs(s1 - s2) <= 1e-6 * max(1, s2)
                    print(f"  oracle {os.path.basename(path)} {b} {config}: {s1:.10g} | {s2:.10g} "
                          f"{'ok' if ok else 'FAIL'}")
                    if not ok:
                        failures.append(f"oracle {os.path.basename(path)} {b} {config}: {s1} != {s2}")
    print(f"{runs} runs, {len(failures)} failures")
    for f in failures:
        print("FAIL", f)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
