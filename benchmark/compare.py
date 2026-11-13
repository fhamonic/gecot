#!/usr/bin/env python3
"""Compares result files of run.py, as Markdown tables.

    compare.py results/flow_mip.csv results/flow_mip_big-m_tight.csv

Per run, the best known score is the best of the files given and of
best_known.csv, and the best known bound the lowest one. Gains are over the
initial score of the instance:
- solved: the algorithm proved its solution optimal;
- reached: its gain as a share of the best known gain;
- guaranteed: its gain as a share of the largest gain its own bound allows.
The times of the summary are shifted geometric means (shift of 1 s): the time
of a run that ended by itself (solved, or a heuristic), the time limit of its
result file otherwise.

A score above another file's bound by more than 1e-6 (relative) is reported:
one of the two is wrong.

--save-best writes the best known scores and bounds to best_known.csv.
"""
import argparse
import csv
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
BEST = os.path.join(HERE, "best_known.csv")


def number(text):
    return float(text) if text not in ("", None) else None


def load(path):
    with open(path) as f:
        runs = {(r["instance"], r["budget"]): r for r in csv.DictReader(f)}
    meta_path = os.path.join(os.path.splitext(path)[0], "meta.json")
    limit = None
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            limit = json.load(f)["time_limit"]
    return runs, limit


def share(gain, of):
    if gain is None or of is None:
        return None
    return 1.0 if of <= 0 else max(0.0, gain / of)


def percent(value):
    return "-" if value is None else f"{100 * value:.2f} %"


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("results", nargs="+", help="CSV files of run.py")
    parser.add_argument("--save-best", action="store_true",
                        help="update best_known.csv with these results")
    args = parser.parse_args()

    labels = [os.path.splitext(os.path.basename(p))[0] for p in args.results]
    files = [load(p) for p in args.results]
    keys = []
    for runs, _ in files:
        keys += [k for k in runs if k not in keys]
    instances = list(dict.fromkeys(k[0] for k in keys))
    keys.sort(key=lambda k: (instances.index(k[0]), float(k[1])))

    best = {}  # key -> [score, bound, initial]
    if os.path.exists(BEST):
        with open(BEST) as f:
            for r in csv.DictReader(f):
                best[r["instance"], r["budget"]] = [
                    number(r["score"]), number(r["bound"]), None]
    issues = []
    for label, (runs, _) in zip(labels, files):
        for key, r in runs.items():
            entry = best.setdefault(key, [None, None, None])
            score, bound = number(r.get("score")), number(r.get("bound"))
            if score is not None and (entry[0] is None or score > entry[0]):
                entry[0] = score
            if bound is not None and (entry[1] is None or bound < entry[1]):
                entry[1] = bound
            if number(r.get("initial_score")) is not None:
                entry[2] = number(r["initial_score"])
    for key, (score, bound, _) in best.items():
        if score is not None and bound is not None and \
                score > bound + 1e-6 * abs(bound):
            issues.append(f"{key[0]} {key[1]}: best score {score!r} above "
                          f"the lowest bound {bound!r}")

    print("| instance | budget | best gain | " +
          " | ".join(labels) + " |")
    print("|---|---|---|" + "---|" * len(labels))
    stats = [{"runs": 0, "solved": 0, "log_time": 0.0, "reached": [],
              "guaranteed": [], "best": 0} for _ in labels]
    for key in keys:
        best_score, best_bound, initial = best[key]
        cells = []
        for (runs, limit), stat in zip(files, stats):
            r = runs.get(key)
            if r is None:
                cells.append("")
                continue
            stat["runs"] += 1
            score, bound = number(r.get("score")), number(r.get("bound"))
            wall = float(r["wall_time"])
            if score is None:
                cells.append(f"{r['status']} ({r['exit']})")
                stat["log_time"] += math.log((limit or wall) + 1)
                stat["reached"].append(0.0)
                continue
            start = number(r["initial_score"])
            reached = share(score - start, best_score - start)
            guaranteed = share(score - start,
                               None if bound is None else bound - start)
            stat["reached"].append(reached)
            if reached >= 1 - 1e-9:
                stat["best"] += 1
            if guaranteed is not None:
                stat["guaranteed"].append(guaranteed)
            if r["status"] == "optimal":
                stat["solved"] += 1
                stat["log_time"] += math.log(wall + 1)
                cells.append(f"{wall:.1f} s")
            elif r["status"] == "completed":  # a heuristic, at its end
                stat["log_time"] += math.log(wall + 1)
                cells.append(f"{percent(reached)} reached in {wall:.1f} s")
            else:
                stat["log_time"] += math.log((limit or wall) + 1)
                cells.append(f"{percent(reached)} reached, " + (
                    f"{percent(guaranteed)} guaranteed"
                    if guaranteed is not None else "no bound"))
        gain = "-" if best_score is None or not initial else \
            f"+{100 * (best_score / initial - 1):.2f} %"
        print(f"| {key[0]} | {key[1]} | {gain} | " + " | ".join(cells) + " |")

    print()
    print("| | runs | solved | best score | time | mean reached | "
          "mean guaranteed |")
    print("|---|---|---|---|---|---|---|")
    for label, stat in zip(labels, stats):
        n = max(stat["runs"], 1)
        mean = lambda v: percent(sum(v) / len(v)) if v else "-"
        print(f"| {label} | {stat['runs']} | {stat['solved']} | "
              f"{stat['best']} | {math.exp(stat['log_time'] / n) - 1:.1f} s "
              f"| {mean(stat['reached'])} | {mean(stat['guaranteed'])} |")
    if issues:
        print("\nInconsistent results:")
        for issue in issues:
            print("- " + issue)

    if args.save_best:
        with open(BEST, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["instance", "budget", "score", "bound"])
            for key in sorted(best, key=lambda k: (k[0], float(k[1]))):
                score, bound, _ = best[key]
                writer.writerow([key[0], key[1],
                                 "" if score is None else repr(score),
                                 "" if bound is None else repr(bound)])


if __name__ == "__main__":
    main()
