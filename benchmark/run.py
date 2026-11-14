#!/usr/bin/env python3
"""Runs one algorithm of GECOT, with fixed parameters, over a suite of the
benchmark, one run at a time.

    run.py flow_mip                          the full suite, defaults
    run.py --suite short flow_mip --big-m tight
    run.py --suite short --label tight flow_mip --big-m tight

The parameters after the algorithm are passed to it as they are. A run is an
instance and a budget of the suite; its result is appended to
results/<label>.csv as soon as it ends, and its solution and its log are kept
in results/<label>/. Running the same command again resumes: the runs already
in the CSV file are skipped (--force runs them again).

The time limit of the suite is given to the algorithm when it has a
--time-limit parameter. A run still going 5 % + 10 s after the limit receives
Ctrl-C, which makes GECOT return its best solution so far, and is killed 30 s
later: no run lasts more than 1.05 x limit + 40 s.
"""
import argparse
import csv
import json
import os
import shlex
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# suite: file and time limit of a run, in seconds
SUITES = {"full": ("suites/full.csv", 900), "short": ("suites/short.csv", 120)}
FIELDS = ["instance", "budget", "status", "wall_time", "time", "initial_score",
          "score", "bound", "cost", "nodes", "exit", "feasible"]


def read_suite(path):
    with open(path) as f:
        rows = csv.DictReader(line for line in f if not line.startswith("#"))
        return [(row["instance"], row["budget"]) for row in rows]


def has_time_limit(gecot, algorithm):
    out = subprocess.run([gecot, "optimize", algorithm, "--list-params"],
                         capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit(f"'{gecot} optimize {algorithm} --list-params' failed:\n"
                 f"{out.stdout}{out.stderr}")
    return "--time-limit" in out.stdout + out.stderr


def run(command, log_path, limit):
    """Returns the wall time and how the run ended: its exit code, 'ctrl-c'
    if it had to be interrupted or 'killed'."""
    start = time.monotonic()
    with open(log_path, "w") as log:
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                   stderr=log)
        end = "exit"
        try:
            process.wait(timeout=1.05 * limit + 10)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            end = "ctrl-c"
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                end = "killed"
    wall = time.monotonic() - start
    return wall, end if end != "exit" else str(process.returncode)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--suite", default="full",
                        help="full, short or a CSV file of instance,budget "
                        "(default: full)")
    parser.add_argument("--time-limit", type=float,
                        help="seconds per run (default: 900 for full, 120 "
                        "for short)")
    parser.add_argument("--gecot", default=os.path.join(HERE, "..", "build",
                                                        "gecot"),
                        help="gecot executable (default: ../build/gecot)")
    parser.add_argument("--label",
                        help="name of the results (default: the algorithm "
                        "and its parameters)")
    parser.add_argument("--results", default=os.path.join(HERE, "results"),
                        help="directory of the results (default: results/)")
    parser.add_argument("--instances", default=os.path.join(HERE, "instances"),
                        help="directory of the instances (default: "
                        "instances/)")
    parser.add_argument("--only", action="append", default=[],
                        metavar="INSTANCE", help="run this instance only "
                        "(can be repeated)")
    parser.add_argument("--force", action="store_true",
                        help="run again the runs already in the results")
    parser.add_argument("algorithm")
    parser.add_argument("parameters", nargs=argparse.REMAINDER,
                        help="parameters of the algorithm")
    args = parser.parse_args()

    suite_path, limit = SUITES.get(args.suite, (args.suite, 900))
    if args.time_limit is not None:
        limit = args.time_limit
    runs = read_suite(os.path.join(HERE, suite_path)
                      if args.suite in SUITES else suite_path)
    if args.only:
        runs = [r for r in runs if r[0] in args.only]
    gecot = os.path.abspath(args.gecot)
    label = args.label or "_".join(
        [args.algorithm] + [p.lstrip("-") for p in args.parameters])
    out_dir = os.path.join(args.results, label)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(args.results, label + ".csv")

    done = set()
    if os.path.exists(csv_path) and not args.force:
        with open(csv_path) as f:
            done = {(r["instance"], r["budget"]) for r in csv.DictReader(f)}
    elif os.path.exists(csv_path):
        os.remove(csv_path)
    todo = [r for r in runs if r not in done]
    limited = has_time_limit(gecot, args.algorithm)
    version = subprocess.run([gecot, "--version"], capture_output=True,
                             text=True).stdout.strip().splitlines()
    with open(os.path.join(out_dir, "meta.json"), "w") as f:
        json.dump({
            "algorithm": args.algorithm, "parameters": args.parameters,
            "suite": args.suite, "time_limit": limit, "gecot": gecot,
            "version": version[-1] if version else "",
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "host": os.uname().nodename, "cpus": os.cpu_count(),
        }, f, indent=4)
    print(f"{len(todo)} runs of {len(runs)} to do, at most "
          f"{len(todo) * (1.05 * limit + 40) / 3600:.1f} h; results in "
          f"{csv_path}", flush=True)

    new_file = not os.path.exists(csv_path)
    fields = FIELDS
    if not new_file:  # appended rows keep the columns of the file
        with open(csv_path) as f:
            fields = next(csv.reader(f), FIELDS)
    with open(csv_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        if new_file:
            writer.writeheader()
        for i, (instance, budget) in enumerate(todo):
            stem = os.path.join(out_dir, f"{instance}_{budget}")
            if os.path.exists(stem + ".json"):
                os.remove(stem + ".json")
            command = [gecot, "optimize", args.algorithm,
                       os.path.join(args.instances, instance,
                                    "instance.json"),
                       budget, "-v", "-o", stem + ".json"]
            if limited:
                command += ["--time-limit", str(limit)]
            command += args.parameters
            wall, end = run(command, stem + ".log", limit)
            row = {"instance": instance, "budget": budget, "exit": end,
                   "wall_time": f"{wall:.2f}", "status": "failed"}
            if os.path.exists(stem + ".json"):
                with open(stem + ".json") as g:
                    result = json.load(g)
                report = result.get("algorithm_report", {})
                row.update({
                    "status": report.get("status", ""),
                    "time": result["computation_time_ms"] / 1000,
                    "initial_score": repr(result["initial_score"]),
                    "score": repr(result["solution_score"]),
                    # an object per dimension with several costs
                    "cost": json.dumps(result["solution_cost"])
                    if isinstance(result["solution_cost"], dict)
                    else repr(result["solution_cost"]),
                    "feasible": result.get("feasible", ""),
                    "bound": repr(report["best_bound"])
                    if "best_bound" in report else "",
                    "nodes": report.get("nodes", ""),
                })
            writer.writerow(row)
            f.flush()
            print(f"[{i + 1}/{len(todo)}] {instance} {budget}: "
                  f"{row['status']} in {wall:.1f} s, score "
                  f"{row.get('score', '-')}, bound {row.get('bound', '-')}",
                  flush=True)
    print("command:", shlex.join([gecot, "optimize", args.algorithm,
                                  "<instance>", "<budget>"] +
                                 (["--time-limit", str(limit)] if limited
                                  else []) + args.parameters))


if __name__ == "__main__":
    main()
