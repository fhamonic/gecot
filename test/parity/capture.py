#!/usr/bin/env python3
"""Captures what a gecot binary prints and writes on a fixed matrix of runs.
compare.py then tells whether two captures differ: a change of the code that
must preserve the behaviour is checked by capturing before and after it.

usage: capture.py GECOT_BIN OUT_DIR [-j JOBS] [--only REGEX]
                  [--timeout SECONDS] [--build-dir DIR] [--keep-models]

A capture is comparable with another one made on the same machine, with the
same compiler, solver and dependency revisions (meta.json records what the
binary and its build directory tell), from a build with the default release
flags. Every run of the matrix, and every run of path_lp or tree_lp, has
--max-threads 1: they add their columns in the order their pricing threads
finish. The solver keeps its own threads.

Equal captures mean equal outputs, model sizes, node and cut counts, and for
the runs that end in Gurobi the same model: a gurobi.env in the directory of
each run makes Gurobi write the model it solves, of which the capture keeps
the hash (the file itself with --keep-models). The model file lists the rows
and the columns in the order they were added, with their coefficients to 17
digits. What it does not tell:
- the order of the terms inside a row;
- the models of a run but the last one solved (a reformulation, a MIP start,
  the rounds of a column generation), nor the cuts added during a search;
- anything of a run with another solver.

The matrix: the command line surface (help, lists, parameters, errors), each
algorithm and bound with its main parameters on small instances, with a JSON
result and the log at -v, and the printed summary on four instances. The
instances with conflict sets or several costs are in it as a regression
check; only the others have to stay bitwise identical.
"""
import argparse, concurrent.futures, hashlib, json, os, platform, re
import shutil, subprocess, sys, time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
T = f"{REPO}/test/instances"
E = f"{REPO}/docs/examples"

# name, file, budgets
PLAIN = [
    ("test", f"{T}/test/instance.json", ["10"]),
    ("three-woods", f"{E}/three-woods.json", ["3", "5"]),
    ("two-species", f"{E}/two-species.json", ["3"]),
    ("aude", f"{T}/Aude/aude.json", ["3", "7"]),
    ("biobj-aude", f"{T}/Aude/biobj_aude.json", ["5"]),
    ("aix30", f"{T}/Aix/instance_30x30/instance.json", ["8"]),
    # the exact algorithms time out from budget 9 on
    ("dks30", f"{REPO}/benchmark/instances/dks-30/instance.json", ["3", "6"]),
]
CONFLICTS_AND_COSTS = [
    ("conflicts", f"{T}/conflicts/instance.json", ["money=4,area=5"]),
    ("tw-works", f"{E}/three-woods-works.json", ["7"]),
    ("tw-alternatives", f"{E}/three-woods-alternatives.json", ["5"]),
    ("tw-resources", f"{E}/three-woods-resources.json", ["money=7,area=2,crossings=1"]),
]
# unequal costs, where a change of the heuristics' ratios shows; too large
# for the exact algorithms, and for greedy --decremental (200 s)
HEURISTICS_ONLY = [("quebec", f"{T}/Quebec_438_RASY/instance.json", ["5000"])]

HEURISTICS = [
    "static", "static --decremental", "static --decremental --only-dec",
    "greedy", "greedy --decremental", "greedy --worst", "greedy --decremental --worst",
]
ALGORITHMS = HEURISTICS + [
    "flow_mip", "flow_mip --big-m plain", "flow_mip --big-m knapsack_dantzig",
    "flow_mip --big-m tight", "flow_mip --nb-mus 3", "flow_mip --preprocessing none",
    "flow_mip --all-targets --no-pruning", "flow_mip --lp-relaxation",
    "flow_mip --solver highs", "flow_mip --mip-start-algorithm greedy",
    "flow_mip --reformulation path_lp", "flow_mip --no-merging",
    "flow_mip --indicator-constraints",
    "multicut_benders", "multicut_benders --groups 0", "multicut_benders --big-m tight",
    "multicut_benders --big-m plain", "multicut_benders --grouping consecutive",
    "flow_benders", "flow_benders --big-m knapsack_dantzig", "flow_benders --big-m tight",
    "flow_benders --big-m plain", "flow_benders --mip-start-algorithm static",
    "flow_benders --indicator-constraints",
    "multiflow_benders", "multiflow_benders --no-root-cuts", "multiflow_benders --no-in-out",
    "multiflow_benders --no-stored-arcs",
    "tree_lp", "path_lp", "path_lp --pricing enumerate", "path_lp --mip-start-algorithm greedy",
]
CHEAP_BOUNDS = ["all_options", "qkp_planes", "qkp_planes --iterations 20", "split_lagrange"]
BOUNDS = CHEAP_BOUNDS + ["flow_mip", "multicut_benders", "flow_benders", "multiflow_benders", "path_lp"]
EXCLUDED = {
    # does not end in five minutes
    "dks30@6__multicut_benders_--big-m_plain",
    # the solver's LP, on its own threads, ends on one of two last digits
    "aix30@8__flow_mip_--lp-relaxation",
}
ALGORITHM_NAMES = ["static", "greedy", "flow_mip", "multicut_benders", "flow_benders",
                   "multiflow_benders", "tree_lp", "path_lp"]
METHOD_NAMES = ["all_options", "qkp_planes", "split_lagrange", "flow_mip", "multicut_benders",
                "flow_benders", "multiflow_benders", "path_lp"]


def slug(text):
    return re.sub(r"[^A-Za-z0-9.=-]+", "_", text).strip("_")


def surface_cases():
    """The command line itself: (id, arguments). I is the test instance and
    START a result of flow_mip on it, written before the cases run."""
    I, o = f"{T}/test/instance.json", "optimize"
    cases = [
        ("top-help", ["--help"]), ("top-none", []), ("top-version", ["--version"]),
        ("top-unknown", ["nope"]), ("top-option-first", ["-v", o]),
        ("opt-help", [o, "--help"]), ("opt-none", [o]), ("opt-list", [o, "-A"]),
        ("opt-params-none", [o, "-P"]), ("opt-verbose-only", [o, "-v"]),
        ("bound-help", ["bound", "--help"]),
        ("bound-list", ["bound", "-M"]), ("bound-unknown", ["bound", "nope", I, "10"]),
        ("bound-no-bound", ["bound", "tree_lp", I, "10", "-q"]),
        ("bound-static", ["bound", "static", I, "10", "-q"]),
    ]
    cases += [(f"opt-params-{a}", [o, a, "-P"]) for a in ALGORITHM_NAMES]
    cases += [(f"bound-params-{m}", ["bound", m, "-P"]) for m in METHOD_NAMES]
    numbered = [
        "static I 10 -q", "-a static -i I -B 10 -q", "static I 10 --feasibility-tolerance=1e-5 -q",
        "static I 10 -t1e-5 -q", "greedy I 10 --decremental --only-dec -q", "static I 10 -t 1e-5 -q",
        "-a static -i I -B 10 --feasibility-tolerance 1e-5", "static I 10 --bogus=3",
        "static I 10 --bogus 3", "static I", "static I -5", "nope x 1",
        "static I 10 -o F.json --max-threads 2 -q", "static I 10 --output-csv F.csv -q",
        "static I 10 -o F.json -q", "flow_mip I 10 --mip-start START -q",
        "flow_mip I 10 --mip-start=START -q", "flow_mip I 10 --nb-mus 3 -q",
        "flow_mip I 10 --nb-mus=3 -q", "static I 10 extra",
        "greedy I 10 --decremental --only-dec -q -t 1e-5", "greedy I 10 -t1e-5 -q --only-dec --decremental",
        "static I 10 -t", "static -t 1e-5 I 10 -q", "static I 10 -t -1e-5 -q",
        "static I 10 --max-threads -2 -q", "static I 10 --max-threads 0 -q", "static I abc",
        "static I 10 -qv", "-q static I 10", "-a static I 10 -q", "static I 10",
        "greedy I 10 --decremental --only-dec", "flow_mip I 10 --resolution 1e-6 -q",
        "flow_mip I 10 --resolution=1e-6 -q", "static I 10 -q -q", "-a static -a flow_mip -i I -B 10 -q",
        "--algorithm=static --instance=I --budget=10 -q", "-a static -i I -B10 -q",
        "static I 10 --output-json F.json --output-csv F.csv -q", "static I 1e2 -q", "static I 10 -q -h",
        "flow_mip I 10 --mip-start missing.json -q", "flow_mip I 10 --mip-start=missing.json -q",
        "static I 10 -- -t", "tree_lp I 10 --max-threads 1 -q", "static I 10 --decremental --only-dec -q",
        "greedy I 10 --decremental --worst -q", "static /nonexistent.json 10 -q",
        "static I 10 -o /nonexistent_dir/F.json -q", "tree_lp I 10 --nb-mus=2 --resolution=1e-6 --max-threads 1 -q",
        "greedy I 10 --worst -q", "greedy I 10 -q", "static I 10 -t abc", "flow_mip I 10 --nb-mus abc",
        "tree_lp I 10 --resolution abc", "static I 10 --only-dec -q", "static I 1,000 -q",
        "static I cost=10 -q", "static I money=10 -q",
        "flow_mip I 10 --reformulation path_lp --reformulation-params pricing=enumerate "
        "--max-threads 1 -q",
        "flow_mip I 10 --reformulation-params pricing=enumerate -q",
        # abbreviated options, an unknown flag after those of a MIP algorithm
        "static I 10 --quie", "static I 10 --feas=1e-5 -q", "greedy I 10 --only -q",
        "static I 10 --max 2 -q", "--list-alg", "static --list-p",
        "--alg=static --inst=I --bud=10 -q", "static I 10 --output F.json -q",
        "flow_mip I 10 -m -q", "multicut_benders I 10 -t 1e-6 -m -q",
        "multicut_benders I 10 -t1e-6 -m -q",
        "flow_mip I 10 --indicator-constraints --solver highs -q",
        "flow_mip I 10 --indicator-constraints --lp-relaxation -q",
        "flow_benders I 10 --indicator-constraints --solver copt -q",
    ]
    for n, text in enumerate(numbered):
        args = [I if w == "I" else w.replace("=I", f"={I}") for w in text.split()]
        cases.append((f"opt-{n:02d}-{slug(text)[:40]}", [o] + args))
    # a file name with a space, which split() cannot write
    cases.append(("opt-file-with-space", [o, "static", I, "10", "-o", "a b.json", "-q"]))
    return cases


def matrix_cases():
    cases = []
    for name, path, budgets in PLAIN + CONFLICTS_AND_COSTS + HEURISTICS_ONLY:
        heuristics_only = (name, path, budgets) in HEURISTICS_ONLY
        for budget in budgets:
            where = f"{name}@{slug(budget)}"
            for config in ALGORITHMS:
                if heuristics_only and (config not in HEURISTICS or
                                        config.startswith("greedy --decremental")):
                    continue
                algorithm, *params = config.split()
                cases.append((f"{where}__{slug(config)}", [
                    "optimize", algorithm, path, budget, *params, "-v", "--max-threads", "1",
                    "-o", "out.json"]))
            for config in CHEAP_BOUNDS if heuristics_only else BOUNDS:
                method, *params = config.split()
                cases.append((f"{where}__bound_{slug(config)}", [
                    "bound", method, path, budget, *params, "-v", "--max-threads", "1",
                    "-o", "out.json"]))
    # the summaries printed without -o: with one case, with two, with several
    # costs, and on the one instance of the three that split_lagrange accepts
    for name, path, budget in (("test", f"{T}/test/instance.json", "10"),
                               ("three-woods", f"{E}/three-woods.json", "5"),
                               ("two-species", f"{E}/two-species.json", "3"),
                               ("tw-resources", f"{E}/three-woods-resources.json",
                                "money=7,area=2,crossings=1")):
        for algorithm in ALGORITHM_NAMES:
            cases.append((f"{name}@{slug(budget)}__printed_{algorithm}", [
                "optimize", algorithm, path, budget, "--max-threads", "1"]))
        for method in METHOD_NAMES:
            cases.append((f"{name}@{slug(budget)}__printed_bound_{method}", [
                "bound", method, path, budget, "--max-threads", "1"]))
    return cases


def run(binary, out, case, timeout, keep_models):
    name, args = case
    cwd = os.path.join(out, "cases", name)
    os.makedirs(cwd)
    # read by Gurobi when its environment starts, in the current directory
    with open(os.path.join(cwd, "gurobi.env"), "w") as f:
        f.write("ResultFile model.mps\n")
    start = time.time()
    try:
        p = subprocess.run([binary, *args], cwd=cwd, capture_output=True, timeout=timeout)
        rc, stdout, stderr = str(p.returncode), p.stdout, p.stderr
    except subprocess.TimeoutExpired as e:
        rc, stdout, stderr = "timeout", e.stdout or b"", e.stderr or b""
    for file, content in (("rc", rc.encode()), ("stdout", stdout), ("stderr", stderr),
                          ("args", "\n".join(args).encode())):
        with open(os.path.join(cwd, file), "wb") as f:
            f.write(content)
    model = os.path.join(cwd, "model.mps")
    if os.path.exists(model) and not keep_models:
        with open(model, "rb") as f:
            digest = hashlib.sha256(f.read()).hexdigest()
        with open(os.path.join(cwd, "model.sha256"), "w") as f:
            f.write(digest + "\n")
        os.remove(model)
    return name, rc, time.time() - start


def meta(binary, out, build_dir):
    version = subprocess.run([binary, "--version"], capture_output=True, text=True).stdout
    packages, build = {}, {}
    toolchain = os.path.join(build_dir, "conan_toolchain.cmake")
    if os.path.exists(toolchain):
        for package in ("melon", "mippp"):
            found = re.search(rf'/p/(?:b/)?({package[:5]}\w+)/p/include', open(toolchain).read())
            packages[package] = found.group(1) if found else None
    cache = os.path.join(build_dir, "CMakeCache.txt")
    if os.path.exists(cache):
        for key in ("CMAKE_CXX_COMPILER", "CMAKE_BUILD_TYPE", "CMAKE_CXX_FLAGS_RELEASE"):
            found = re.search(rf"^{key}:\w+=(.*)$", open(cache).read(), re.M)
            build[key] = found.group(1) if found else None
    return {
        # compare.py replaces this path in the logs
        "out": out,
        "binary": binary,
        "sha256": hashlib.sha256(open(binary, "rb").read()).hexdigest(),
        "version": version.strip().splitlines()[-1] if version.strip() else None,
        "date": time.strftime("%Y-%m-%d %H:%M"),
        "host": platform.node(), "cpus": os.cpu_count(),
        # of this script, not of the binary
        "harness_commit": subprocess.run(
            ["git", "-C", REPO, "describe", "--always", "--dirty"],
            capture_output=True, text=True).stdout.strip(),
        "build": build,
        "conan_packages": packages,
        "solver_homes": {k: os.environ.get(k) for k in ("GUROBI_HOME", "CPLEX_HOME", "COPT_HOME")},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("binary")
    parser.add_argument("out")
    parser.add_argument("-j", "--jobs", type=int, default=1)
    parser.add_argument("--only", help="run the cases whose name matches")
    parser.add_argument("--timeout", type=float, default=300)
    parser.add_argument("--build-dir", help="where the binary was built, for the revisions "
                        "of its dependencies (default: its directory)")
    parser.add_argument("--keep-models", action="store_true",
                        help="keep the model files, not only their hash")
    args = parser.parse_args()
    binary, out = os.path.abspath(args.binary), os.path.abspath(args.out)
    if os.path.exists(out):
        sys.exit(f"{out} exists")
    os.makedirs(out)
    json.dump(meta(binary, out, args.build_dir or os.path.dirname(binary)),
              open(os.path.join(out, "meta.json"), "w"), indent=2)
    # the MIP start of the surface cases, by the captured binary itself
    start = os.path.join(out, "start.json")
    subprocess.run([binary, "optimize", "flow_mip", f"{T}/test/instance.json", "10", "-q",
                    "-o", start], check=True)
    cases = [(name, [start if a == "START" else a.replace("=START", f"={start}") for a in arguments])
             for name, arguments in surface_cases() + matrix_cases()]
    cases = [c for c in cases if c[0] not in EXCLUDED]
    if args.only:
        cases = [c for c in cases if re.search(args.only, c[0])]
    assert len({name for name, _ in cases}) == len(cases), "two cases share a name"
    slowest, timeouts = [], []
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        for name, rc, seconds in pool.map(
                lambda c: run(binary, out, c, args.timeout, args.keep_models), cases):
            slowest.append((seconds, name))
            if rc == "timeout":
                timeouts.append(name)
    print(f"{len(cases)} cases captured in {out}")
    for seconds, name in sorted(slowest, reverse=True)[:5]:
        print(f"  {seconds:6.1f} s  {name}")
    for name in timeouts:
        print(f"  TIMEOUT {name}")
    sys.exit(1 if timeouts else 0)


if __name__ == "__main__":
    main()
