#!/usr/bin/env python3
"""Compares two captures of capture.py and exits with 1 if they differ.

usage: compare.py CAPTURE_A CAPTURE_B [--only REGEX] [--ignore REGEX]

For each case: the exit code, what was printed, the log, the files written
and the model given to Gurobi, without what changes from one run to the next:
- the times: the "Time" line of a summary, the "*_time_ms" fields, the
  seconds that begin each log line;
- the tables of the progress, of a search or of rounds, whose rows are
  sampled on the clock, and the "Ctrl-C stops" line that comes with them;
- the build date, the lines that name an output file, and the directory of
  the capture.
A case that timed out on either side is reported as different: its end was
not captured.

--ignore drops the lines that match: the lines of a new parameter, to check
that a change adds nothing else. A new last property of a JSON object adds
only its own line (the comma that ends a line is not compared), but the
continuation lines of a wrapped help text and the cases that only one side
has are still reported.
"""
import argparse, json, os, re, sys

PREFIX = re.compile(r"^\[\s*[0-9.]+s\] ?")
RULE = re.compile(r"^-+(\+-+)+$")
BUILD_DATE = re.compile(r"built on [^)]*")
DROPPED = re.compile(
    r'^\s*"\w+_time_ms":|^Time {2,}|^Ctrl-C stops|^(Solution|Bound) written to ')
# the headers of the tables that the progress fills as time passes
PROGRESS_TABLE = re.compile(
    r"^\s*(gain\s*\|\s*max gain\s*\|\s*guaranteed|round\s*\|\s*(relaxation|fractional))")


def normalized(path, capture, ignore):
    if not os.path.exists(path):
        return ["<missing>"]
    kept, table, rules = [], [], 0

    def close():
        nonlocal rules
        if not any(PROGRESS_TABLE.search(row) for row in table):
            kept.extend(table)
        table.clear()
        rules = 0
    # newline="": a carriage return stays in its line and is compared
    for line in open(path, newline="", errors="surrogateescape").read().split("\n"):
        line = PREFIX.sub("", line).replace(capture, "<CAPTURE>")
        line = BUILD_DATE.sub("built on <DATE>", line)
        if path.endswith(".json"):
            line = line.rstrip(",")
        dropped = DROPPED.search(line) or (ignore and ignore.search(line))
        rule = RULE.match(line.strip())
        # A table is a rule, its header, a rule, its rows and a rule. It is
        # closed there, and by any other line: two tables that follow each
        # other, or that only a dropped line separates, are not one table,
        # which would be dropped whole if one of them is a progress table.
        if rule or " | " in line:
            if not dropped:
                table.append(line)
                rules += bool(rule)
                if rules == 3:
                    close()
        else:
            close()
            if not dropped:
                kept.append(line)
    close()
    return kept


def first_difference(a, b):
    for n, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return f"line {n + 1}: {x!r} | {y!r}"
    return f"line {min(len(a), len(b)) + 1}: one side ends"


def recorded_path(capture):
    """Where the capture was written: the path that its logs contain, which
    is not its present one if it was moved."""
    meta = json.load(open(f"{capture}/meta.json"))
    return meta.get("out", capture)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("a")
    parser.add_argument("b")
    parser.add_argument("--only", help="compare the cases whose name matches")
    parser.add_argument("--ignore", type=re.compile, help="drop the lines that match")
    args = parser.parse_args()
    a, b = os.path.abspath(args.a), os.path.abspath(args.b)
    path_a, path_b = recorded_path(a), recorded_path(b)
    names = sorted(set(os.listdir(f"{a}/cases")) | set(os.listdir(f"{b}/cases")))
    if args.only:
        names = [n for n in names if re.search(args.only, n)]
    different = 0
    for name in names:
        da, db = f"{a}/cases/{name}", f"{b}/cases/{name}"
        if not os.path.isdir(da) or not os.path.isdir(db):
            different += 1
            print(f"{name}\n    only in {a if os.path.isdir(da) else b}")
            continue
        files = sorted((set(os.listdir(da)) | set(os.listdir(db))) - {"args"})
        found = []
        if "timeout" in (open(f"{da}/rc").read(), open(f"{db}/rc").read()):
            found.append("rc: timeout, the end of the run is not captured")
        for file in files:
            x = normalized(f"{da}/{file}", path_a, args.ignore)
            y = normalized(f"{db}/{file}", path_b, args.ignore)
            if x != y:
                found.append(f"{file}: {first_difference(x, y)}")
        if found:
            different += 1
            print(name)
            for f in found:
                print("    " + f[:300])
    print(f"{len(names)} cases, {different} differ")
    sys.exit(1 if different else 0)


if __name__ == "__main__":
    main()
