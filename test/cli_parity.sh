#!/usr/bin/env bash
# CLI parity matrix: runs every case against OLD_BIN and NEW_BIN, each from its
# own scratch cwd, and prints one row per case:
#   id | args
#       rc old/new | stdout | files
#       first error line of old / new
# stdout is compared after dropping timing lines; output files are compared
# after dropping the computation_time_ms field.
# Usage: cli_parity.sh OLD_BIN NEW_BIN
set -u
if [ $# -ne 2 ]; then
    echo "usage: $0 OLD_BIN NEW_BIN" >&2
    exit 2
fi
HERE=$(dirname "$(realpath "$0")")
OLD=$(realpath "$1")
NEW=$(realpath "$2")
I=$HERE/instances/test/instance.json
OUT=$(mktemp -d)
echo "cli_parity: outputs in $OUT" >&2
ulimit -c 0

# MIP start file shared by the --mip-start cases, produced by OLD_BIN
START=$OUT/start.json
"$OLD" mip "$I" 10 -o "$START" -q

run_bin() { # tag bin id args...
    local tag=$1 bin=$2 id=$3
    shift 3
    local wd=$OUT/$id/$tag
    mkdir -p "$wd"
    (cd "$wd" && timeout 10 "$bin" "$@" >stdout 2>stderr; echo $? >rc)
}

first_error() { # dir
    grep -h -m1 '^\[error\]' "$1/stdout" "$1/stderr" | head -n1
}

filtered_stdout() { # dir
    grep -v -E 'Computation time: [0-9]+ ms|ran in [0-9]+ ms|built on ' "$1/stdout"
}

compare_files() { # id
    local b=$OUT/$1/old n=$OUT/$1/new res="" f
    for f in $(cd "$b" && ls | grep -v -E '^(stdout|stderr|rc)$'); do
        if [ ! -f "$n/$f" ]; then
            res+="$f:missing "
        elif diff -q <(grep -v computation_time_ms "$b/$f") \
            <(grep -v computation_time_ms "$n/$f") >/dev/null; then
            res+="$f:same "
        else
            res+="$f:DIFF "
        fi
    done
    echo "${res:--}"
}

case_() { # id args...
    local id=$1
    shift
    run_bin old "$OLD" "$id" "$@"
    run_bin new "$NEW" "$id" "$@"
    local ro rn eo en so
    ro=$(cat "$OUT/$id/old/rc")
    rn=$(cat "$OUT/$id/new/rc")
    eo=$(first_error "$OUT/$id/old")
    en=$(first_error "$OUT/$id/new")
    if diff -q <(filtered_stdout "$OUT/$id/old") \
        <(filtered_stdout "$OUT/$id/new") >/dev/null; then
        so=same
    else
        so=DIFF
    fi
    local shown="$*"
    shown=${shown//$I/I}
    shown=${shown//$START/START}
    printf '%s | %s\n    rc %s/%s | stdout %s | files %s\n    old: %s\n    new: %s\n' \
        "$id" "$shown" "$ro" "$rn" "$so" "$(compare_files "$id")" \
        "${eo:--}" "${en:--}"
}

# measured baseline cases
case_ B01 --help
case_ B02
case_ B03 -A
case_ B04 static_incr -P
case_ B05 greedy_decr -P
case_ B06 mip -P
case_ B07 -P
case_ B08 static_incr "$I" 10 -q
case_ B09 -a static_incr -i "$I" -B 10 -q
case_ B10 static_incr "$I" 10 --feasibility-tolerance=1e-5 -q
case_ B11 static_incr "$I" 10 -t1e-5 -q
case_ B12 greedy_decr "$I" 10 --only-dec -q
case_ B13 static_incr "$I" 10 -t 1e-5 -q
case_ B14 -a static_incr -i "$I" -B 10 --feasibility-tolerance 1e-5
case_ B15 static_incr "$I" 10 --bogus=3
case_ B16 static_incr "$I" 10 --bogus 3
case_ B17 static_incr "$I"
case_ B18 static_incr "$I" -5
case_ B19 nope x 1
case_ B20 static_incr "$I" 10 -o F.json --max-threads 2 -q
# required extras
case_ R01 static_incr "$I" 10 --output-csv F.csv -q
case_ R02 static_incr "$I" 10 -o F.json -q
case_ R03 mip "$I" 10 --mip-start "$START" -q
case_ R04 mip "$I" 10 --mip-start="$START" -q
case_ R05 target_benders "$I" 10 --nb-mus 3 -q
case_ R06 target_benders "$I" 10 --nb-mus=3 -q
case_ R07 static_incr "$I" 10 extra
case_ R08 -v
# further extras
case_ E01 greedy_decr "$I" 10 --only-dec -q -t 1e-5
case_ E02 greedy_decr "$I" 10 -t1e-5 -q --only-dec
case_ E03 static_incr "$I" 10 -t
case_ E04 static_incr -t 1e-5 "$I" 10 -q
case_ E05 static_incr "$I" 10 -t -1e-5 -q
case_ E06 static_incr "$I" 10 --max-threads -2 -q
case_ E07 static_incr "$I" 10 --max-threads 0 -q
case_ E08 static_incr "$I" abc
case_ E09 static_incr "$I" 10 -qv
case_ E10 -q static_incr "$I" 10
case_ E11 -a static_incr "$I" 10 -q
case_ E12 static_incr "$I" 10
case_ E13 greedy_decr "$I" 10 --only-dec
case_ E14 mip "$I" 10 -m -q
case_ E15 prep_mip "$I" 10 --resolution 1e-6 -q
case_ E16 prep_mip "$I" 10 --resolution=1e-6 -q
case_ E17 static_incr "$I" 10 -q -q
case_ E18 -a static_incr -a mip -i "$I" -B 10 -q
case_ E19 static_incr "$I" 10 -o "a b.json" -q
case_ E20 --algorithm=static_incr --instance="$I" --budget=10 -q
case_ E21 -a static_incr -i "$I" -B10 -q
case_ E22 static_incr "$I" 10 --output-json F.json --output-csv F.csv -q
case_ E23 static_incr "$I" 1e2 -q
case_ E24 static_incr "$I" 10 -q -h
case_ E25 mip "$I" 10 --mip-start missing.json -q
case_ E26 mip "$I" 10 --mip-start=missing.json -q
case_ E27 static_incr "$I" 10 -- -t
case_ E28 tree_rr "$I" 10 -q
case_ E29 static_decr "$I" 10 --only-dec -q
case_ E30 worst_greedy_decr -P
case_ E31 static_incr /nonexistent.json 10 -q
case_ E32 static_incr "$I" 10 -o /nonexistent_dir/F.json -q
case_ E33 flow_benders "$I" 10 --nb-mus=2 --resolution=1e-6 -q
case_ E34 naive_benders "$I" 10 -t 1e-6 -m -q
case_ E35 worst_greedy_incr "$I" 10 -q
case_ E36 greedy_incr "$I" 10 -q
# space-separated solver values must reach the option, not a 4th positional
case_ V01 static_incr "$I" 10 -t abc
case_ V02 prep_mip "$I" 10 --nb-mus abc
case_ V03 target_benders "$I" 10 --resolution abc
# abbreviated long options (boost's default style allows guessing)
case_ G01 static_incr "$I" 10 --quie
case_ G02 static_incr "$I" 10 --feas=1e-5 -q
case_ G03 greedy_decr "$I" 10 --only -q
case_ G04 static_incr "$I" 10 --max 2 -q
case_ G05 --list-alg
case_ G06 static_incr --list-p
case_ G07 --alg=static_incr --inst="$I" --bud=10 -q
case_ G08 static_incr "$I" 10 --output o.json -q
# attached spellings of the space-form cases, for the cross-checks below
case_ X14 -a static_incr -i "$I" -B 10 --feasibility-tolerance=1e-5
case_ X34 naive_benders "$I" 10 -t1e-6 -m -q
# every algorithm with defaults and full trace logging
for alg in static_incr static_decr greedy_incr greedy_decr mip prep_mip \
    naive_benders target_benders flow_benders tree_rr worst_greedy_incr \
    worst_greedy_decr; do
    case_ "A-$alg" "$alg" "$I" 10 -v
done

cross_() { # new_case old_case [tag of the second run, default old]
    local tag=${3:-old}
    if diff -q <(filtered_stdout "$OUT/$1/new") \
        <(filtered_stdout "$OUT/$2/$tag") >/dev/null; then
        echo "cross $1(new) vs $2($tag): stdout same"
    else
        echo "cross $1(new) vs $2($tag): stdout DIFF"
    fi
}
# abbreviated options land on the right option
cross_ G01 B08
cross_ G02 B10
cross_ G03 B12
cross_ G04 B08
cross_ G05 B03
cross_ G06 B04 new
cross_ G07 B08
cross_ B13 B10
cross_ B14 X14
cross_ R03 R04
cross_ R05 R06
cross_ E01 E02
cross_ E11 B08
cross_ E15 E16
cross_ E17 B08
cross_ E34 X34
if diff -q --ignore-matching-lines=computation_time_ms \
    "$OUT/E19/new/a b.json" "$OUT/R02/old/F.json" >/dev/null; then
    echo "cross E19(new 'a b.json') vs R02(old F.json): same"
else
    echo "cross E19(new 'a b.json') vs R02(old F.json): DIFF"
fi
