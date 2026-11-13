#!/bin/sh
# Builds every instance of the benchmark in ../instances, as committed.
#   all.sh [cache directory]
# The reductions need Python 3 only; the landscapes and the bike networks need
# the packages of ../requirements.txt, and download to the cache directory
# what is not there: the BD TOPO of the department (3 GB once extracted) and
# the population grid of INSEE (0.5 GB).
set -e
cd "$(dirname "$0")"
out=../instances

python3 dks.py $out/dks-30 -n 30 --density 0.5 --kmax 15 --seed 1
python3 dks.py $out/dks-50 -n 50 --density 0.5 --kmax 25 --seed 1
python3 coverage.py $out/cover-40 -m 40 -n 320 --degree 2 --seed 1
python3 coverage.py $out/cover-80 -m 80 -n 640 --degree 3 --seed 1
python3 coverage.py $out/faircover-40 -m 40 -n 320 --degree 2 --groups 4 \
    --span 2 --seed 1
python3 coverage.py $out/faircover-80 -m 80 -n 640 --degree 3 --groups 8 \
    --span 2 --seed 1

python3 landscape.py landscapes.json --out $out ${1:+--cache "$1"}
python3 bike.py bikes.json --out $out ${1:+--cache "$1"}
