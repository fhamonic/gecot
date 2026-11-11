

# ![GECOT](misc/gecot_title.png)

GECOT, the Graph-based Ecological Connectivity Optimization Tool, is an open-source C++ tool designed to tackle systematic conservation (or restoration) planning problems. Given a graph representation of the landscape for each target species, a list of actions for improving the graphs elements and a budget, GECOT allows to select a group of actions whose cost is below the budget and whose impacts on the Landscapes maximizes the ecological connectivity, measured with the Probability of Connectivity indicator (PC).

[![Generic badge](https://img.shields.io/badge/C++-26-blue.svg?style=flat&logo=c%2B%2B)](https://en.cppreference.com/w/cpp/26)
[![Generic badge](https://img.shields.io/badge/CMake-3.25+-blue.svg?style=flat&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI1MDguOTkyIiBoZWlnaHQ9IjU1OC42NTYiIGZpbGw9IiNmZmZmZmYiIHhtbG5zOnY9Imh0dHBzOi8vdmVjdGEuaW8vbmFubyI+PHBhdGggZD0iTTYuMzU3IDQ2My4yOTZDNi43OCA0NjIuMDMyIDIzOS4wMTEtLjE0MiAyMzkuMTUzIDBjLjA2OS4wNjggNC45MzUgNTUuNzAzIDEwLjgxNSAxMjMuNjMybDkuMzg4IDEyNC43MzZjLS43MTYuNjc2LTUzLjc1MiA0NS44NjItMTE3Ljg1OCAxMDAuNDE0TDE1LjUxMyA0NTYuMDQzYy01LjE4NyA0LjQ0MS05LjMwNiA3LjcwNi05LjE1NSA3LjI1NHptNDAxLjAyOC0xMC4wNDlsLTEwMS42NjktNDEuODNjLS4zMzgtLjMzOC0zMy45MTItMzg3Ljk0OS0zMy42MjktMzg4LjIzNy4wOTgtLjA5OSA1My40OTYgMTA1Ljg1OSAxMTguNjYzIDIzNS40NjJsMTE4LjI0MiAyMzUuODg2Yy0uMTM0LjEzNC00NS44NTctMTguNDQzLTEwMS42MDgtNDEuMjgyek0wIDUwOS4zNzRjMy44NTgtMy43MSAxNTAuOTc2LTEyOC40ODQgMTUxLjI3Ni0xMjguMzAxLjIzOS4xNDUgNzAuNDczIDI5LjAwMyAxNTYuMDc1IDY0LjEyOWwxNTUuOTM2IDY0LjE1OWMuMTYyLjE2Mi0xMDQuMDc3LjI5NS0yMzEuNjQzLjI5NVMtLjE2MiA1MDkuNTI5IDAgNTA5LjM3NHoiLz48L3N2Zz4=)](https://cmake.org/cmake/help/latest/release/3.25.html)
[![Generic badge](https://img.shields.io/badge/Conan-2.0+-blue.svg?style=flat&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI0ODEiIGhlaWdodD0iNTEyIiBmaWxsPSIjZmZmIiB4bWxuczp2PSJodHRwczovL3ZlY3RhLmlvL25hbm8iPjxwYXRoIGQ9Ik0xMjEuNzQ1IDQyNS43MjRMLjcwNCAzMzkuOTYxVjIyNi42OTkgMTEzLjQzN2wxMDUuNjQtNTAuODYyTDIyMy43MjMgNi4xMDJsMTEuNzQtNS42MTEgNzguNzU4IDM4LjM1MSAxMjIuMjQ2IDU5LjU0MSA0My40ODggMjEuMTktLjAwNCAxMTcuMzM2LS4wMDQgMTE3LjMzNi02Ni4zNzQgNDMuOTk3LTExOC41ODEgNzguNjIxLTUyLjIwOCAzNC42MjR6bTE4Mi4xNDUgMjQuMDc0bDU0LjU4LTM0LjQ0OC4xNzgtMTA1LjEwNS0uNzA0LTEwNC43NjJjLS40ODUuMTg5LTI2LjU1MyAxNC45NDEtNTcuOTI4IDMyLjc4NGwtNTcuMDQ1IDMyLjQ0MS0uMTc4IDEwOC44NDUtLjE3OCAxMDguODQ1IDMuMzQ3LTIuMDc2IDU3LjkyOC0zNi41MjR6bTExOC41NjItNzYuNTM4bDQzLjM2MS0yOC4xNzEuMDI5LTEwMS42NjUtMS4wNjItMTAxLjI0N2MtLjYuMjMtMjEuMzQ0IDEyLjIyOC00Ni4wOTggMjYuNjYxbC00NS4wMDggMjYuMjQzLS4wMzEgMTA0LjgzMS0uMDMxIDEwNC44MzEgMi43MzktMS42NTZjMS41MDctLjkxMSAyMi4yNTItMTQuMzMzIDQ2LjEtMjkuODI3em0tNzEuNzI3LTE3OS4zODlsMTE1LjE0Ni02Ni4wOTVjMC0uMjQ1LTUxLjg3My0yNi41MDktMTE1LjI3My01OC4zNjZMMjM1LjMyNSAxMS40ODkgMjA4LjUxMyAyNC42MiA5Ny4yNzggNzkuMDlsLTg0LjUxMiA0MS45NDJjLS4wNjMuNDI2IDIyMS4wNjUgMTM3LjkyMSAyMjIuNjM1IDEzOC40MzEuMDk3LjAzMiA1MS45OTMtMjkuNDg1IDExNS4zMjMtNjUuNTkyek0yMTQuODAxIDIwNi4zMWMtMjQuMTIyLTQuMDc2LTUxLjEzNi0xNy44MjctNjcuNjA5LTM0LjQxNi0xMS4xNC0xMS4yMTgtMTUuNjMtMTkuNzQ1LTE2LjM0Ny0zMS4wNDItMS4yODItMjAuMjEyIDE2LjQzNi00MC42OTkgNDkuOTk3LTU3LjgxMSAyMS45NzMtMTEuMjA0IDQxLjA1Mi0xNi43ODcgNjMuOTEtMTguNzAyIDQ0LjE2Mi0zLjcgOTMuODM1IDE2LjQ5OSAxMjkuODEgNTIuNzg3bDYuMjMxIDYuMjg1LTkuNzUzIDUuNjI4Yy0xOC4wMDggMTAuMzkxLTQ2LjQ5MyAyNS4zNDEtNDcuMDc5IDI0LjcwOC0uMTExLS4xMi4zOC0yLjQzNyAxLjA5MS01LjE0OCA1LjAzNC0xOS4xOTUtNS4wNi0zNi4yMzItMjcuNzczLTQ2Ljg3Ni0xMi4xNjEtNS42OTktMjYuMjM2LTguNTczLTQxLjk4NC04LjU3NC0xNi4zNC0uMDAxLTI4LjcxNiAyLjc5My00MS41NTIgOS4zOC0xNy44OTQgOS4xODMtMjkuOTk4IDIyLjYzMS0zMi40NzcgMzYuMDgxLTEuNjg5IDkuMTY1IDIuNTAyIDE4LjY5NyAxMC45OTYgMjUuMDEzIDE0LjI1MyAxMC41OTcgMzkuMDc0IDE2LjI1NiA3MS43MzQgMTYuMzU0bDEyLjc3OC4wMzgtMjcuMTE0IDEzLjY1N2MtMTQuOTEzIDcuNTExLTI3LjU5IDEzLjY0Mi0yOC4xNzEgMTMuNjIzcy0zLjU5Mi0uNDYyLTYuNjkxLS45ODV6Ii8+PC9zdmc+)](https://conan.io/index.html)
[![Generic badge](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.html)

## Installation

For Windows, download and run the [last released](https://github.com/fhamonic/gecot/releases) installer.

For Linux, download and extract the precompiled binaries of the [last release](https://github.com/fhamonic/gecot/releases).

Otherwise, [here](https://github.com/fhamonic/gecot_docker) is a Docker container for building and testing GECOT.

## Usage

```
$ gecot --help
GECOT — Graph-based Ecological Connectivity Optimization Tool
Version: 2.0 (built on Nov  5 2026)

Usage:
  gecot --help
  gecot --version
  gecot optimize --help
  gecot optimize <algorithm> <instance> <budget> [<parameters> ...]
  gecot bound --help
  gecot bound <method> <instance> <budget> [<parameters> ...]

Commands:
  optimize    Select options within a budget to maximize the score
  bound       Bound the best score of the options within a budget
```

`gecot optimize` selects the options to buy within the budget:

```
$ gecot optimize --help
GECOT — Graph-based Ecological Connectivity Optimization Tool
Version: 2.0 (built on Nov  5 2026)

Usage:
  gecot optimize --help
  gecot optimize --list-algorithms
  gecot optimize <algorithm> --list-params
  gecot optimize <algorithm> <instance> <budget> [<parameters> ...]


POSITIONALS:
  algorithm TEXT              Algorithm to use
  instance TEXT               Instance JSON file
  budget FLOAT                Budget value

OPTIONS:
  -h,     --help              Display this help message
  -A,     --list-algorithms   List the available algorithms
  -P,     --list-params       List the parameters of the chosen algorithm
  -a,     --algorithm TEXT    Algorithm to use
  -i,     --instance TEXT     Instance JSON file
  -B,     --budget FLOAT      Budget value
  -o,     --output-json TEXT  Output solution data in JSON file
          --output-csv TEXT   Output solution value in CSV file
          --reformulation TEXT:{trivial,path_lp} [trivial]
                              Instance given to the algorithm, trivial: without the options of
                              cost 0, bought, or over the budget; path_lp: also without the
                              options that the path relaxation proves to be in no better
                              solution than its own
          --reformulation-params TEXT
                              Parameters of the path_lp of --reformulation path_lp, as
                              "time-limit=60 pricing=enumerate"; its MIP start is greedy_incr's
                              solution by default
  -v,     --verbose [0]       Log the algorithms' details, -vv everything with the solver's own
                              log on stdout
  -q,     --quiet             Silence all logging except errors
          --max-threads UINT:POSITIVE
                              Limits the number of threads used
```

`gecot bound` computes an upper bound on the scores `gecot optimize` can reach within the budget, the options of cost 0 being always bought, which certifies how far a solution is from the optimum:

```
$ gecot bound --help
GECOT — Graph-based Ecological Connectivity Optimization Tool
Version: 2.0 (built on Nov  5 2026)

Usage:
  gecot bound --help
  gecot bound --list-methods
  gecot bound <method> --list-params
  gecot bound <method> <instance> <budget> [<parameters> ...]


POSITIONALS:
  method TEXT                 Bound method to use
  instance TEXT               Instance JSON file
  budget FLOAT                Budget value

OPTIONS:
  -h,     --help              Display this help message
  -M,     --list-methods      List the available bound methods
  -P,     --list-params       List the parameters of the chosen method
          --method TEXT       Bound method to use
  -i,     --instance TEXT     Instance JSON file
  -B,     --budget FLOAT      Budget value
  -o,     --output-json TEXT  Output bound data in JSON file
  -v,     --verbose [0]       Log the methods' details, -vv everything with the solver's own
                              log on stdout
  -q,     --quiet             Silence all logging except errors
          --max-threads UINT:POSITIVE
                              Limits the number of threads used
```

`gecot optimize --list-algorithms` and `gecot bound --list-methods` describe the algorithms and the bound methods, and `--list-params` their parameters.
The `gecot_optimize <algorithm> ...` command of former releases is now `gecot optimize <algorithm> ...`.

The result is printed on stdout, unless `-o` or `--output-csv` writes it to a file. The log goes to stderr, each line after the seconds since the start: the steps of the run, then a table of the algorithm's progress, with a row when a value changes, at most every 5 s and at least every minute. Its `gain` is the best solution's over the initial score, `max gain` the best possible gain, proven, and `guaranteed` the share of this best possible gain that the solution is proven to reach. The rows of `tree_lp` and `path_lp` are their rounds, with the gain of their `relaxation`.
Ctrl-C stops the algorithm and keeps its best result so far, with the status `interrupted`; a second Ctrl-C quits at once. A MIP solver notices it only when it next reports its progress: within seconds with Gurobi (2 s into a root relaxation on Quebec), up to about 15 s apart with HiGHS on the same model, never with CBC, which reports none. The column generations of `tree_lp` and `path_lp` stop at the end of their current round, and the heuristics run to the end.


## Citation

Please cite 'GECOT' when using it in publications using:

> Hamonic, F., Vaxès, Y., Couëtoux, B., & Albert, C. H. (2025). GECOT: Graph-based ecological connectivity optimization tool. Methods in Ecology and Evolution, 00, 1–9. https://doi.org/10.1111/2041-210X.70055 

Additionally, releases are automatically published to Zenodo (https://doi.org/10.5281/zenodo.15295914)

## Acknowledgments
This work has been funded by [Region Sud](https://www.maregionsud.fr/) and [Natural Solutions](https://www.natural-solutions.eu/) ([PhD thesis](https://theses.fr/api/v1/document/2023AIXM9063) of François Hamonic), and by the ERC project [SCALED](https://www.scaled-erc.eu/), grant n°949812 (postdoctoral position).

