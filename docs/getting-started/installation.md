# Installation

GECOT is a single command line program, `gecot`. Its heuristics work on their own; its exact algorithms and relaxations call a linear or mixed integer solver that must be installed on the machine.

## Getting GECOT

### Released binaries

The [releases page](https://github.com/fhamonic/gecot/releases) of the repository provides an installer for Windows and precompiled binaries for Linux.

!!! warning "The 1.0 release predates this documentation"

    This documentation describes GECOT 2.0. The 1.0 release installs a program named `gecot_optimize`, not `gecot`: its command is `gecot_optimize <algorithm> <instance> <budget>`, its instance files have a single `quality` column per vertex and a `quality_gain` column per improvement, and it has six algorithms: `mip`, `prep_mip` (now `flow_mip`) and the heuristics under the names `static_incr`, `greedy_incr`, `static_decr` and `greedy_decr` (now `static` and `greedy`, with `--decremental` for the last two). Until 2.0 is released, build it from the sources.

### Docker

The repository [gecot_docker](https://github.com/fhamonic/gecot_docker) holds a container for building and testing GECOT.

### From the sources

GECOT is written in C++26 and built with [CMake](https://cmake.org) (3.25 or later) and [Conan](https://conan.io) (2.0 or later), which fetches and builds its dependencies. It is developed with GCC 15.

1. Create a Conan profile for your compiler with the C++26 standard, for instance in `~/.conan2/profiles/gcc15_c++26`:

    ```ini
    [settings]
    arch=x86_64
    build_type=Release
    compiler=gcc
    compiler.cppstd=26
    compiler.libcxx=libstdc++11
    compiler.version=15
    os=Linux
    ```

2. One dependency is not on Conan Center: the solver interface [mippp](https://github.com/fhamonic/mippp). GECOT 2.0 needs its version 1.1.0 as it stands on the branch `feat/lp-algorithm`, until this branch is merged into mippp's `main`. Clone it and add it to your local Conan cache:

    ```console
    $ git clone -b feat/lp-algorithm https://github.com/fhamonic/mippp && cd mippp
    $ conan create . -pr=gcc15_c++26 -b=missing
    ```

    Conan fetches the other dependencies, the graph library [melon](https://github.com/fhamonic/melon) among them, when GECOT is built.

3. Build GECOT:

    ```console
    $ git clone https://github.com/fhamonic/gecot && cd gecot
    $ conan build . -of=build -b=missing -pr=gcc15_c++26
    ```

    The program is `build/gecot`. `make` runs the same command with the profile named in the `Makefile`, then the tests and the packaging.

!!! tip "Memory"

    Some source files instantiate the algorithms for every solver and need 2 to 2.5 GB of memory each to compile. On a machine with little memory, limit the parallel jobs, for instance with `-c tools.build:jobs=2` on the `conan build` command.

## Solvers

GECOT talks to the solvers through mippp, which loads the solver's shared library when the program runs. Nothing is linked when GECOT is built: a solver is available as soon as its library can be found, and one binary works with all of them.

| Algorithms | Solvers (`--solver`) |
|---|---|
| The heuristics; the bounds `all_options`, `qkp_planes`, `split_lagrange` | none needed |
| `flow_mip`, `path_lp` | `gurobi`, `cplex`, `copt`, `xpress`, `mosek`, `highs`, `scip`, `cbc`, `glpk` |
| `tree_lp` | the same without `glpk` |
| `multicut_benders`, `flow_benders`, `multiflow_benders` | `gurobi`, `cplex`, `copt` |

The default is `gurobi`. [HiGHS](https://highs.dev), [SCIP](https://www.scipopt.org), [CBC](https://github.com/coin-or/Cbc) and [GLPK](https://www.gnu.org/software/glpk/) are free and open source; Gurobi, CPLEX, COPT, Xpress and MOSEK are commercial, with free academic licences. The Benders decompositions, usually the fastest exact algorithms, need one of the three solvers that accept lazy constraints through mippp.

### How a solver library is found

For each solver, in this order:

1. the environment variable `MIPPP_<SOLVER>_LIBRARY`, if set, gives the exact library file, for instance

    ```console
    $ export MIPPP_HIGHS_LIBRARY=/opt/highs/lib/libhighs.so
    ```

    The names are `MIPPP_GUROBI_LIBRARY`, `MIPPP_CPLEX_LIBRARY`, `MIPPP_COPT_LIBRARY`, `MIPPP_XPRESS_LIBRARY`, `MIPPP_MOSEK_LIBRARY`, `MIPPP_HIGHS_LIBRARY`, `MIPPP_SCIP_LIBRARY`, `MIPPP_CBC_LIBRARY`, `MIPPP_CLP_LIBRARY` and `MIPPP_GLPK_LIBRARY`.

2. otherwise the library is searched under its usual names in the directories of the system's dynamic loader: those of `LD_LIBRARY_PATH` and the system library directories on Linux, of `PATH` on Windows, of `DYLD_LIBRARY_PATH` on macOS.

A solver installed in the usual way is therefore found without any setting. When the library cannot be loaded, the run stops with an error that says what was tried:

```text
error: Running algorithm 'flow_mip': mippp: failed to load the HIGHS solver library from MIPPP_HIGHS_LIBRARY:
  cannot load '/opt/missing/libhighs.so': /opt/missing/libhighs.so: cannot open shared object file: No such file or directory
```

mippp is validated against a range of versions of each solver. With a library outside this range it prints a warning and goes on; setting `MIPPP_NO_VERSION_WARNING` silences it.

Two solvers work in pairs: `--solver cbc` solves its linear programs with CLP, and `--solver scip` with HiGHS, so `path_lp` and `tree_lp` need both libraries of the pair.

## Checking the installation

```console
$ gecot --version
GECOT — Graph-based Ecological Connectivity Optimization Tool
Version: 2.0 (built on Nov 12 2026)
```

Then follow [A first run](first-run.md): its example takes a few milliseconds and tells at once whether the solver is found.
