GECOT : https://github.com/fhamonic/gecot

# Installation

## Windows

Run the installer named "Gecot-1.0-win64.msi".

## Linux

For Linux, extract the precompiled binaries contained in "Gecot-1.0-Linux.tar.gz".

The main executable is under "bin/gecot_optimize".

## Otherwise

Here is a Docker container for running GECOT : https://github.com/fhamonic/gecot_docker

# Usage

```
$ gecot_optimize --help
GECOT — Graph-based Ecological Connectivity Optimization Tool
Version: 1.0 (built on Dec  1 2024)

Usage:
  gecot_optimize --help
  gecot_optimize --list-algorithms
  gecot_optimize <algorithm> --list-params
  gecot_optimize <algorithm> <instance> <budget> [<parameters> ...]

Allowed parameters:
  -h [ --help ]             Display this help message
  -A [ --list-algorithms ]  List the available algorithms
  -P [ --list-params ]      List the parameters of the chosen algorithm
  -a [ --algorithm ] arg    Algorithm to use
  -i [ --instance ] arg     Instance JSON file
  -B [ --budget ] arg       Budget value
  -o [ --output-json ] arg  Output solution data in JSON file
  --output-csv arg          Output solution value in CSV file
  -v [ --verbose ]          Enable all logging informations
  -q [ --quiet ]            Silence all logging except errors
```


# Acknowledgments
This work has been funded by [Region Sud](https://www.maregionsud.fr/) and [Natural Solutions](https://www.natural-solutions.eu/) ([PhD thesis](https://theses.fr/api/v1/document/2023AIXM9063) of François Hamonic), and by the ERC project [SCALED](https://www.scaled-erc.eu/), grant n°949812 (postdoctoral position).

