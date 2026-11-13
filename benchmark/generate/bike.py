#!/usr/bin/env python3
"""Bike network design instances of GECOT, built from OpenStreetMap (ODbL)
and the population grid of INSEE (Filosofi 2019, 200 m cells).

    bike.py bikes.json                every instance of the file
    bike.py bikes.json bike-centre    this one only

The area of an instance is a rectangle given in longitude and latitude.

Graph. The vertices and the arcs are those of the bike network of OSMnx
(network type "bike", simplified, every street in both directions), as
OpenStreetMap was on OSM_DATE. The answers of the Overpass server for the
instances of bikes.json are in osm.tar.gz, and are used instead of asking
again: dated queries are slow and often time out on the public server. An arc
of length d (m) has the probability

    exp(-d / alpha)

alpha (m) being 6000 with a cycling infrastructure (a cycleway, or a cycleway
tag of LANE_TAGS on the street), and else the one of its kind of street in
ALPHA, the lowest if OSMnx merged several kinds.

Qualities. Each 200 m cell of the INSEE grid gives its population to the
vertex nearest to its centre, as source quality; each point of interest (the
amenities, shops and buildings of POI_QUERY) adds 1 to the target quality of
the vertex nearest to the centre of its cell of the same grid. The score
weighs each pair (inhabitant, point of interest) by the probability of the
most reliable path between them.

Options. A bike lane on a section of main road (MAIN_ROADS: tertiary and
above) without cycling infrastructure: it raises alpha to 6000 on both
directions. A section is the arcs of a same street name whose middles fall in
the same H3 cell at "group_resolution"; an unnamed street is its own section.
The cost is the length of the section, in metres.

Besides the instance, the directory gets vertices with their coordinates,
options_details.csv (street, length and arcs of each option), bike.json and
map.png.

Squirrels. With "squirrel", the instance has a second case: the squirrels of
the same rectangle, on the cells of landscape.py (its "department" and
"resolution" are given there). The two cases share the options:
- a lane comes with a planted strip, "strip_width" metres wide along the
  section. In each cell the section runs through, the strip turns road surface
  into woody cover: it raises the probability of the cell, and its quality if
  it is habitat. The option costs "lane_cost" per metre of lane plus
  "strip_cost" per square metre planted (thousands of euros);
- a rope bridge ("rope_bridge": cost, probability, spacing of the sites) is a
  crossing for squirrels between two cells that a barrier separates. It does
  nothing for bikes.
The criterion is the lower, over the two cases, of 1 + the share of its
largest gain that the case has reached: its score minus its initial score,
over its score with every option minus its initial score. It is 1 without
any option and 2 with all of them.
"""
import argparse
import csv
import hashlib
import json
import math
import os
import sys
import tarfile
import time
import zipfile

import h3
import numpy as np
import osmnx as ox
import requests
import shapely
from pyproj import Transformer
from shapely.geometry import Polygon

import landscape
from landscape import write

OSM_DATE = "2026-11-01T00:00:00Z"
OVERPASS = "https://overpass-api.de/api/interpreter"
AGENT = "gecot benchmark (https://github.com/fhamonic/gecot)"
INSEE = ("https://www.insee.fr/fr/statistiques/fichier/7655475/"
         "Filosofi2019_carreaux_200m_csv.zip")

LANE_ALPHA = 6000
LANE_TAGS = {"lane", "track", "separate", "segregated", "opposite_lane",
             "opposite_track"}
CYCLEWAY_KEYS = ["cycleway", "cycleway:left", "cycleway:right",
                 "cycleway:both"]
ALPHA = {
    "cycleway": LANE_ALPHA,
    "service": 4000, "residential": 4000, "living_street": 4000,
    "unclassified": 4000,
    "tertiary": 3000, "tertiary_link": 3000,
    "secondary": 2000, "secondary_link": 2000,
    "primary": 2000, "primary_link": 2000,
    "trunk": 2000, "trunk_link": 2000,
}
DEFAULT_ALPHA = 3000  # tracks, paths, pedestrian streets
MAIN_ROADS = {kind for kind, alpha in ALPHA.items() if alpha <= 3000}

POI_QUERY = """(
  node["amenity"~"^(cafe|school|community_center|pharmacy|doctors|restaurant|bar|bicycle_parking|university)$"](poly:"{poly}");
  way["shop"](poly:"{poly}");
  way["building"~"^(retail|supermarket|train_station|sports_centre|sports_hall|university|stadium)$"](poly:"{poly}");
);
out center;"""


def patiently(fetch):
    """The public Overpass server often times out on a dated query. OSMnx
    keeps the pieces it received, so each new try goes further."""
    for wait in (30, 60, 120, 240, 480):
        try:
            return fetch()
        except (ox._errors.ResponseStatusCodeError,
                requests.RequestException) as error:
            print(f"Overpass: {str(error)[:80]}; new try in {wait} s",
                  flush=True)
            time.sleep(wait)
    return fetch()


def values(tag):
    """The values of a tag of an OSMnx edge: several if it merged streets."""
    if isinstance(tag, list):
        return tag
    return [] if tag is None or tag != tag else [tag]


def population_cells(cache, bounds):
    """Centre (EPSG:3035) and population of the INSEE cells within bounds."""
    path = os.path.join(cache, "carreaux_200m_met.csv")
    if not os.path.exists(path):
        import py7zr
        os.makedirs(cache, exist_ok=True)
        archive = os.path.join(cache, "Filosofi2019_carreaux_200m_csv.zip")
        if not os.path.exists(archive):
            print("downloading the INSEE population grid", flush=True)
            with requests.get(INSEE, stream=True) as r:
                r.raise_for_status()
                with open(archive, "wb") as f:
                    for chunk in r.iter_content(chunk_size=1 << 20):
                        f.write(chunk)
        with zipfile.ZipFile(archive) as z:
            z.extractall(cache)
        with py7zr.SevenZipFile(os.path.join(
                cache, "Filosofi2019_carreaux_200m_csv.7z")) as z:
            z.extractall(cache)
    xmin, ymin, xmax, ymax = bounds
    cells = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        ident, ind = header.index("idcar_200m"), header.index("ind")
        for row in reader:
            # CRS3035RES200mN<north>E<east>: the south-west corner
            y = int(row[ident][15:22]) + 100
            x = int(row[ident][23:30]) + 100
            if xmin <= x <= xmax and ymin <= y <= ymax:
                cells.append((x, y, float(row[ind])))
    return cells


def points_of_interest(cache, west, south, east, north):
    """Longitude and latitude of the points of interest of the rectangle."""
    poly = f"{south} {west} {south} {east} {north} {east} {north} {west}"
    query = f'[out:json][timeout:120][date:"{OSM_DATE}"];\n' + \
        POI_QUERY.format(poly=poly)
    path = os.path.join(cache, "overpass_" + hashlib.sha256(
        query.encode()).hexdigest()[:16] + ".json")
    if not os.path.exists(path):
        os.makedirs(cache, exist_ok=True)
        def fetch():
            # the server refuses the default agent of requests
            response = requests.post(OVERPASS, data={"data": query},
                                     timeout=200,
                                     headers={"User-Agent": AGENT})
            response.raise_for_status()
            return response.json()
        with open(path, "w") as f:
            json.dump(patiently(fetch), f)
    with open(path) as f:
        elements = json.load(f)["elements"]
    return sorted((e["lon"], e["lat"]) if e["type"] == "node" else
                  (e["center"]["lon"], e["center"]["lat"]) for e in elements)


def build(name, config, cache, out_root):
    out = os.path.join(out_root, name)
    os.makedirs(out, exist_ok=True)
    west, south, east, north = config["bbox"]

    archive = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "osm.tar.gz")
    if os.path.exists(archive):
        with tarfile.open(archive) as tar:
            tar.extractall(cache, members=[
                m for m in tar.getmembers()
                if not os.path.exists(os.path.join(cache, m.name))])
    ox.settings.use_cache = True
    ox.settings.cache_folder = os.path.join(cache, "osmnx")
    ox.settings.bidirectional_network_types = ["walk", "bike"]
    ox.settings.useful_tags_way = ["highway", "name"] + CYCLEWAY_KEYS
    ox.settings.overpass_settings = \
        '[out:json][timeout:{timeout}]{maxsize}[date:"%s"]' % OSM_DATE
    # a dated query over more than a few km2 times out on the public server:
    # OSMnx then asks for the area in pieces of this size (m2)
    ox.settings.max_query_area_size = 4e6
    rectangle = Polygon([(west, south), (east, south), (east, north),
                         (west, north)])
    graph = patiently(lambda: ox.graph_from_polygon(
        rectangle, network_type="bike", retain_all=True))
    nodes, edges = ox.graph_to_gdfs(graph)
    node_ids = sorted(nodes.index)
    to_3035 = Transformer.from_crs(4326, 3035, always_xy=True)
    xs, ys = to_3035.transform(nodes.loc[node_ids, "x"].values,
                               nodes.loc[node_ids, "y"].values)
    tree = shapely.STRtree(shapely.points(xs, ys))

    # ---- qualities ----------------------------------------------------
    population = np.zeros(len(node_ids))
    cells = population_cells(cache, (xs.min() - 200, ys.min() - 200,
                                     xs.max() + 200, ys.max() + 200))
    corner = to_3035.transform([west, east, east, west],
                               [south, south, north, north])
    area = Polygon(zip(*corner))
    cells = [c for c in cells if area.contains(shapely.Point(c[0], c[1]))]
    for i, c in zip(tree.nearest(shapely.points(
            [c[0] for c in cells], [c[1] for c in cells])), cells):
        population[i] += c[2]
    interest = np.zeros(len(node_ids))
    pois = points_of_interest(cache, west, south, east, north)
    px, py = to_3035.transform([p[0] for p in pois], [p[1] for p in pois])
    for i in tree.nearest(shapely.points(
            np.floor(np.asarray(px) / 200) * 200 + 100,
            np.floor(np.asarray(py) / 200) * 200 + 100)):
        interest[i] += 1

    # ---- arcs ---------------------------------------------------------
    arcs = []  # (id, u, v, length, alpha, improvable, name, middle, lane)
    for (u, v, k), e in sorted(edges.iterrows(), key=lambda item: item[0]):
        kinds = values(e.get("highway"))
        lane = "cycleway" in kinds or any(
            t in LANE_TAGS for key in CYCLEWAY_KEYS for t in values(e.get(key)))
        alpha = LANE_ALPHA if lane else min(
            (ALPHA.get(kind, DEFAULT_ALPHA) for kind in kinds),
            default=DEFAULT_ALPHA)
        names = values(e.get("name"))
        middle = e.geometry.interpolate(0.5, normalized=True)
        arcs.append((f"{u}_{v}_{k}", u, v, float(e["length"]), alpha,
                     not lane and any(kind in MAIN_ROADS for kind in kinds),
                     " / ".join(sorted(names)), (middle.y, middle.x), lane))

    # ---- options ------------------------------------------------------
    sections = {}
    for a in arcs:
        if not a[5]:
            continue
        cell = h3.latlng_to_cell(a[7][0], a[7][1], config["group_resolution"])
        key = (a[6], cell) if a[6] else \
            ("", f"{min(a[1], a[2])}_{max(a[1], a[2])}")
        sections.setdefault(key, []).append(a)
    options = []  # (id, street, cost, arcs)
    for n, key in enumerate(sorted(sections)):
        members = sections[key]
        # each street once, although it has an arc per direction
        streets = {(min(a[1], a[2]), max(a[1], a[2]), round(a[3], 1))
                   for a in members}
        options.append((f"lane_{n}", key[0], sum(s[2] for s in streets),
                        members))

    # ---- files --------------------------------------------------------
    def bike_case(prefix):
        write(out, prefix + "vertices.csv",
              "id,source_quality,target_quality,longitude,latitude",
              ((n, f"{population[i]:.6g}", f"{interest[i]:.6g}",
                f"{nodes.at[n, 'x']:.7f}", f"{nodes.at[n, 'y']:.7f}")
               for i, n in enumerate(node_ids)))
        write(out, prefix + "arcs.csv", "id,from,to,probability",
              ((a[0], a[1], a[2], f"{math.exp(-a[3] / a[4]):.12g}")
               for a in arcs))
        write(out, prefix + "arcs_improvements.csv",
              "arc_id,option_id,improved_probability",
              ((a[0], o, f"{math.exp(-a[3] / LANE_ALPHA):.12g}")
               for o, street, cost, members in options for a in members))
        return {
            "vertices": {"csv_file": prefix + "vertices.csv"},
            "arcs": {"csv_file": prefix + "arcs.csv"},
            "arcs_improvements": {"csv_file": prefix + "arcs_improvements.csv"},
        }

    length = lambda keep: round(sum(a[3] for a in arcs if keep(a)) / 2000, 1)
    summary = {
        "vertices": len(node_ids), "arcs": len(arcs),
        "area_km2": round(area.area / 1e6, 1),
        "population": round(float(population.sum())),
        "source_vertices": int((population > 0).sum()),
        "points_of_interest": len(pois),
        "target_vertices": int((interest > 0).sum()),
        "streets_km": {
            "with a lane": length(lambda a: a[8]),
            "improvable": length(lambda a: a[5]),
            "other": length(lambda a: not a[8] and not a[5])},
        "options": len(options),
        "total_cost": round(sum(o[2] for o in options), 1),
    }
    if "squirrel" in config:
        cases, criterion = with_squirrels(
            config, cache, out, node_ids, edges, population, interest, arcs,
            options, summary)
        cases = {"bike": bike_case("bike_"), **cases}
    else:
        write(out, "options.csv", "id,cost",
              ((o, f"{cost:.1f}") for o, street, cost, members in options))
        write(out, "options_details.csv", "id,street,length,arcs",
              ((o, '"' + street.replace('"', "'") + '"', f"{cost:.1f}",
                " ".join(a[0] for a in members))
               for o, street, cost, members in options))
        cases, criterion = {"bike": bike_case("")}, "bike"
        improvable = {a[0] for o, street, cost, members in options
                      for a in members}
        draw(os.path.join(out, "map.png"), edges, arcs, improvable, nodes,
             node_ids, population, interest)
    with open(os.path.join(out, "instance.json"), "w") as f:
        json.dump({"options": {"csv_file": "options.csv"}, "cases": cases,
                   "criterion": criterion}, f, indent=4)
        f.write("\n")
    with open(os.path.join(out, "bike.json"), "w") as f:
        json.dump({"config": config, "osm_date": OSM_DATE,
                   "summary": summary}, f, indent=4, ensure_ascii=False)
        f.write("\n")
    print(name, json.dumps(summary), flush=True)


def with_squirrels(config, cache, out, node_ids, edges, population, interest,
                   arcs, sections, summary):
    """Adds to the bike network the squirrels of the same rectangle, on the
    cells of landscape.py, and couples them: see the top of the file. Writes
    the options and the files of the squirrels, completes the summary, and
    returns the case of the squirrels and the criterion."""
    parameters = config["squirrel"]
    land = landscape.survey({**parameters, "bbox": config["bbox"]}, cache)
    cells, col = land.cells, land.col
    base_p = land.probability(land.area)
    best_p = base_p.copy()
    lines = edges.geometry.to_crs(2154)
    tree = shapely.STRtree(land.polygons)
    road = land.area[:, col["street"]] + land.area[:, col["highway"]]
    options = []  # (id, kind, cost, street, length, cells)
    vertex_gains, arc_gains = [], []
    for o, street, length, members in sections:
        # each street once, although it has an arc per direction
        once = {(min(a[1], a[2]), max(a[1], a[2]), round(a[3], 1)): a
                for a in members}
        strip = {}  # cell -> area of the planted strip, m2
        for a in once.values():
            line = lines.loc[(a[1], a[2], int(a[0].rsplit("_", 1)[1]))]
            for c in tree.query(line, predicate="intersects"):
                strip[c] = strip.get(c, 0.0) + parameters["strip_width"] * \
                    shapely.intersection(line, land.polygons[c]).length
        along = sorted(strip)
        # the strip replaces road surface: no more than the cell has
        planted = np.minimum([strip[c] for c in along], road[along])
        improved = land.area[along].copy()
        streets = np.minimum(planted, improved[:, col["street"]])
        improved[:, col["street"]] -= streets
        improved[:, col["highway"]] -= planted - streets
        improved[:, col["woody"]] += planted
        for c, p, gain in zip(along, land.probability(improved),
                              planted / 1e4):
            if p > base_p[c]:
                best_p[c] = max(best_p[c], p)
                arc_gains += [(f"{cells[c]}_in", o, math.sqrt(p)),
                              (f"{cells[c]}_out", o, math.sqrt(p))]
            # in a habitat cell only: areas add up when several lanes run
            # through it, a change of status would be counted once per lane
            if land.quality[c] > 0 and gain > 0:
                # as the file will hold it: the criterion uses these gains
                vertex_gains.append((c, o, float(f"{gain:.6g}")))
        options.append((o, "lane", parameters["lane_cost"] * length +
                        parameters["strip_cost"] * planted.sum(), street,
                        length, along))
    rope = parameters["rope_bridge"]
    crossed = {}
    for i, j in landscape.crossing_sites(land, rope["spacing"],
                                         rope.get("max_sites", 10 ** 9)):
        o = f"rope_{cells[i]}_{cells[j]}"
        options.append((o, "rope_bridge", rope["cost"], "", 0.0, [i, j]))
        for a, b in ((i, j), (j, i)):
            arc_gains.append((f"{cells[a]}_{cells[b]}", o,
                              rope["probability"]))
            crossed[a, b] = rope["probability"]

    # ---- criterion: the lower of the two shares of the largest gain ---
    index = {n: i for i, n in enumerate(node_ids)}

    def bike_score(alpha):
        steps = [[] for _ in node_ids]
        for a in arcs:
            steps[index[a[1]]].append(
                (index[a[2]], math.exp(-a[3] / alpha(a))))
        return landscape.connectivity(population, interest, steps)

    full_quality = land.quality.copy()
    for c, o, gain in vertex_gains:
        full_quality[c] += gain
    scores = {
        "bike": (bike_score(lambda a: a[4]),
                 bike_score(lambda a: LANE_ALPHA if a[5] else a[4])),
        "squirrel": (
            landscape.connectivity(land.quality, land.quality, landscape.moves(
                land, base_p, landscape.BARRIER)),
            landscape.connectivity(full_quality, full_quality, landscape.moves(
                land, best_p, landscape.BARRIER, crossed))),
    }
    criterion = {"operation": "min", "values": [
        {"operation": "sum", "values": [
            [1 / (full - initial), case], 1 - initial / (full - initial)]}
        for case, (initial, full) in scores.items()]}

    # ---- files --------------------------------------------------------
    write(out, "options.csv", "id,cost", ((o[0], f"{o[2]:.6g}")
                                          for o in options))
    write(out, "options_details.csv", "id,kind,cost,street,length,cells",
          ((o, kind, f"{cost:.6g}", '"' + street.replace('"', "'") + '"',
            f"{length:.1f}", " ".join(cells[c] for c in along))
           for o, kind, cost, street, length, along in options))
    landscape.write_cells(out, land, {"probability": base_p})

    def network(ax):
        from matplotlib.collections import LineCollection
        improvable = {a[0] for a in arcs if a[5]}
        for keep, colour, width in (
                (lambda a: a[8], (0.1, 0.35, 0.9), 1.6),
                (lambda a: a[0] in improvable, (0.85, 0.1, 0.1), 1.6)):
            ax.add_collection(LineCollection(
                [np.asarray(lines.loc[(a[1], a[2],
                                       int(a[0].rsplit("_", 1)[1]))].coords)
                 for a in arcs if a[1] < a[2] and keep(a)],
                colors=[colour], linewidths=width, zorder=2))
        sites = np.array([(land.centres[i] + land.centres[j]) / 2
                          for i, j in crossed if i < j])
        if len(sites):
            ax.scatter(sites[:, 0], sites[:, 1], s=30, c="yellow",
                       edgecolors="black", linewidths=0.8, zorder=3)

    landscape.draw(os.path.join(out, "map.png"), land.cover, land.bounds,
                   land.polygons, land.quality, [], land.centres,
                   land.barrier, network)
    summary.update({
        "cells": len(cells), "habitat_cells": int((land.quality > 0).sum()),
        "options": {kind: sum(1 for o in options if o[1] == kind)
                    for kind in ("lane", "rope_bridge")},
        "total_cost": round(sum(o[2] for o in options), 1),
        "initial_and_largest_scores": scores,
    })
    return {"squirrel": landscape.write_case(
        out, "squirrel_", land, base_p, landscape.BARRIER, vertex_gains,
        arc_gains)}, criterion


def draw(path, edges, arcs, improvable, nodes, node_ids, population,
         interest):
    """Streets with a lane (blue), improvable (red) and others (grey);
    population (orange) and points of interest (dark blue) per vertex."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    lane = {a[0] for a in arcs if a[8]}
    lines = {"other": [], "improvable": [], "lane": []}
    for (u, v, k), geometry in edges.geometry.items():
        if u > v:
            continue
        arc = f"{u}_{v}_{k}"
        kind = "lane" if arc in lane else \
            "improvable" if arc in improvable else "other"
        lines[kind].append(np.asarray(geometry.coords))
    x = nodes.loc[node_ids, "x"].values
    y = nodes.loc[node_ids, "y"].values
    ratio = (y.max() - y.min()) / ((x.max() - x.min()) *
                                   math.cos(math.radians(y.mean())))
    fig, ax = plt.subplots(figsize=(8, 8 * ratio), dpi=100)
    for kind, colour, width in (("other", (0.72, 0.72, 0.72), 0.6),
                                ("improvable", (0.85, 0.2, 0.15), 1.4),
                                ("lane", (0.1, 0.35, 0.9), 1.6)):
        ax.add_collection(LineCollection(lines[kind], colors=[colour],
                                         linewidths=width))
    keep = population > 0
    ax.scatter(x[keep], y[keep], s=population[keep] / 12, c="orange",
               alpha=0.45, linewidths=0, zorder=3)
    keep = interest > 0
    ax.scatter(x[keep], y[keep], s=6 + 6 * interest[keep], c="navy",
               alpha=0.7, linewidths=0, zorder=4)
    ax.set_xlim(x.min(), x.max())
    ax.set_ylim(y.min(), y.max())
    ax.set_aspect(1 / math.cos(math.radians(y.mean())))
    ax.set_axis_off()
    fig.subplots_adjust(0, 0, 1, 1)
    fig.savefig(path)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("config", help="bikes.json")
    parser.add_argument("names", nargs="*", help="instances to build "
                        "(default: all)")
    here = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument("--cache", default=os.path.join(here, "..", "cache"),
                        help="directory of the downloaded data (default: "
                        "../cache)")
    parser.add_argument("--out", default=os.path.join(here, "..",
                                                      "instances"),
                        help="directory of the instances (default: "
                        "../instances)")
    args = parser.parse_args()
    with open(args.config) as f:
        configs = json.load(f)
    unknown = set(args.names) - set(configs)
    if unknown:
        sys.exit(f"unknown instances: {', '.join(sorted(unknown))}")
    for name_, config_ in configs.items():
        if not args.names or name_ in args.names:
            build(name_, config_, args.cache, args.out)
