#!/usr/bin/env python3
"""Instances of GECOT built from the BD TOPO of IGN (Licence Ouverte 2.0).

    landscape.py landscapes.json                 every instance of the file
    landscape.py landscapes.json urban-greening  this one only

The landscape of an instance is a rectangle given in longitude and latitude,
cut into the hexagonal cells of the H3 grid. landscapes.json gives, per
instance, the department, the rectangle, the H3 resolution and the kinds of
options with their parameters; the model below is the same for all.

Land cover. The BD TOPO layers are painted on a 2 m raster, each over the
previous ones: woody vegetation (zone_de_vegetation but orchards and
vineyards, and haie), orchards and vineyards, water (surface_hydrographique),
railways, paved roads (streets, then main roads) and buildings; what is left
is open land. Tunnels are not painted. A cell gets the share of each cover.

Graph. A cell h has three vertices h_in, h and h_out, and arcs h_in -> h and
h -> h_out of probability sqrt(p_h), with

    p_h = 0.5 ^ (D / theta_h)

D being the distance between the centres of adjacent cells and theta_h the
mean over the covers of the cell of the distance an individual crosses with
probability one half (THETA below). An arc u_out -> v_in joins adjacent cells,
of probability 1, or BARRIER[class] if the segment between their centres
crosses a main road or a railway at ground level. The cells at least
HABITAT_SHARE wooded are the habitat: their vertex h has their wooded area, in
hectares, as source and target quality; the other cells have quality 0.

Options.
- greening: in the cells of a block (the cells of the same parent cell at
  "group_resolution"), a share of the street area becomes woody. It raises
  p_h, and the qualities of h to the new wooded area if the cell is, or
  becomes, habitat.
- planting: the same with a share of the open land of the cells that are
  mostly open, not habitat yet and not in a greening option.
- crossing: a wildlife crossing between two adjacent cells that a barrier
  separates; it raises the two arcs between them to probability 1. The sites
  are the pairs with the most wooded area around, at least "spacing" metres
  apart.
- rope_bridge: at the same sites, a cheaper crossing that only the species of
  its "species" can use, with its "probability".

Species. With "species", the instance has one case per species, on the same
cells and habitat, with the same options. A species has its "dispersal",
which multiplies the distances of THETA, and its "barrier" probabilities. The
criterion is then the lowest score of the species, each divided by its
initial score: 1 without any option.
Costs are in thousands of euros, from the unit costs of landscapes.json
(orders of magnitude, not estimates).

Besides the instance, the directory gets cells.csv (centre, cover shares,
quality and probability of each cell), options_details.csv (kind, cost and
cells of each option) and map.png.
"""
import argparse
import glob
import heapq
import json
import math
import os
import sys
import types

import geopandas as gpd
import h3
import numpy as np
import pandas as pd
import shapely
from pyproj import Transformer
from rasterio import features
from rasterio.transform import from_origin
from shapely.geometry import Polygon

# distance (m) crossed with probability 1/2, per land cover
THETA = {"open": 800, "woody": 3000, "crop": 1500, "water": 50, "rail": 100,
         "street": 100, "highway": 100, "built": 0}
COVERS = list(THETA)  # painted in this order
# probability of an arc between two cells across a barrier
BARRIER = {"major": 0.05, "secondary": 0.3}
HABITAT_SHARE = 0.3
PIXEL = 2.0
MARGIN = 300.0

CROP_NATURES = {"Verger", "Vigne"}
STREET_NATURES = {"Route à 1 chaussée", "Rond-point"}
HIGHWAY_NATURES = {"Type autoroutier", "Route à 2 chaussées", "Bretelle"}
DEFAULT_WIDTH = {"Type autoroutier": 10.0, "Route à 2 chaussées": 7.0}
RAIL_NATURES = {"LGV", "Voie ferrée principale", "Voie de service", "Tramway"}

EDITION = "2025-09-15"
URL = ("https://data.geopf.fr/telechargement/download/BDTOPO/{stem}/"
       "{stem}.7z")


def bdtopo(department, cache):
    """Path of the GeoPackage of the department, downloaded if missing."""
    stem = f"BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_D{department:03d}_{EDITION}"
    pattern = os.path.join(cache, stem, "**", "*.gpkg")
    found = glob.glob(pattern, recursive=True)
    if not found:
        import py7zr
        import requests
        os.makedirs(cache, exist_ok=True)
        archive = os.path.join(cache, stem + ".7z")
        if not os.path.exists(archive):
            print(f"downloading {stem}.7z", flush=True)
            with requests.get(URL.format(stem=stem), stream=True) as r:
                r.raise_for_status()
                with open(archive, "wb") as f:
                    for chunk in r.iter_content(chunk_size=1 << 20):
                        f.write(chunk)
        print(f"extracting {archive}", flush=True)
        with py7zr.SevenZipFile(archive, "r") as z:
            z.extractall(path=cache)
        found = glob.glob(pattern, recursive=True)
    return found[0]


def on_ground(frame):
    """Without the tunnels (negative position); bridges stay."""
    # numbers as text, and a few words ("Gué ou radier"), at ground level
    position = pd.to_numeric(frame["position_par_rapport_au_sol"],
                             errors="coerce").fillna(0)
    return frame[position >= 0], frame[position == 0]


def survey(config, cache):
    """The cells of the rectangle, their land cover, habitat and barriers."""
    gpkg = bdtopo(config["department"], cache)
    resolution = config["resolution"]
    west, south, east, north = config["bbox"]

    # ---- cells --------------------------------------------------------
    to_l93 = Transformer.from_crs(4326, 2154, always_xy=True)
    cells = sorted(h3.polygon_to_cells(h3.LatLngPoly(
        [(south, west), (south, east), (north, east), (north, west)]),
        resolution))
    index = {c: i for i, c in enumerate(cells)}
    centres = np.array([to_l93.transform(lng, lat) for lat, lng in
                        map(h3.cell_to_latlng, cells)])
    polygons = [Polygon([to_l93.transform(lng, lat) for lat, lng in
                         h3.cell_to_boundary(c)]) for c in cells]
    pairs = sorted({(index[c], index[d]) for c in cells
                    for d in h3.grid_disk(c, 1) if d != c and d in index})
    spacing = float(np.mean([math.dist(centres[i], centres[j])
                             for i, j in pairs]))

    # ---- land cover ---------------------------------------------------
    xmin, ymin, xmax, ymax = shapely.total_bounds(polygons)
    xmin, ymin, xmax, ymax = (xmin - MARGIN, ymin - MARGIN, xmax + MARGIN,
                              ymax + MARGIN)
    shape = (math.ceil((ymax - ymin) / PIXEL),
             math.ceil((xmax - xmin) / PIXEL))
    transform = from_origin(xmin, ymax, PIXEL, PIXEL)

    def read(layer, columns):
        frame = gpd.read_file(gpkg, layer=layer, columns=columns,
                              bbox=(xmin, ymin, xmax, ymax))
        frame = frame[~frame.geometry.is_empty & frame.geometry.notna()]
        frame["geometry"] = shapely.force_2d(frame.geometry.values)
        return frame

    vegetation = read("zone_de_vegetation", ["nature"])
    hedges = read("haie", [])
    water = read("surface_hydrographique", [])
    buildings = read("batiment", [])
    roads = read("troncon_de_route", [
        "nature", "importance", "largeur_de_chaussee",
        "position_par_rapport_au_sol"])
    rails = read("troncon_de_voie_ferree", [
        "nature", "nombre_de_voies", "position_par_rapport_au_sol"])
    rails = rails[rails["nature"].isin(RAIL_NATURES)]
    roads, roads_at_grade = on_ground(roads)
    rails, rails_at_grade = on_ground(rails)

    def width(frame):
        given = frame["largeur_de_chaussee"].astype(float).fillna(0)
        default = frame["nature"].map(DEFAULT_WIDTH).fillna(5.0)
        return np.where(given > 0, given, default)

    streets = roads[roads["nature"].isin(STREET_NATURES)]
    highways = roads[roads["nature"].isin(HIGHWAY_NATURES)]
    is_crop = vegetation["nature"].isin(CROP_NATURES)
    tracks = rails["nombre_de_voies"].astype(float).fillna(1).clip(lower=1)
    layers = {
        "woody": list(vegetation[~is_crop].geometry) +
        list(hedges.geometry.buffer(2.5)),
        "crop": list(vegetation[is_crop].geometry),
        "water": list(water.geometry),
        "rail": list(rails.geometry.buffer(2.5 * tracks.values)),
        "street": list(streets.geometry.buffer(width(streets) / 2)),
        "highway": list(highways.geometry.buffer(width(highways) / 2)),
        "built": list(buildings.geometry),
    }
    cover = np.zeros(shape, dtype=np.uint8)
    for code, name_ in enumerate(COVERS):
        if layers.get(name_):
            features.rasterize(((g, code) for g in layers[name_]), out=cover,
                               transform=transform)
    cell_of = features.rasterize(
        ((p, i + 1) for i, p in enumerate(polygons)), out_shape=shape,
        transform=transform, fill=0, dtype=np.int32)
    inside = cell_of > 0
    counts = np.bincount(
        (cell_of[inside] - 1) * len(COVERS) + cover[inside],
        minlength=len(cells) * len(COVERS)).reshape(len(cells), len(COVERS))
    area = counts * PIXEL * PIXEL  # m2 per cell and cover
    cell_area = area.sum(axis=1)
    col = {name_: k for k, name_ in enumerate(COVERS)}
    theta = np.array([THETA[c] for c in COVERS], dtype=float)

    def probability(areas, dispersal=1.0):
        mean_theta = dispersal * (areas @ theta) / areas.sum(axis=1)
        with np.errstate(divide="ignore"):
            return np.where(mean_theta > 0, 0.5 ** (spacing / mean_theta), 0.0)

    woody = area[:, col["woody"]]
    quality = np.where(woody >= HABITAT_SHARE * cell_area, woody / 1e4, 0.0)

    # ---- barriers -----------------------------------------------------
    major = list(roads_at_grade[roads_at_grade["nature"] ==
                                "Type autoroutier"].geometry) + \
        list(rails_at_grade[rails_at_grade["nature"] == "LGV"].geometry)
    main_road = roads_at_grade["nature"].isin(
        {"Route à 2 chaussées", "Bretelle"}) | (
        roads_at_grade["nature"].isin(STREET_NATURES) &
        roads_at_grade["importance"].isin(["1", "2"]))
    secondary = list(roads_at_grade[main_road].geometry) + list(
        rails_at_grade[rails_at_grade["nature"] ==
                       "Voie ferrée principale"].geometry)
    segments = shapely.linestrings(
        [[centres[i], centres[j]] for i, j in pairs])
    barrier = {}  # pair -> class
    for klass, lines in (("secondary", secondary), ("major", major)):
        if lines:
            tree = shapely.STRtree(lines)
            for k in np.unique(tree.query(segments,
                                          predicate="intersects")[0]):
                barrier[pairs[k]] = klass
    return types.SimpleNamespace(
        cells=cells, index=index, centres=centres, polygons=polygons,
        pairs=pairs, spacing=spacing, cover=cover,
        bounds=(xmin, ymin, xmax, ymax), area=area, cell_area=cell_area,
        col=col, woody=woody, quality=quality, barrier=barrier,
        probability=probability)


def crossing_sites(land, spacing, max_sites):
    """The pairs of adjacent cells that a barrier separates, those with the
    most wooded area around first, at least `spacing` metres apart."""
    ring = [[land.index[d] for d in h3.grid_disk(c, 1) if d in land.index]
            for c in land.cells]
    around = np.array([land.woody[r].sum() for r in ring])
    candidates = sorted(
        ((min(around[i], around[j]), i, j) for (i, j) in land.barrier
         if i < j and min(around[i], around[j]) > 0), reverse=True)
    sites, middles = [], []
    for score, i, j in candidates:
        middle = (land.centres[i] + land.centres[j]) / 2
        if all(math.dist(middle, m) >= spacing for m in middles):
            middles.append(middle)
            sites.append((i, j))
            if len(sites) == max_sites:
                break
    return sites


def write(out, file, header, rows):
    with open(os.path.join(out, file), "w") as f:
        f.write(header + "\n")
        f.writelines(",".join(map(str, row)) + "\n" for row in rows)


def write_case(out, prefix, land, base_p, cut, vertex_gains, arc_gains):
    """Writes the files of a case on the cells of land, and returns its
    entry of instance.json. cut gives the probability of an arc across a
    barrier of each class."""
    cells = land.cells
    write(out, prefix + "vertices.csv", "id,source_quality,target_quality",
          (row for c, q in zip(cells, land.quality) for row in (
              (f"{c}_in", 0, 0), (c, f"{q:.6g}", f"{q:.6g}"),
              (f"{c}_out", 0, 0))))
    root = np.sqrt(base_p)
    write(out, prefix + "arcs.csv", "id,from,to,probability",
          [(f"{c}_in", f"{c}_in", c, f"{r:.12g}")
           for c, r in zip(cells, root)] +
          [(f"{c}_out", c, f"{c}_out", f"{r:.12g}")
           for c, r in zip(cells, root)] +
          [(f"{cells[i]}_{cells[j]}", f"{cells[i]}_out", f"{cells[j]}_in",
            cut[land.barrier[i, j]] if (i, j) in land.barrier else 1)
           for i, j in land.pairs])
    write(out, prefix + "vertices_improvements.csv",
          "vertex_id,option_id,source_quality_gain,target_quality_gain",
          ((cells[i], o, f"{gain:.6g}", f"{gain:.6g}")
           for i, o, gain in vertex_gains))
    write(out, prefix + "arcs_improvements.csv",
          "arc_id,option_id,improved_probability",
          ((a, o, f"{p:.12g}") for a, o, p in arc_gains))
    return {
        "vertices": {"csv_file": prefix + "vertices.csv"},
        "arcs": {"csv_file": prefix + "arcs.csv"},
        "vertices_improvements": {
            "csv_file": prefix + "vertices_improvements.csv"},
        "arcs_improvements": {"csv_file": prefix + "arcs_improvements.csv"},
    }


def write_cells(out, land, probabilities):
    """cells.csv: centre, cover shares and quality of each cell, then a
    column per entry of probabilities."""
    write(out, "cells.csv", "id,longitude,latitude," + ",".join(COVERS) +
          ",quality," + ",".join(probabilities),
          ((c, *(f"{v:.6f}" for v in h3.cell_to_latlng(c)[::-1]),
            *(f"{v:.4f}" for v in land.area[i] / land.cell_area[i]),
            f"{land.quality[i]:.6g}",
            *(f"{p[i]:.6g}" for p in probabilities.values()))
           for i, c in enumerate(land.cells)))


def moves(land, base_p, cut, crossed={}):
    """Per cell, the (cell, probability) of the steps to its neighbours;
    crossed gives the probability of the pairs with a crossing."""
    root = np.sqrt(base_p)
    steps = [[] for _ in land.cells]
    for i, j in land.pairs:
        steps[i].append((j, root[i] * root[j] * crossed.get((i, j), (
            cut[land.barrier[i, j]] if (i, j) in land.barrier else 1))))
    return steps


def build(name, config, cache, out_root):
    out = os.path.join(out_root, name)
    os.makedirs(out, exist_ok=True)
    land = survey(config, cache)
    resolution = config["resolution"]
    cells, centres, polygons = land.cells, land.centres, land.polygons
    area, cell_area, col = land.area, land.cell_area, land.col
    quality, barrier, probability = land.quality, land.barrier, land.probability
    species = config.get("species", {"landscape": {}})
    dispersal = {s: d.get("dispersal", 1.0) for s, d in species.items()}
    base_p = {s: probability(area, dispersal[s]) for s in species}

    # ---- options ------------------------------------------------------
    options = []  # (id, kind, cost, cells)
    vertex_gains = []  # (cell, option, gain)
    arc_gains = {s: [] for s in species}  # (arc id, option, probability)

    def conversion(kind, parameters, source, eligible):
        """Per block, a share of the `source` cover of its eligible cells
        becomes woody."""
        blocks = {}
        for i, c in enumerate(cells):
            if eligible[i]:
                parent = h3.cell_to_parent(c, parameters.get(
                    "group_resolution", resolution))
                blocks.setdefault(parent, []).append(i)
        for parent in sorted(blocks):
            members = blocks[parent]
            planted = parameters["share"] * area[members, col[source]]
            option = f"{kind}_{parent}"
            options.append((option, kind, parameters["unit_cost"] *
                            planted.sum(), members))
            improved = area[members].copy()
            improved[:, col[source]] -= planted
            improved[:, col["woody"]] += planted
            wooded = improved[:, col["woody"]]
            gains = np.where(wooded >= HABITAT_SHARE * cell_area[members],
                             wooded / 1e4, 0.0) - quality[members]
            vertex_gains.extend((i, option, gain)
                                for i, gain in zip(members, gains) if gain > 0)
            for s in species:
                for i, p in zip(members, probability(improved, dispersal[s])):
                    if p > base_p[s][i]:
                        arc_gains[s] += [
                            (f"{cells[i]}_in", option, math.sqrt(p)),
                            (f"{cells[i]}_out", option, math.sqrt(p))]

    kinds = config["options"]
    greened = np.zeros(len(cells), dtype=bool)
    if "greening" in kinds:
        g = kinds["greening"]
        greened = area[:, col["street"]] >= g["min_share"] * cell_area
        conversion("greening", g, "street", greened)
    if "planting" in kinds:
        g = kinds["planting"]
        # not the greened cells: the quality gains of two options add up,
        # which would count the wooded area of such a cell twice
        conversion("planting", g, "open",
                   (area[:, col["open"]] >= g["min_share"] * cell_area) &
                   (quality == 0) & ~greened)
    if "crossing" in kinds:
        g = kinds["crossing"]
        for i, j in crossing_sites(land, g["spacing"],
                                   g.get("max_sites", 10 ** 9)):
            both = [f"{cells[i]}_{cells[j]}", f"{cells[j]}_{cells[i]}"]
            option = f"crossing_{both[0]}"
            options.append((option, "crossing", g["cost"][barrier[i, j]],
                            [i, j]))
            for s in species:
                arc_gains[s] += [(arc, option, 1.0) for arc in both]
            if "rope_bridge" in kinds:
                rope = kinds["rope_bridge"]
                option = f"rope_{both[0]}"
                options.append((option, "rope_bridge", rope["cost"], [i, j]))
                for s in rope["species"]:
                    arc_gains[s] += [(arc, option, rope["probability"])
                                     for arc in both]

    # ---- files --------------------------------------------------------
    write(out, "options.csv", "id,cost",
          ((o, f"{cost:.6g}") for o, kind, cost, members in options))
    multi = "species" in config
    cut = {s: d.get("barrier", BARRIER) for s, d in species.items()}
    cases = {s: write_case(out, f"{s}_" if multi else "", land, base_p[s],
                           cut[s], vertex_gains, arc_gains[s])
             for s in species}
    criterion = "landscape"
    if multi:
        # the lowest score of the species, each divided by its initial score
        initial = {s: connectivity(quality, quality,
                                   moves(land, base_p[s], cut[s]))
                   for s in species}
        criterion = {"operation": "min",
                     "values": [[1 / initial[s], s] for s in species]}
    with open(os.path.join(out, "instance.json"), "w") as f:
        json.dump({"options": {"csv_file": "options.csv"}, "cases": cases,
                   "criterion": criterion}, f, indent=4)
        f.write("\n")
    write_cells(out, land, {f"probability_{s}" if multi else "probability":
                            base_p[s] for s in species})
    write(out, "options_details.csv", "id,kind,cost,cells",
          ((o, kind, f"{cost:.6g}", " ".join(cells[i] for i in members))
           for o, kind, cost, members in options))

    draw(os.path.join(out, "map.png"), land.cover, land.bounds, polygons,
         quality, options, centres, barrier)
    by_kind = {}
    for o, kind, cost, members in options:
        n, total = by_kind.get(kind, (0, 0.0))
        by_kind[kind] = (n + 1, total + cost)
    summary = {
        "cells": len(cells), "cell_spacing_m": round(land.spacing, 1),
        "area_km2": round(cell_area.sum() / 1e6, 1),
        "habitat_cells": int((quality > 0).sum()),
        "cells_options_turn_into_habitat": len(
            {i for i, o, gain in vertex_gains if quality[i] == 0}),
        "cover": {c: round(float(area[:, col[c]].sum() / cell_area.sum()), 3)
                  for c in COVERS},
        "barrier_pairs": {k: sum(1 for v in barrier.values() if v == k) // 2
                          for k in BARRIER},
        "options": {k: {"count": n, "total_cost": round(t, 1)}
                    for k, (n, t) in by_kind.items()},
        "total_cost": round(sum(o[2] for o in options), 1),
    }
    if multi:
        summary["initial_scores"] = initial
    with open(os.path.join(out, "landscape.json"), "w") as f:
        json.dump({"config": config, "summary": summary}, f, indent=4,
                  ensure_ascii=False)
        f.write("\n")
    print(name, json.dumps(summary), flush=True)


def connectivity(source_quality, target_quality, moves):
    """The score of a case: the sum over the pairs (s, t) of vertices of
    source_quality[s] target_quality[t] P(s, t), P being the probability of
    the most reliable path and moves[u] the (v, probability) of the arcs out
    of u."""
    total = 0.0
    for source in np.flatnonzero(source_quality):
        best = {source: 1.0}
        heap = [(-1.0, source)]
        settled = set()
        while heap:
            p, u = heapq.heappop(heap)
            if u in settled:
                continue
            settled.add(u)
            total += source_quality[source] * target_quality[u] * -p
            for v, step in moves[u]:
                if -p * step > best.get(v, 0.0):
                    best[v] = -p * step
                    heapq.heappush(heap, (p * step, v))
    return total


def draw(path, cover, bounds, polygons, quality, options, centres, barrier,
         extra=None):
    """The land cover, the habitat cells (dark green outline), the cells of
    the greening (blue) and planting (orange) options, the arcs that a
    barrier cuts (red: major, pink: secondary) and the crossing sites
    (yellow dots); extra(ax) draws over them."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection, PolyCollection

    palette = np.array([
        (0.93, 0.90, 0.78),  # open
        (0.30, 0.62, 0.32),  # woody
        (0.76, 0.84, 0.50),  # crop
        (0.45, 0.68, 0.90),  # water
        (0.30, 0.25, 0.30),  # rail
        (0.80, 0.62, 0.62),  # street
        (0.62, 0.22, 0.22),  # highway
        (0.50, 0.50, 0.50),  # built
    ])
    xmin, ymin, xmax, ymax = bounds
    step = max(1, round(max(cover.shape) / 1100))
    fig, ax = plt.subplots(
        figsize=(8, 8 * (ymax - ymin) / (xmax - xmin)), dpi=100)
    ax.imshow(palette[cover[::step, ::step]], extent=(xmin, xmax, ymin, ymax),
              interpolation="nearest")
    rings = [np.asarray(p.exterior.coords) for p in polygons]
    ax.add_collection(PolyCollection(rings, facecolors="none",
                                     edgecolors=(1, 1, 1, 0.35),
                                     linewidths=0.3))
    for kind, colour in (("greening", (0.1, 0.25, 0.9)),
                         ("planting", (0.95, 0.5, 0.0))):
        members = [rings[i] for o, k, cost, cells_ in options if k == kind
                   for i in cells_]
        ax.add_collection(PolyCollection(members, facecolors="none",
                                         edgecolors=colour, linewidths=0.7))
    ax.add_collection(PolyCollection(
        [r for r, q in zip(rings, quality) if q > 0], facecolors="none",
        edgecolors=(0, 0.2, 0.05), linewidths=1.1))
    cut = [(i, j) for (i, j) in barrier if i < j]
    ax.add_collection(LineCollection(
        [(centres[i], centres[j]) for i, j in cut],
        colors=[(0.85, 0, 0) if barrier[i, j] == "major" else (1, 0.45, 0.6)
                for i, j in cut], linewidths=0.9))
    sites = np.array([(centres[cells_[0]] + centres[cells_[1]]) / 2
                      for o, k, cost, cells_ in options if k == "crossing"])
    if len(sites):
        ax.scatter(sites[:, 0], sites[:, 1], s=30, c="yellow",
                   edgecolors="black", linewidths=0.8, zorder=3)
    if extra:
        extra(ax)
    hull = shapely.total_bounds(polygons)
    ax.set_xlim(hull[0], hull[2])
    ax.set_ylim(hull[1], hull[3])
    ax.set_aspect("equal")
    ax.set_axis_off()
    fig.subplots_adjust(0, 0, 1, 1)
    fig.savefig(path)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("config", help="landscapes.json")
    parser.add_argument("names", nargs="*", help="instances to build "
                        "(default: all)")
    here = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument("--cache", default=os.path.join(here, "..", "cache"),
                        help="directory of the BD TOPO archives (default: "
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
