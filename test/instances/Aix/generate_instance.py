from PIL import Image
import csv
import itertools as it
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import os
import pathlib

# ################################ INPUT DATA #################################

root_path = pathlib.Path(__file__).parent.resolve()

tif_image = Image.open(root_path / "map.tiff")
tif_matrix = np.array(tif_image)

# instance_name = "instance_70x70"

instance_name = "instance_30x30"
tif_matrix = tif_matrix[0:30, 11:41]

# instance_name = "instance_40x40"
# tif_matrix = tif_matrix[0:40, 0:40]

# print(tif_matrix)


# return the type of a cell as a function of its value
def cell_type(v):
    v = int(v)
    if v == 1:
        return "habitat"
    if v == 10:
        return "field"
    if v == 100:
        return "road"
    if 100 < v < 200:
        return "wildlife crossing"
    if 200 < v:
        return "land acquisition"
    raise ValueError(f"Unknown cell value '{v}'")


# return the option_id of a cell as a function of its value
def cell_option_id(v):
    v = int(v)
    if 100 < v < 200:
        return str(v)
    if 200 < v:
        return str(v)
    return None


colors = {
    "habitat": (64, 192, 64),
    "field": (192, 255, 0),
    "road": (255, 128, 0),
    "wildlife crossing": (64, 128, 255),
    "land acquisition": (192, 128, 192),
}
quality_map = {
    "habitat": 1,
    "field": 0,
    "road": 0,
    "wildlife crossing": 0,
    "land acquisition": 0,
}
probability_map = {
    "habitat": 1,
    "field": 0.98,
    "road": 0.66,
    "wildlife crossing": 0.8,
    "land acquisition": 0.98,
}
quality_gain_map = {"wildlife crossing": 0, "land acquisition": 1}
improved_probability_map = {"wildlife crossing": 0.95, "land acquisition": 0.995}
cell_cost = {"wildlife crossing": 1, "land acquisition": 0.2}


# ################################ PRINT IMAGE ################################

os.makedirs(root_path / instance_name, exist_ok=True)

(width, height) = tif_matrix.shape
img = Image.new("RGB", (width, height))
pixels = img.load()
for idx, value in np.ndenumerate(tif_matrix):
    (y, x) = idx
    pixels[x, y] = colors[cell_type(value)]

fig, ax = plt.subplots(figsize=(16, 16))
ax.imshow(img)

legend_patches = [
    mpatches.Patch(color=np.array(color) / 255, label=color_id, ec=(0.5, 0.5, 0.5))
    for color_id, color in colors.items()
]
ax.legend(
    handles=legend_patches,
    fontsize=40,
    loc="lower left",
    framealpha=0.9,
    bbox_to_anchor=(1, 0),
)
ax.set_xticks([])
ax.set_yticks([])

plt.tight_layout()
# plt.show()
plt.savefig(root_path / f"{instance_name}/{instance_name}.png")

# ################################ PRINT FILES ################################


def readCSVDict(file_name, id_column="id", delimiter=","):
    file = csv.DictReader(open(file_name), delimiter=delimiter)
    d = {}
    for row in file:
        d[row[id_column]] = row
    return d


def writeCSV(file_name, columns):
    file_path = file_name
    file = open(file_path, "w")
    file.write(columns + "\n")
    return file


vertices_csv = writeCSV(root_path / f"{instance_name}/vertices.csv", "id,quality,x,y")
arcs_csv = writeCSV(root_path / f"{instance_name}/arcs.csv", "id,from,to,probability")


def add_vertex(id, quality, x, y):
    vertices_csv.write(f"{id},{quality},{x},{y}\n")


def add_arc(arc_id, source_id, target_id, probability):
    arcs_csv.write(f"{arc_id},{source_id},{target_id},{probability}\n")


offset_for_display = 0.33

(width, height) = tif_matrix.shape
for idx, value in np.ndenumerate(tif_matrix):
    (y, x) = idx
    id = f"{x}:{y}"
    quality = quality_map[cell_type(value)]
    probability = probability_map[cell_type(value)]

    add_vertex(id, quality, x, y)
    add_vertex(f"{id}_in", 0, x - offset_for_display, y - offset_for_display)
    add_vertex(f"{id}_out", 0, x + offset_for_display, y + offset_for_display)

    add_arc(f"{id}_in_arc", f"{id}_in", id, math.sqrt(probability))
    add_arc(f"{id}_out_arc", id, f"{id}_out", math.sqrt(probability))

    for to_x, to_y in [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]:
        if 0 <= to_x < width and 0 <= to_y < height:
            to_id = f"{to_x}:{to_y}"
            arc_id = f"{id}_to_{to_id}"
            add_arc(arc_id, f"{id}_out", f"{to_id}_in", 1)

vertices_csv.close()
arcs_csv.close()

options_csv = writeCSV(root_path / f"{instance_name}/options.csv", "id,cost")
vertex_options_csv = writeCSV(
    root_path / f"{instance_name}/vertices_improvements.csv",
    "vertex_id,option_id,quality_gain",
)
arc_options_csv = writeCSV(
    root_path / f"{instance_name}/arcs_improvements.csv",
    "arc_id,option_id,improved_probability",
)


def add_option(option_id, cost):
    options_csv.write(f"{option_id},{cost}\n")


def add_vertex_option(vertex_id, option_id, quality_gain):
    vertex_options_csv.write(f"{vertex_id},{option_id},{quality_gain}\n")


def add_arc_option(arc_id, option_id, improved_prob):
    arc_options_csv.write(f"{arc_id},{option_id},{improved_prob}\n")


options = {}

for idx, value in np.ndenumerate(tif_matrix):
    (y, x) = idx
    id = f"{x}:{y}"
    value = int(value)
    cell_t = cell_type(value)
    cell_option = cell_option_id(value)
    if cell_option is None:
        continue

    if cell_option not in options:
        options[cell_option] = 0
    options[cell_option] += cell_cost[cell_t]

    quality_gain = quality_gain_map[cell_t]
    improved_prob = improved_probability_map[cell_t]

    if quality_gain > 0:
        add_vertex_option(id, value, quality_gain)

    if improved_prob > probability_map[cell_t]:
        add_arc_option(f"{id}_in_arc", value, math.sqrt(improved_prob))
        add_arc_option(f"{id}_out_arc", value, math.sqrt(improved_prob))

for id, cost in options.items():
    add_option(id, cost)

options_csv.close()
vertex_options_csv.close()
arc_options_csv.close()


instance_file = open(root_path / f"{instance_name}/instance.json", "w")
instance_file.write(
    """{
    "options": {
        "csv_file": "options.csv"
    },
    "cases": {
        "squirrel": {
            "vertices": {
                "csv_file": "vertices.csv"
            },
            "arcs": {
                "csv_file": "arcs.csv"
            },
            "vertices_improvements": {
                "csv_file": "vertices_improvements.csv"
            },
            "arcs_improvements": {
                "csv_file": "arcs_improvements.csv"
            }
        }
    },
    "criterion": "squirrel"
}"""
)
instance_file.close()
