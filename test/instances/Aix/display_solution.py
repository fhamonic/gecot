from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pathlib

root_path = pathlib.Path(__file__).parent.resolve()

tif_image = Image.open(root_path / "map.tiff")
tif_matrix = np.array(tif_image)

instance_name = "instance_40x40"
tif_matrix = tif_matrix[0:40, 0:40]

# instance_name = "instance_70x70"

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

solution = {
    "125": 0,
        "126": 0,
        "124": 1,
        "138": 1,
        "117": 0,
        "116": 0,
        "207": 1,
        "102": 1,
        "115": 0,
        "216": 1,
        "137": 0,
        "118": 0,
        "121": 0,
        "206": 1,
        "120": 0,
        "101": 0,
        "114": 1,
        "128": 0,
        "201": 1,
        "119": 0,
        "214": 1,
        "132": 0,
        "127": 0,
        "213": 1,
        "122": 0,
        "209": 1,
        "113": 1
}

solution_color = (0, 0, 0)

(width, height) = tif_matrix.shape
sol_img = Image.new("RGB", (width, height))
pixels = sol_img.load()
for idx, value in np.ndenumerate(tif_matrix):
    (y, x) = idx
    pixels[x, y] = colors[cell_type(value)]
    cell_option = cell_option_id(value)
    if cell_option is None:
        continue
    if solution[cell_option]:
        pixels[x, y] = solution_color

fig, ax = plt.subplots(figsize=(16, 16))
ax.imshow(sol_img)

sol_legend_patches = [
    mpatches.Patch(color=np.array(color) / 255, label=color_id, ec=(0.5, 0.5, 0.5))
    for color_id, color in colors.items()
] + [
    mpatches.Patch(
        color=np.array(solution_color) / 255, label="solution", ec=(0.5, 0.5, 0.5)
    )
]
ax.legend(
    handles=sol_legend_patches,
    fontsize=40,
    loc="lower left",
    framealpha=0.9,
    bbox_to_anchor=(1, 0),
)

ax.set_xticks([])
ax.set_yticks([])

plt.tight_layout()
plt.savefig(root_path / f"{instance_name}/{instance_name}_solution.png")
