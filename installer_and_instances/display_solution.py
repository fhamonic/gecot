from PIL import Image
import json
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import sys

import input_data

with open(sys.argv[1]) as json_data:
    solution = json.load(json_data)["solution"]
    json_data.close()

solution_color = (0, 0, 0)

(width, height) = input_data.tif_matrix.shape
sol_img = Image.new("RGB", (width, height))
pixels = sol_img.load()
for idx, value in np.ndenumerate(input_data.tif_matrix):
    (y, x) = idx
    pixels[x, y] = input_data.colors[input_data.cell_type(value)]
    cell_option = input_data.cell_option_id(value)
    if cell_option is None:
        continue
    if solution[cell_option]:
        pixels[x, y] = solution_color

fig, ax = plt.subplots(figsize=(16, 16))
ax.imshow(sol_img)

sol_legend_patches = [
    mpatches.Patch(color=np.array(color) / 255, label=color_id, ec=(0.5, 0.5, 0.5))
    for color_id, color in input_data.colors.items()
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
plt.show()