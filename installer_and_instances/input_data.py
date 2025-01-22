from PIL import Image
import numpy as np

# ################################ INPUT DATA #################################

tif_image = Image.open("map.tiff")
tif_matrix = np.array(tif_image)

# instance_name = "instance_70x70"

instance_name = "instance_40x40"
tif_matrix = tif_matrix[0:40, 0:40]

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
    "road": 0.8,
    "wildlife crossing": 0.8,
    "land acquisition": 0.98,
}
quality_gain_map = {"wildlife crossing": 0, "land acquisition": 1}
improved_probability_map = {"wildlife crossing": 0.95, "land acquisition": 0.995}
cell_cost = {"wildlife crossing": 1, "land acquisition": 0.2}
