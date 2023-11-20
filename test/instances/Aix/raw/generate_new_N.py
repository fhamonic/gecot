import math
import csv
from statistics import mean


def readCSV(file_name, delimiter=","):
    file = csv.DictReader(open(file_name), delimiter=delimiter)
    return list([row for row in file])


def coef_route(route_type):
    match route_type:
        case 0:
            return 1.0
        case 1:
            return 0.9
        case 3:
            return 0.8
        case 5:
            return 1.0
        case 6:
            return 0.7
        case 2:
            return 0
        case 4:
            return 0
        case _:
            raise Exception("oups")


def cost_to_prob(cost, route_type):
    match cost:
        case 1:
            return 1
        case 10:
            return 0.95
        case 150:
            return 0.9
        case 300:
            return 0.7
        case 800:
            return 0.7 * coef_route(route_type)
        case 1000:
            return 0.1


def cost_to_quality(cost):
    if cost == 1:
        return 1
    else:
        return 0


def product(values):
    p = 1
    for v in values:
        p = p * v
    return p


N1_hexagons_csv = readCSV("test/instances/Aix/raw/vertexN1_v2.csv")
N2_hexagons = {}

new_N1_hexagons_csv = open("test/instances/Aix/raw/new_N1.csv", "w")
new_N1_hexagons_csv.write(
    "id,X,Y,area1,area2,area3,area4,cost,init_quality\n"
)

for hexagon in N1_hexagons_csv:
    hexagon_id = hexagon["N2_id"]
    if hexagon_id not in N2_hexagons:
        N2_hexagons[hexagon_id] = []
    N2_hexagons[hexagon_id].append(hexagon)
    new_N1_hexagons_csv.write(
        "{id},{x},{y},{a1},{a2},{a3},{a4},{prob},{qual}\n".format(
            id=hexagon["id"],
            x=hexagon["X"],
            y=hexagon["Y"],
            a1=hexagon["area1"],
            a2=hexagon["area2"],
            a3=hexagon["area3"],
            a4=hexagon["area4"],
            prob=cost_to_prob(int(hexagon["cost"]), int(hexagon["import_rou"])),
            qual=cost_to_quality(int(hexagon["cost"]))
        )
    )

new_N2_hexagons_csv = open("test/instances/Aix/raw/new_N2.csv", "w")
new_N2_hexagons_csv.write(
    "id,X,Y,area1,area2,area3,area4,cost,init_quality\n"
)

for id, hexagons in N2_hexagons.items():
    x = mean([float(row["X"]) for row in hexagons])
    y = mean([float(row["Y"]) for row in hexagons])
    area1 = max([int(row["area1"]) for row in hexagons])
    area2 = max([int(row["area2"]) for row in hexagons])
    area3 = max([int(row["area3"]) for row in hexagons])
    area4 = max([int(row["area4"]) for row in hexagons])
    init_prob = product(
        [
            cost_to_prob(int(row["cost"]), int(row["import_rou"]))
            if row["cost"] != 1000
            else 1
            for row in hexagons
        ]
    )

    nb_bati = sum([row["cost"] == 1000 for row in hexagons])

    init_prob = math.pow(init_prob, 1 / (len(hexagons) - nb_bati))

    if nb_bati in [1, 2]:
        init_prob = 0.4
    if nb_bati in [3, 4]:
        init_prob = 0.2
    if nb_bati in [5, 6, 7]:
        init_prob = 0

    init_quality = mean([cost_to_quality(int(row["cost"])) for row in hexagons])

    new_N2_hexagons_csv.write(
        "{id},{x},{y},{a1},{a2},{a3},{a4},{p},{q}\n".format(
            id=id,
            x=x,
            y=y,
            a1=area1,
            a2=area2,
            a3=area3,
            a4=area4,
            p=init_prob,
            q=init_quality
        )
    )
