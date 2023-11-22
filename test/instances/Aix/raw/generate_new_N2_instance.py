import csv


def readCSV(file_name, delimiter=","):
    file = csv.DictReader(open(file_name), delimiter=delimiter)
    return list([row for row in file])


def writeCSV(file_name, columns):
    file = open("test/instances/Aix/" + file_name, "w")
    file.write(columns + "\n")
    return file


hexagons_csv = readCSV("test/instances/Aix/raw/new_N2.csv")
links_csv = readCSV("test/instances/Aix/raw/AL_N2.csv")
crossref_stretches_csv = readCSV("test/instances/Aix/raw/croisemt_troncon_hexagN2.txt")

vegetalized_probability = 0.98

vertices_csv = writeCSV("N2/vertices.csv", "hex_id,quality,x,y")
arcs_csv = writeCSV("N2/arcs.csv", "arc_id,source_id,target_id,probability")
stretches_csv = writeCSV("N2/stretches.csv", "stretch_id,stretch_cost")
vertex_options_csv = writeCSV("N2/vertex_options.csv", "hex_id,stretch_id,quality_gain")
arc_options_csv = writeCSV("N2/arc_options.csv", "arc_id,stretch_id,improved_prob")

hexagons = {}
stretches = {}

for hexagon in hexagons_csv:
    # if hexagon['area1'] != "1": continue
    # if hexagon['area2'] != "1": continue
    # if hexagon['area3'] != "1": continue
    if hexagon['area4'] != "1": continue
    id = hexagon["id"]
    hexagons[id] = hexagon
    hexagons[id]["adj_route"] = int(hexagon["route"])
    vertices_csv.write(
        "{id},{weight},{x},{y}\n{id}_in,0,{x},{y}\n{id}_out,0,{x},{y}\n".format(
            id=id, weight=hexagon["init_quality"], x=hexagon["X"], y=hexagon["Y"]
        )
    )
    arcs_csv.write(
        "{id}_in_arc,{id}_in,{id},{prob}\n{id}_out_arc,{id},{id}_out,{prob}\n".format(
            id=id, prob=hexagon["init_prob"]
        )
    )

for link in links_csv:
    s = link["from"]
    t = link["to"]
    if (s not in hexagons) or (t not in hexagons):
        continue
    arcs_csv.write(
        "{source}_{target},{source}_out,{target}_in,1\n{target}_{source},{target}_out,{source}_in,1\n".format(
            source=s, target=t
        )
    )
    hexagons[s]["adj_route"] = max(hexagons[s]["adj_route"], int(hexagons[t]["route"]))
    hexagons[t]["adj_route"] = max(hexagons[t]["adj_route"], int(hexagons[s]["route"]))


for crossref in crossref_stretches_csv:
    stretch_id = crossref["troncon_id"]
    N2_id = crossref["N2_id"]
    if N2_id not in hexagons:
        continue
    hexagon = hexagons[N2_id]
    if stretch_id not in stretches:
        stretches[stretch_id] = 0
    stretches[stretch_id] += 1
    if int(hexagon["route"]) == 0:
        continue
    if int(hexagon["adj_route"]) == 0:
        continue

    arc_options_csv.write(
        "{id}_in_arc,{stretch_id},{p}\n".format(
            id=hexagon["id"], stretch_id=stretch_id, p=vegetalized_probability
        )
    )

for stretch_id, cost in stretches.items():
    stretches_csv.write(
        "{stretch_id},{cost}\n".format(stretch_id=stretch_id, cost=cost)
    )

vertices_csv.close()
arcs_csv.close()
stretches_csv.close()
vertex_options_csv.close()
arc_options_csv.close()
