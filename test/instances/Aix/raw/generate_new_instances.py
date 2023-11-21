import math
import csv
import numpy as np

def readCSV(file_name, delimiter=','):
    file = csv.DictReader(open(file_name), delimiter=delimiter)
    return list([row for row in file])

hexagons_csv = readCSV('test/instances/Aix/raw/new_N1.csv')
links_csv = readCSV('test/instances/Aix/raw/AL_N1.csv')
crossref_stretches_csv = readCSV('test/instances/Aix/raw/croisemt_troncon_hexagN2.txt')

vegetalized_probability = 0.98

vertices_csv = open("test/instances/Aix/N1/vertices.csv", "w")
arcs_csv = open("test/instances/Aix/N1/arcs.csv", "w")
stretches_csv = open("test/instances/Aix/N1/stretches.csv", "w")
vertex_options_csv = open("test/instances/Aix/N1/vertex_options.csv", "w")
arc_options_csv = open("test/instances/Aix/N1/arc_options.csv", "w")

vertices_csv.write("hexagon_id,quality,x,y\n")
arcs_csv.write("arc_id,source_id,target_id,probability\n")
stretches_csv.write("stretch_id,stretch_cost\n")
vertex_options_csv.write("hexagon_id,stretch_id,quality_gain\n")
arc_options_csv.write("arc_id,stretch_id,improved_prob\n")

hexagons = {}
stretches = {}

for hexagon in hexagons_csv:
    # if hexagon['area1'] != "1": continue
    # if hexagon['area2'] != "1": continue
    # if hexagon['area3'] != "1": continue
    if hexagon['area4'] != "1": continue
    hexagon_id = hexagon['N2']
    hexagons[hexagon_id] = hexagon
    hexagons[hexagon_id]['adj_route'] = int(hexagon['route'])
    vertices_csv.write("{id},{weight},{x},{y}\n{id}_in,0,{x},{y}\n{id}_out,0,{x},{y}\n".format(id = hexagon_id, weight = hexagon['init_quality'], x = hexagon['X'], y = hexagon['Y']))
    arcs_csv.write("{id}_in_arc,{id}_in,{id},{prob}\n{id}_out_arc,{id},{id}_out,{prob}\n".format(id = hexagon_id, prob = hexagon['init_prob']))

for link in links_csv:
    s = link['from']
    t = link['to']
    if (s not in hexagons) or (t not in hexagons): continue
    arcs_csv.write("{source}_{target},{source}_out,{target}_in,1\n{target}_{source},{target}_out,{source}_in,1\n".format(source = s,target = t))
    hexagons[s]['adj_route'] = max(hexagons[s]['adj_route'], int(hexagons[t]['route']))
    hexagons[t]['adj_route'] = max(hexagons[t]['adj_route'], int(hexagons[s]['route']))
    

for crossref in crossref_stretches_csv:
    stretch_id = crossref['troncon_id']
    hexagon_id = crossref['N2_id']
    if (hexagon_id not in hexagons): continue
    if int(hexagons[hexagon_id]["route"]) == 0: continue
    if int(hexagons[hexagon_id]["adj_route"]) == 0: continue
    if (stretch_id not in stretches): stretches[stretch_id] = 0
    stretches[stretch_id] += 1

    arc_options_csv.write("{id}_in_arc,{stretch_id},{p}\n".format(id=hexagon_id, stretch_id=stretch_id, p =vegetalized_probability))

for stretch_id, cost in stretches.items():
    stretches_csv.write("{stretch_id},{cost}\n".format(stretch_id = stretch_id, cost=cost))

vertices_csv.close()
arcs_csv.close()
stretches_csv.close()
vertex_options_csv.close()
arc_options_csv.close()


