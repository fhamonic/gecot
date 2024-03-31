import math
import csv
import numpy as np

def readCSV(file_name, delimiter=','):
    file = csv.DictReader(open(file_name), delimiter=delimiter)
    return list([row for row in file])

hexagons_csv = readCSV('test/instances/Aix/raw/vertexN2_v7.csv')
links_csv = readCSV('test/instances/Aix/raw/AL_N2.csv')
crossref_stretches_csv = readCSV('test/instances/Aix/raw/croisemt_troncon_hexagN2.txt')

global distance_coef
distance_coef = 1

def probability(cost):
    if cost == 1: return math.pow(1, distance_coef)
    elif cost == 10: return math.pow(0.98, distance_coef)
    elif cost == 150: return math.pow(0.8, distance_coef)
    elif cost == 300: return math.pow(0.6, distance_coef)
    elif cost == 800: return math.pow(0.4, distance_coef)
    else: return 0

vegetalized_probability = probability(10)

vertices_csv = open("test/instances/Aix/N2/vertices.csv", "w")
arcs_csv = open("test/instances/Aix/N2/arcs.csv", "w")
stretches_csv = open("test/instances/Aix/N2/stretches.csv", "w")
vertex_options_csv = open("test/instances/Aix/N2/vertex_options.csv", "w")
arc_options_csv = open("test/instances/Aix/N2/arc_options.csv", "w")

vertices_csv.write("hex_id,quality,x,y\n")
arcs_csv.write("arc_id,source_id,target_id,probability\n")
stretches_csv.write("stretch_id,stretch_cost\n")
vertex_options_csv.write("hex_id,stretch_id,quality_gain\n")
arc_options_csv.write("arc_id,stretch_id,improved_prob\n")

hexagons = {}
stretches = {}

for hexagon in hexagons_csv:
    # if hexagon['area2'] != "1": continue
    # if hexagon['area3'] != "1": continue
    if hexagon['area4'] != "1": continue
    hexagon_id = hexagon['N2_id']
    hexagon_cost = int(hexagon['cost_mode'])
    hexagons[hexagon_id] = hexagon_cost
    vertices_csv.write("{id},{weight},{x},{y}\n{id}_in,0,{x},{y}\n{id}_out,0,{x},{y}\n".format(id = hexagon_id,weight = 1 if hexagon_cost == 1 else 0, x = hexagon['X'], y = hexagon['Y']))
    arcs_csv.write("{id}_in_arc,{id}_in,{id},{prob}\n{id}_out_arc,{id},{id}_out,{prob}\n".format(id = hexagon_id,prob = probability(hexagon_cost)))

for link in links_csv:
    s = link['from']
    t = link['to']
    if (s not in hexagons) or (t not in hexagons): continue
    arcs_csv.write("{source}_{target},{source}_out,{target}_in,1\n{target}_{source},{target}_out,{source}_in,1\n".format(source = s,target = t))

for crossref in crossref_stretches_csv:
    stretch_id = crossref['troncon_id']
    hexagon_id = crossref['N2_id']
    if (hexagon_id not in hexagons): continue
    if (stretch_id not in stretches): stretches[stretch_id] = 0
    stretches[stretch_id] += 1

    hexagon_cost = hexagons[hexagon_id]
    if hexagon_cost == 0:
        vertex_options_csv.write("{id},{stretch_id},1\n".format(id=hexagon_id, stretch_id=stretch_id))
    if hexagon_cost > 10:
        arc_options_csv.write("{id}_in_arc,{stretch_id},{p}\n".format(id=hexagon_id, stretch_id=stretch_id, p =vegetalized_probability))

for stretch_id, cost in stretches.items():
    stretches_csv.write("{stretch_id},{cost}\n".format(stretch_id = stretch_id, cost=cost))

vertices_csv.close()
arcs_csv.close()
stretches_csv.close()
vertex_options_csv.close()
arc_options_csv.close()


