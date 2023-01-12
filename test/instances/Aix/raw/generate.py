import math
import csv
import numpy as np

def readCSV(file_name, delimiter=','):
    file = csv.DictReader(open(file_name), delimiter=delimiter)
    return list([row for row in file])

hexagons_csv = readCSV('test/instances/Aix/raw/vertexN2_v7.csv')
links_csv = readCSV('test/instances/Aix/raw/AL_N2.csv')

global distance_coef = 1

def probability(cost):
    if cost == 1: return math.pow(1, distance_coef)
    elif cost == 10: return math.pow(0.98, distance_coef)
    elif cost == 150: return math.pow(0.8, distance_coef)
    elif cost == 300: return math.pow(0.6, distance_coef)
    elif cost == 800: return math.pow(0.4, distance_coef)
    else return 0

vertices_csv = open("test/instances/Aix/vertices.csv", "w")
arcs_csv = open("test/instances/Aix/arcs.csv", "w")
stretches_csv = open("test/instances/Aix/stretches.csv", "w")
vertex_options_csv = open("test/instances/Aix/vertex_options.csv", "w")
arc_options_csv = open("test/instances/Aix/arc_options.csv", "w")

vertices_csv.write("id,length,x,y\n")
arcs_csv.write("arc_id,source_id,target_id,probability\n")

vertices = set()

for hexagon in hexagons_csv:
    if hexagon['area2'] != 1: continue;
    vertices.add(hexagon['N2_id'])
    vertices_csv.write("{id},{weight},{x},{y}\n{id}_in,0,{x},{y}\n{id}_out,0,{x},{y}\n".format(id = hexagon['N2_id'],weight = 1 if hexagon['cost'] == 1 else 0, x = hexagon['X'], y = hexagon['Y']))
    arcs_csv..write("{id}_in_arc,{id}_in,{id},{prob}\n{id}_out_arc,{id},{id}_out,{prob}\n".format(id = hexagon['N2_id'],prob = probability(hexagon['cost'])))

for link in links_csv:
    s = link['from']
    t = link['to']
    if (s not in vertices) or (t not in vertices): continue
    arcs_csv..write("{source}_{target},{source}_out,{target}_in,1\n{target}_{source},{target}_out,{source}_in,1\n".format(source = s,target = t))


arc_options_csv.write("arc_id,option_id,improved_prob\n")


vertices_csv.close()
arcs_csv.close()
stretches_csv.close()
vertex_options_csv.close()
arc_options_csv.close()


