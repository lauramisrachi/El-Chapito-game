#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 10 12:32:08 2018

@author: lauramisrachi
"""

import Vertex
from Dijkstra_algo import *
from game_init import *
from game_with_strategy import *

# Initialisation of our graph
graph = Vertex.Graph()
file_input = '/Users/lauramisrachi/Documents/Master Maths appliqués UPMC/Python/Projet_El_Chapito/2018_Python_Projet/Test_10.txt'

# Reading and creating the node graph
nb_vertex, nb_edge, nb_out_doors, el_chap_init_vertex, l_index_outdoors = read_node_file(file_input, graph)
graph_original = copy.deepcopy(graph)

# Computing the number of edges to close at the beginning of the game
nb_edges_to_close = number_of_edges_to_close(graph, l_index_outdoors)  

# Updating the weights of the adjacent nodes at the beginning of the game
update_weight_node_adj_outdoors(graph, l_index_outdoors)  

# Running the game
El_Chapito_Game(el_chap_init_vertex, l_index_outdoors, graph, nb_edges_to_close)      