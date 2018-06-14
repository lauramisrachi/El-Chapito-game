#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 10 12:20:30 2018

@author: lauramisrachi
"""

def read_node_file(file_input, graph):
    
    """ Function to read a node file as input and to complete the graph
    information based on the input file.
    
    INPUT:
        - file_input : the file containing the informations about the nodes:
            their edges...
        - an empty graph that is filled in with nodes and edges with 
            this function
    
    OUTPUT:
        - nb_vertex : the number of nodes in the graph
        - nb_edges : the number of edges in the graph
        - nb_out_doors : the number of outdoors in the graph
        - el_chap_init_vertex : the position of El Chapito at the beginning 
            of the game
        - l_index_outdoors : the position of the outdoors in the game
        
    """
    with open(file_input, 'r') as file_in:
        
        # reading the first line and keeping the info : number of vertices, 
        # number of edges, number of outdoors.
        line = file_in.readline().split()
        nb_vertex = int(line[0])
        nb_edge = int(line[1])
        nb_out_doors = int(line[2])
        
        # reading the second line and finding the initial position of El Chapito
        line = file_in.readline().split()
        el_chap_init_vertex = int(line[0])
        
        # Reading the lines giving the vertices and their coordinates
        for i in range(nb_vertex):
            line = file_in.readline().split()
            node = int(line[0])
            coord_x = float(line[1])
            coord_y = float(line[2])
            graph.add_vertex(node, coord_x, coord_y)
            
        # Reading the lines giving the edges to create within the graph
        for i in range(nb_edge):
            line = file_in.readline().split()
            node1 = int(line[0])
            node2 = int(line[1])
            graph.add_edge(node1, node2)
            
        # Reading the line which gives the index of the outdoors
        line = file_in.readline().split()
        l_index_outdoors = []
        
        for i in line:
            l_index_outdoors.append(int(i))
            
    return nb_vertex, nb_edge, nb_out_doors, el_chap_init_vertex, l_index_outdoors



def number_of_edges_to_close(graph, l_index_outdoors):
    """
        Function to determine the total number of edges to close
        for the guardian to win the game. This function should be called
        exclusively at the beginning of the program, when the graph has not
        been modified yet.
        
        INPUT : 
            graph : the graph of our game
        
        OUTPUT : 
            nb_edge_to_close : the number of edges to be close
            for the guardian to win the game.
        
    """
    nb_edges_to_close = 0
    for out_node in l_index_outdoors:
        list_adjacent = graph.node_get_adj(out_node)
        nb_edges_to_close += len(list_adjacent)
        
    return nb_edges_to_close



def update_weight_node_adj_outdoors(graph, l_index_outdoors):
    
    """
    Function to update the weight of the nodes/edges in the graph. This function 
    should be called to update the weight of the edges close to an outdoor node.
    This should be done all along during the game.
    The cost are set as follows:
        - 1 for nodes that are adjacent to at least 2 outdoor nodes
        - 10 for nodes that are adjacent to just one outdoor node and that
           are terminal nodes: this helps preventing El chapito to go in that direction.
        - 2 for nodes that are adjacent to just one outdoor node but that are
        not terminal nodes.
        - the other cost are not modified
        
    INPUT : 
        - graph :  our graph which is up-to-date (adjacent list, weights...)
        - l_index_outdoors : the list of outdoor nodes in our graph.
        
    OUTPUT : 
        - no output but the graph has been modified in terms of its 'weights' attribute
        
    """
    graph.update_nb_of_outdoors_adj(l_index_outdoors)
    for node1 in l_index_outdoors:
        for node2 in graph.node_get_adj(node1):
            if graph.nb_of_outdoors_adj[node2] >= 2:
                graph.update_weight(node1, node2, 1)
            if graph.nb_of_outdoors_adj[node2] == 1: 
                if len(graph.node_get_adj(node1)) == 1:
                    graph.update_weight(node1, node2, 10)
                else:
                    graph.update_weight(node1, node2, 2)