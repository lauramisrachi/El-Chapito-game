#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 15 17:10:17 2018

@author: lauramisrachi
"""

class Graph(object):
    
    """ We define the Graph class
    useful to generate a graph for our El Chapito Game"""
    
    def __init__(self):
        self.__vertices = {}
        self.__node_adjacent = {}
        self.__visited_by_guardian = {}
        self.__nb_closed_edges = 0
        self.__weights = {}
        self.__nb_of_outdoors_adj = {}
    
    # The getter for the vertices attribute    
    @property 
    def vertices(self):
        return self.__vertices
   
    # The setter for the vertices attribute
    @vertices.setter
    def vertices(self, dictionnary):
        self.__vertices = dictionnary
        
    # Method to add a vertex to our graph and to add its visited_by_guardian
    # attribute
    def add_vertex(self, node, coord_x, coord_y):
        self.__vertices[node] = [coord_x, coord_y]
        self.__visited_by_guardian[node] = None
    
    # The getter for our node_adjacent attribute
    @property
    def node_adjacent(self):
        return self.__node_adjacent
    
    # The setter for our node_adjacent attribute
    @node_adjacent.setter
    def node_adjacent(self, adjacent_dicto):
        self.__node_adjacent = adjacent_dicto
        
    # The getter for our visited_by_guardian attribute
    @property
    def visited_by_guardian(self):
        return self.__visited_by_guardian
    
    # The setter for our visited_by_guardian attribute
    @visited_by_guardian.setter
    def visited_by_guardian(self, dict_of_visited):
        self.__visited_by_guardian = dict_of_visited
     
    # The getter for our nb_closed_edges attribute    
    @property
    def nb_closed_edges(self):
        return self.__nb_closed_edges
    
    # The setter for our nb_closed_edges attribute
    @nb_closed_edges.setter
    def nb_closed_edges(self, nb):
        self.__nb_closed_edges = nb
        
    # The getter for our weights attribute       
    @property
    def weights(self):
        return self.__weights

    # The setter for our weights attribute
    @weights.setter
    def weights(self, weights_dic):
        self.__weights = weights_dic
        
    # The getter for the nb_of_outdoors_adj attribute    
    @property 
    def nb_of_outdoors_adj(self):
        return self.__nb_of_outdoors_adj
   
    # The setter for the nb_of_outdoors_adj attribute
    @nb_of_outdoors_adj.setter
    def nb_of_outdoors_adj(self, dictionnary):
        self.__nb_of_outdoors_adj = dictionnary

   
    # Method to add an edge to our graph : 
    # that way node_adjacent is a dict with the following shape:
    # {nb_vertices : list_of_adjacent_vertices}
    def add_edge(self, node1, node2):
        if node1 in self.__node_adjacent: 
            self.__node_adjacent[node1].append(node2)
            self.__weights[(node1, node2)] = 5
        else:
            self.__node_adjacent[node1] = [node2]
            self.__weights[(node1, node2)] = 5
        if node2 in self.__node_adjacent: 
            self.__node_adjacent[node2].append(node1)
            self.__weights[(node2, node1)] = 5
        else:
            self.__node_adjacent[node2] = [node1]
            self.__weights[(node2, node1)] = 5
    
    # Method to delete a node from the entire graph
    def delete_node(self, node_to_delete):
        # The node has to be deleted from the keys
        del self.__node_adjacent[node_to_delete]
        # The node has to be deleted from the list of adjacent nodes 
        # for all the other nodes of the dictionary
        for list_adjacent in self.__node_adjacent.values():
            if node_to_delete in list_adjacent:
                list_adjacent.remove(node_to_delete)
                
    
    # Method to delete an edge, i.e. to delete node2 from the list of 
    # ajdacent of node1 and vice-versa.
    def delete_edge(self, node1, node2):
        # We delete node1 from the list of adjacent of node2 if it is present:
        if node1 in self.__node_adjacent[node2]:
            self.__node_adjacent[node2].remove(node1)
        # We delete node2 from the list of adjacent of node1 if it is present:
        if node2 in self.__node_adjacent[node1]:
            self.__node_adjacent[node1].remove(node2)
        
        
    # Method to get the list of adjacents node for a given node
    def node_get_adj(self, node):
        return self.__node_adjacent[node]
    
        
    # Method to indicate if a node has been visited by the guardian or not
    # and from which outdoor node if it was the case
    def node_visited_guardian(self, node, outdoor_node):
        self.__visited_by_guardian[node] = outdoor_node
    
    # Method to increment the number of closed edges
    def increment_closed_edges(self):
        self.__nb_closed_edges += 1
        
    # Method to update the weight of a given node
    def update_weight(self, node1, node2, weight):
        self.__weights[(node1, node2)] = weight
        self.__weights[(node2, node1)] = weight
        
    def update_nb_of_outdoors_adj(self, l_index_outdoors):    
        """
        Function to determine the number of outdoors node which are adjacent to 
        each node of a given graph. The returning result is a dictionary with the 
        following shape:
        {index of node: number of outdoors node adjacent}
        """
        for node in self.__node_adjacent:
            self.__nb_of_outdoors_adj[node] = 0
            for e in self.__node_adjacent[node]:
                if e in l_index_outdoors:
                    self.__nb_of_outdoors_adj[node] += 1
                     
    def __repr__(self):
        return str(self.__node_adjacent)
    
    
    
    