#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 10 12:18:36 2018

@author: lauramisrachi
"""

import numpy as np
import random 


def Dijkstra(node_init, node_end, graph):
    
    """ Function to determine the shortest path from a certain node_init 
    in our graph down to node_end in our graph. Unit weights are used here
    between two nodes.
    
    INPUT : 
        - node_init : the label (number) of the node we depart from
        - node_end : the label (number) of the end node
    
    OUTPUT : 
        - shortest_path : the list of nodes to cover to reach the shortest path
        - cost : the cost of this shortest path in terms of distance
        
    """

    ### Parameter initialisation
    node_list = list(graph.vertices.keys())
    dist = np.full(len(node_list), -np.inf)
    # At the beginning we have not reached the end_node
    node_end_reached = False
    # At the beginning, we assume there is a shortest path:
    no_path = False

    # Initialising the distances of the nodes
    dist[node_init] = 0
    # Setting the father_node which contains the provenance of the nodes
    father_node = np.full(len(node_list), -np.inf)
    # Initialising the current node
    current_node = node_init 
    # Initialising the dictionnary of fixed node which has the following shape:
    #{fixed_node: previous_node}
    # Fixing the number of iterations
    k = 0
    dict_fixed_node = {node_init:(None,k)}
    
    # In the trivial case where the two nodes are identical
    if node_init == node_end:
        cost = 0
        shortest_path = [node_init]
        no_path = False
        return cost, shortest_path, no_path
    
    # While the end node has not been reached
    while not node_end_reached:
        current_node_adj = graph.node_get_adj(current_node).copy()
        # We get rid off the node that have been fixed, except at the first iteration
        if k != 0:
            current_node_adj.remove(dict_fixed_node[current_node][0])
        ## Updating the distances : either the node are neighbors and 
        # something might change, either they are not, and their distance 
        # does not change.
        # For the neighbors node
        for e in current_node_adj:
            dist_temp = dist[current_node] + 1
            # We change the distance only if it is lower than it used to be
            # otherwise, we keep it
            if dist_temp < dist[e] or dist[e] == -np.inf:
                dist[e] = dist_temp
                # Setting the father node
                father_node[e] = current_node
            father_node[current_node] = None
        # We set the distance of the current node to 0
        dist[current_node] = 0  
        # Index and distances which are not 0 and not minus infty
        sub_dist_index = [i for i, e in enumerate(dist) if e > 0]
        sub_dist_value = np.array([e for i, e in enumerate(dist) if e > 0])
        # If these two lists are empty, we stop the algorithm and that means
        # that we cannot reach our point
        if not sub_dist_index or sub_dist_value.size == 0:
            no_path = True
            cost = 'impossible path'
            shortest_path = 'impossible path'
            break
        # Now we need to set our choice for the next node
        if np.array_equal(sub_dist_value, np.ones(len(sub_dist_value))):
            ## If there are only ones : we pick them up randomly
            current_node = int(random.choice(list(sub_dist_index)))
        else:
            ## If not we just pick up the one with the minimum distance.
            current_node = sub_dist_index[sub_dist_value.argmin()]
        # Adding this node to the dictionnary
        dict_fixed_node[current_node] = (int(father_node[current_node]), k)
        # If the end_node has been reached, we stop the search algorithm
        if node_end in dict_fixed_node.keys():
            node_end_reached = True
        # Incrementing the counter
        k += 1

    # Now we need to get the shortest path from our iterations whose information 
    # are in dict_fixed_node. To do this, we need to circle back from the end_node
    # to the init_node in this dictionnary.
    # This is done only if some path between node_init and end_node exists.
    if no_path == False:
        list_father_node = list(dict_fixed_node.values())
        previous_node = list_father_node[-1][0]
        shortest_path = [node_end, previous_node]
        # While the initial node has not been reached, we add the relevant
        # nodes to our shortest path
        while previous_node != node_init:
            previous_node = dict_fixed_node[previous_node][0]
            shortest_path.append(previous_node)
    
        # Computing the cost of this shortest path in terms of weights
        cost = len(shortest_path) - 1
    
    return cost, shortest_path, no_path
    

def Dijkstra2(node_init, node_end, graph):
    
    """ Function to determine the shortest path from a certain node_init 
    in our graph down to node_end in our graph. This second version of the Dijkstra
    algorithm incorporates weights for each edge, based on the 'weights' graph 
    attribute defined in the Graph class (cf Vertex.py file).
    
    INPUT : 
        - node_init : the label (number) of the node we depart from
        - node_end : the label (number) of the end node
    
    OUTPUT : 
        - shortest_path : the list of nodes to cover to reach the shortest path
        - cost : the cost of this shortest path in terms of distance
        
    """

    ### Parameter initialisation
    node_list = list(graph.vertices.keys())
    dist = np.full(len(node_list), -np.inf)
    # At the beginning we have not reached the end_node
    node_end_reached = False
    # At the beginning, we assume there is a shortest path:
    no_path = False
    
    # Initialising the distances of the nodes
    dist[node_init] = 0
    # Setting the father_node which contains the provenance of the nodes
    father_node = np.full(len(node_list), -np.inf)
    # Initialising the current node
    current_node = node_init 
    # Initialising the dictionnary of fixed node which has the following shape:
    #{fixed_node: (previous_node, iteration, cost)}
    # Fixing the number of iterations
    k = 0
    dict_fixed_node = {node_init:(None,k, 0)}
    
    # In the trivial case where the two nodes are identical
    if node_init == node_end:
        cost = 0
        shortest_path = [node_init]
        no_path = False
        return cost, shortest_path, no_path
    
    # While the end node has not been reached
    while not node_end_reached:
        current_node_adj = graph.node_get_adj(current_node).copy()
        # We get rid off the nodes that have been fixed, except at the first iteration
        if k != 0:
            current_node_adj.remove(dict_fixed_node[current_node][0])
        ## Updating the distances : either the node are neighbors and 
        # something might change, either they are not, and their distance 
        # does not change.
        # For the neighbors node
        for e in current_node_adj:
            dist_temp = dist[current_node] + graph.weights[(current_node, e)]
            # We change the distance only if it is lower than it used to be
            # otherwise, we keep it
            if dist_temp < dist[e] or dist[e] == -np.inf:
                dist[e] = dist_temp
                # Setting the father node
                father_node[e] = current_node
            father_node[current_node] = None
        # We set the distance of the current node to 0
        dist[current_node] = 0  
        # Index and distances which are not 0 and not minus infty
        sub_dist_index = [i for i, e in enumerate(dist) if e > 0]
        sub_dist_value = np.array([e for i, e in enumerate(dist) if e > 0])
        # If these two lists are empty, we stop the algorithm and that means
        # that we cannot reach our point
        if not sub_dist_index or sub_dist_value.size == 0:
            no_path = True
            cost = 'impossible path'
            shortest_path = 'impossible path'
            break
        # Now we need to set our choice for the next node
        if np.array_equal(sub_dist_value, np.ones(len(sub_dist_value))):
            ## If there are only ones : we pick them up randomly
            current_node = int(random.choice(list(sub_dist_index)))
            min_dist = sub_dist_value.min()
        else:
            ## If not we just pick up the one with the minimum distance.
            current_node = sub_dist_index[sub_dist_value.argmin()]
            min_dist = sub_dist_value.min()
        # Adding this node to the dictionnary
        dict_fixed_node[current_node] = (int(father_node[current_node]), k, min_dist)
        # If the end_node has been reached, we stop the search algorithm
        if node_end in dict_fixed_node.keys():
            node_end_reached = True
        # Incrementing the counter
        k += 1
        #print('current_node : {}'.format(current_node))
    #print(dict_fixed_node)
    # Now we need to get the shortest path from our iterations whose information 
    # are in dict_fixed_node. To do this, we need to circle back from the end_node
    # to the init_node in this dictionnary.
    # This is done only if some path between node_init and end_node exists.
    if no_path == False:
        list_father_node = list(dict_fixed_node.values())
        previous_node = list_father_node[-1][0]
        shortest_path = [node_end, previous_node]
        # While the initial node has not been reached, we add the relevant
        # nodes to our shortest path
        while previous_node != node_init:
            previous_node = dict_fixed_node[previous_node][0]
            shortest_path.append(previous_node)
    
        # Computing the cost of this shortest path in terms of weights
        cost = int(dict_fixed_node[node_end][2])
        
    return cost, shortest_path, no_path