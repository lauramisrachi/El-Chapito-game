#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 15 18:13:38 2018

@author: lauramisrachi
"""
import Vertex
import numpy as np
import random 
import copy

# Initialisation of our graph
graph = Vertex.Graph()
file_input = '/Users/lauramisrachi/Documents/Master Maths appliqués UPMC/Python/Projet_El_Chapito/2018_Python_Projet/Test_6.txt'


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

nb_vertex, nb_edge, nb_out_doors, el_chap_init_vertex, l_index_outdoors = read_node_file(file_input, graph)
graph_original = copy.deepcopy(graph)

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


nb_edges_to_close = number_of_edges_to_close(graph, l_index_outdoors)  


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

update_weight_node_adj_outdoors(graph, l_index_outdoors)      


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
    
#node_init = 35
#node_end = 26
#cost, shortest_path = Dijkstra(node_init, node_end, graph)

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
    

def guardian_critical_node_to_choose(l_critical_nodes, l_index_outdoors, graph\
                                     , el_chap_position):
    
    
    """ Function to determine which critical nodes the guardian should choose
    to close. Critical nodes are nodes which have at least two outdoors nodes
    in their adjacents. Indeed, among the critical nodes, there are some which
    would allow El Chapito to win based on only one move. To find these nodes:
    we start from the critical nodes and try to loop back from them up to El 
    Chapito's position, passing through nodes which are directly adjacent to 
    outdoor_nodes.
    
    INPUT : 
        - l_critical_nodes: the list of critical nodes, ie the nodes which are
        neighbors to at least two outdoor nodes 
        - l_index_outdoors : the outdoor nodes of the game
        - graph :  our graph (updated in terms of adjacence, weights...)
        - el_chap_position : the position of El Chapito at this step of the game.
        
    OUTPUT : 
        - l_nodes_to_choose : the list of nodes containing the most critical nodes
        of the critical nodes, as explained before.
        
        
    """ 
    
    
    # Updating the number of outdoors_adj by default
    graph.update_nb_of_outdoors_adj(l_index_outdoors)
    # list of the return, i.e the nodes that the guardian should choose to close
    # an edge from
    l_nodes_to_choose = []
    # The list of adjacents of El Chapito
    l_el_chap_adjacent = graph.node_get_adj(el_chap_position)
    # Looping on all the critical nodes
    for critical_node in l_critical_nodes:
        not_critical = False
        # Starting from the critical node in question
        current_node = critical_node
        # Initialising the list of eventual current nodes
        eventual_current_node = []
        # We do not want to revisit the nodes already visited inside a chain
        already_visited = []
        # While we have not reached one of El Chapito's neighbor : we loop
        while current_node not in l_el_chap_adjacent:
            # Is there a node which is adjacent to our current node and adjacent
            # to an outdoor node, but is not an outdoor node ?
            #print('current node: {}'.format(current_node))
            #print('adj current node : {}'.format(graph.node_get_adj(current_node)))
            for node in graph.node_get_adj(current_node):
                adj_node = graph.node_get_adj(node)
                #print('adj_node : {}'.format(adj_node))
                if not set(adj_node).isdisjoint(l_index_outdoors) and node not in already_visited:
                    if node not in eventual_current_node and node not in l_critical_nodes:
                        eventual_current_node.append(node)
            #print('eventual current node : {}'.format(eventual_current_node))
            if eventual_current_node == []:
                # In this case, this node is not critical
                not_critical = True
                break
            else:
                # The node has been visited inside this chain
                already_visited.append(current_node)
                # We set the current node
                current_node = eventual_current_node[0]
                # We actualise the list of other potential current nodes
                eventual_current_node.remove(current_node)
        if not not_critical:
            l_nodes_to_choose.append(critical_node)        
                    
            
    return  l_nodes_to_choose
        

def one_move_guardian(el_chap_position, l_index_outdoors, graph, nb_edges_to_close):
    
    """ Function to determine which edge the guardian should choose to close
    in one iteration of the game based on the current position of El Chapito
    , the outdoors positions and the current status of the graph (including
    closed edges). To resume, our strategy is the following :
        
        - First, if it is not urgent (i.e. if El Chapito is not on the verge 
        of reaching an outdoor_node), then the guardian choose to suppress one 
        of the critical nodes. It starts by suppressing the most critical of the
        critical nodes. These are determined thanks to the 
        guardian_critical_node_to_choose function defined above. If all of these 
        nodes have already been suppressed, the guardian suppress the rest of the 
        critical nodes, based on the one that are the closest to El Chapito's 
        position. 
        - Second, if El Chapito is on the verge of reaching an outdoor node 
        or if all the critical nodes have been suppressed, the the guardian loops 
        on the potential edges to suppress and chooses the edge which 
        is the closest to El Chapito's position.

    
    INPUT : 
        - el_chap_position :  El Chapito's current position
        - l_index_outdoors : the outdoor nodes of our game
        - graph : our up-to-date graph
        - nb_edges_to_close : the maximal number of edges to close in the game
        ( to state on the winning status of the guardian)
        
    OUTPUT : 
        - node_to_close, out_node_current : the edge to close from the game
        - guardian_win : the winning status of the guardian
        - cost : the distance of the deleted edge to El Chapito's position.
                                                                 
    """
    # Updating the number of outdoors_adj by default
    graph.update_nb_of_outdoors_adj(l_index_outdoors)
    # By default, the guardian has not won the game
    guardian_win = False
    # Number of edges to close in order to win the game
    cost = np.inf 
    
    # We loop on the outdoors
    for out_node in l_index_outdoors:
        list_adjacent = graph.node_get_adj(out_node)
        # We loop on the adjacent nodes of the outdoors node
        for node in list_adjacent:
            # If the number of closed edges is the maximal one
            if graph.nb_closed_edges == nb_edges_to_close - 1:
                # In this case, the guardian has win as all the edges have been 
                # closed before El Chapito could escape
                guardian_win = True
                node_to_close = node
                out_node_current = out_node
                break
            # If this node has already been supressed by the guardian, 
            # we go to the next one
            if graph.visited_by_guardian[node] == out_node:
                continue
            # We evaluate the cost of travel from the current node to El Chapito's position
            cost_current, shortest_path_current,  no_path = Dijkstra(node, el_chap_position, graph)
            # We will suppress the edge which is linked to the closest outdoors from El Chapito
            if cost_current < cost:
                cost = cost_current
                node_to_close = node
                out_node_current = out_node
    
    # If there are no outdoor nodes in El Chapito adjacent node number, then the guardian choose 
    # to eliminate one of the priority nodes ie the ones that have at least
    # two outdoor nodes as their adjacents.
    cost2 = np.inf
    if set(l_index_outdoors).isdisjoint(graph.node_get_adj(el_chap_position)):
        dict_temp = graph.nb_of_outdoors_adj
        # List of critical nodes
        l_critical_nodes = [key for key, value in dict_temp.items() if value >= 2]
        # Calling the function to choose the critical nodes to close at first : the 
        # one that could allow El Chapito to win based on its first move.
        l_nodes_to_choose = guardian_critical_node_to_choose(l_critical_nodes, \
                    l_index_outdoors, graph, el_chap_position)
        # If there is only one node of that kind, we suppress it directly
        if len(l_nodes_to_choose) == 1:
            node_to_close = l_nodes_to_choose[0]
            out_node_current = [i for i in graph.node_get_adj(node_to_close) if i in l_index_outdoors][0]
        # If there are more node of that kind, suppress the one which is the closest to El Chapito
        if len(l_nodes_to_choose) > 1:
            for node in l_nodes_to_choose:
                cost_temp, shortest_path2, no_path = Dijkstra2(node, el_chap_position, graph)
                if cost_temp < cost2:
                    cost2 = cost_temp
                    node_to_close = node
                    out_node_current = [i for i in graph.node_get_adj(node_to_close) if i in l_index_outdoors][0]
        # If there are no nodes of that kind, simply suppress some of the critical nodes.
        if len(l_nodes_to_choose) == 0:
            for e in l_critical_nodes:
                cost_temp, shortest_path2, no_path = Dijkstra(e, el_chap_position, graph)
                if cost_temp < cost2:
                    cost2 = cost_temp
                    node_to_close = e
                    out_node_current = [i for i in graph.node_get_adj(node_to_close) if i in l_index_outdoors][0]
                    
                    
    # If the guardian has not won
    if not guardian_win:           
        # In this case, we need to suppress the edge between node_to_close
        # and out_node_current. This is done with the delete_edge method.
        graph.delete_edge(out_node_current, node_to_close)
        graph.increment_closed_edges()
        # We also specify that node_to_close has already been visited by the guardian
        # and from which outdoor node
        graph.node_visited_guardian(node_to_close, out_node_current)
        # We also increment the weight of the closed edge
        graph.update_weight(node_to_close, out_node_current, 7)
        # We also update the number of outdoors adjacent number of the graph
        graph.update_nb_of_outdoors_adj(l_index_outdoors)
        update_weight_node_adj_outdoors(graph, l_index_outdoors)     
    return node_to_close, out_node_current, guardian_win, cost


def one_move_el_chap(el_chap_position, l_index_outdoors, graph):
    
    """ Function to determine which position El chapito should go to, given
    the position of the outdoors, the current status of the graph (including
    closed edges). The strategy followed could be summed up as follows:
        - By priority, El Chapito's moves torward the (still) open critical 
        points, which are to closest to him.
        - Then, when all of these nodes have been investigated, El Chapitos 
        just seeks for the best nodes to suppress based on the Dijksta2 algorithm, 
        and the particular weights that have been set for the different edges
        with the update_weight_node_adj_outdoors function defined above.
        
    INPUT : 
        - el_chap_position : El Chapito's initial position 
        - l_index_outdoors : the outdoor nodes of our game
        - graph : our up-to-date graph.
        
    OUTPUT : 
        - node_to_go : the optimal node El Chapito should choose to reach in 
        his next turn.
        - el_chap_win : the winning status of El Chapito based on this move.
        
    """
    # By default, El Chapito has not won unless specified
    el_chap_win = False
    
    # By priority, El Chapito goes towards the open critical points
    #ie the nodes that are adjacents to at least two outdoor nodes. And it goes
    # towards the closest one
    # Updating the number of outdoors_adj by default and precaution
    graph.update_nb_of_outdoors_adj(l_index_outdoors)
    dict_temp = graph.nb_of_outdoors_adj
    l_critical_nodes = [key for key, value in dict_temp.items() if value >= 2]
    # Initialising the cost
    cost = np.inf
    # In priority, we go in the direction of the critical nodes
    if l_critical_nodes:
        for node in l_critical_nodes:
            cost_current, shortest_path_current, no_path = Dijkstra2(el_chap_position, node, graph)
            if no_path:
                continue
            if cost_current < cost:
                cost = cost_current
                shortest_path = shortest_path_current
        node_to_go = shortest_path[-2]
    # Otherwise, we just seek for the best other nodes to go to 
    else:
        for node in l_index_outdoors:
            cost_current, shortest_path_current, no_path = Dijkstra2(el_chap_position, node, graph)
            if no_path:
                continue
            if cost_current < cost:
                cost = cost_current
                shortest_path = shortest_path_current
        node_to_go = shortest_path[-2]
    
    # Setting the winning conditions
    if node_to_go in l_index_outdoors:
        el_chap_win = True
    
    
    return node_to_go, el_chap_win
    

def El_Chapito_Game(el_chap_init_vertex, l_index_outdoors, graph, nb_edges_to_close):
    
    """
    Function to simulate the El Chapito Game, with alternate moves of the guardian 
    and El Chapitos. As stated in the rules, the guardian starts to play first
    at the beginning of each game. The game stops when either El Chapito or 
    the guardian has won. 
    
    INPUT : 
        - el_chap_init_vertex : the initial position of El Chapito in the game.
        - l_index_outdoors : the outdoor nodes of our game.
        - nb_edges_to_close: the total number of edges to close for the 
        game to finish. 
        
    OUTPUT : 
        None. The different moves are printed within the function. 
    
    """
    
    # Counter for the number of turns in the game
    k = 0 
    el_chap_win = False
    guardian_win = False
    el_chap_position = el_chap_init_vertex
    while not el_chap_win and not guardian_win:
        node_to_close, out_node_current, guardian_win, cost \
        = one_move_guardian(el_chap_position, l_index_outdoors, graph, nb_edges_to_close)
        k += 1
        print('\n \n At iteration {}, the Guardian closes the edge {}-{}. Did the guardian win with this move ? {}'\
              .format(k, out_node_current, node_to_close, guardian_win))
        if guardian_win:
            break
        el_chap_position, el_chap_win \
        = one_move_el_chap(el_chap_position, l_index_outdoors, graph)
        k += 1
        print('\n \n At iteration {}, El Chapito reaches the position {}. Did El Chapito win with this move ? {}'\
              .format(k, el_chap_position, el_chap_win))
        if el_chap_win:
            break

El_Chapito_Game(el_chap_init_vertex, l_index_outdoors, graph, nb_edges_to_close)        
    


## Test 0 : it is ok. Same as the test example.

## Test 1 : it is ok. Same as the test example.

## Test 2 : it is ok. Same as the test example.

## Test 3 : it is ok. Same as the test example.

## Test 4 : Here the guardian wins. It is globally the same as the test example.

 ## Test 5 : The guardian wins. It is globally the same as the test example.
 
 ## Test 6 : The guardian wins. It is globally the same as the test example.
 
 ## Test 7: The guardian wins. 
 
 ## Test 8: The guardian wins. It is globally the same as the test example. 
 
 ## Test 9: The guardian wins. It is globally the same as the test example. 
 
 ## Test 10: The guardian wins. It is globally the same as the test example. 
 
 ## Test 11 : The guardian wins. It is globally the same as the test example. 
 
 ## Test 12 : El chapito wins. Same as the test example. 
 
 
 
 