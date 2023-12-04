# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 17:39:44 2023

@author: Lenovo
"""

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt


def get_nodes_and_sides(obstacles, start, goal) :
    """
    args:   obstacles, list of #obs lists of tuples (coord of vertices of obstacles)
            start,     starting point position of the robot (~center of Thymio)
    return: nodes,     list of nodes numbers
            vertices,  list of coordinates of the vertices associated with nodes
            sides,     list of #obs lists of tuples (end-nodes of each side of obstacle)
    """
    vertices = []
    nodes = [[0]]
    vertices.append(tuple(start))
    i = 1
    for obstacle in obstacles :
        obs_nodes = []
        for vertex in obstacle :
            vertices.append(vertex)
            obs_nodes.append(i)
            i += 1
        nodes.append(obs_nodes)
    nodes.append([i])
    vertices.append(tuple(goal))
    
    sides = []
    curr_node = 1
    for obstacle in obstacles :
        obs_sides = []
        first_node = curr_node
        for vertex in range(len(obstacle)) :
            obs_sides.append(tuple([first_node+vertex, first_node+(vertex+1)%len(obstacle)]))
            curr_node += 1
        sides.append(obs_sides)
        
    return nodes, vertices, sides

def intersect(num1a, num1b, num2a, num2b, vertices) :
    """
    args   : num-i,l endpoints a and b of segments 1 and 2.
             vertices, coordinates of points.
             eps : tolerance against computations errors
    return : whether segments 1 and 2 intersect or not.
    """
    # Get corresponding coordinates in the tab
    point1a = vertices[num1a]
    point1b = vertices[num1b]
    point2a = vertices[num2a]
    point2b = vertices[num2b]
    
    def on_segment(p, q, r):
        """
        determine whether point q is (stricly or not) between p and r, for p q r on a same line.
        """
        if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                    q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
                return True
        return False

    def are_collinear_and_overlap(p1a, p1b, p2a, p2b):
        return (min(p1a[0], p1b[0]) <= max(p2a[0], p2b[0]) and
                min(p2a[0], p2b[0]) <= max(p1a[0], p1b[0]) and
                min(p1a[1], p1b[1]) <= max(p2a[1], p2b[1]) and
                min(p2a[1], p2b[1]) <= max(p1a[1], p1b[1]))

    slope1 = (point1b[1] - point1a[1]) / (point1b[0] - point1a[0]) if (point1b[0] - point1a[0]) != 0 else float('inf')
    slope2 = (point2b[1] - point2a[1]) / (point2b[0] - point2a[0]) if (point2b[0] - point2a[0]) != 0 else float('inf')
       
    
    # If adjacent sides, we want them to count as visible, even though they share a endpoint, 
    # so count as no intersection
    if num1a==num2a or num1a==num2b or num1b==num2a or num1b==num2b :
        return False
        
    if slope1 != slope2:
        if slope1 != float('inf') and slope2 != float('inf') :
            
            oao1 = point1b[1]-point1b[0]*slope1
            oao2 = point2b[1]-point2b[0]*slope2
            intersect_point_x = (oao1 - oao2) / (slope2 - slope1)
            intersect_point_y = slope1*intersect_point_x + oao1
        
        # Cases where one of the sides is vertical
        elif slope1 == float('inf') :
            intersect_point_x = point1a[0]
            oao2 = point2b[1]-point2b[0]*slope2
            intersect_point_y = slope2*intersect_point_x + oao2    
        else :
            intersect_point_x = point2a[0]
            oao1 = point1b[1]-point1b[0]*slope1
            intersect_point_y = slope1*intersect_point_x + oao1
        
        #print('intersect', intersect_point_x, intersect_point_y)
        #print('strict', strict)
            
        if on_segment(point1a, [intersect_point_x, intersect_point_y], point1b) and on_segment(point2a, [intersect_point_x, intersect_point_y], point2b) :
            #print(point1a, point1b, point2a, point2b)
            return True
        
        return False
    
    elif slope1 == slope2 :
        if slope1!=float('inf'):
            oao1 = point1b[1]-point1b[0]*slope1
            oao2 = point2b[1]-point2b[0]*slope2
            if oao1!=oao2 :
                return False
            return True
        return are_collinear_and_overlap(point1a, point1b, point2a, point2b)
    
    else:
        return False
    
def get_visible_vertices(point, num_obs, nodes, vertices, sides) :
    """
    args   : point, the numero of the node we're interested in,
             num_obs, the numero of the obstacle this node belongs to,
             nodes, the list of other nodes, 
             vertices, the coordinates corresponding to the nodes,
             sides, the sides of obstacles,
    return : visible_vertices, a list of the vertices visible from point 'point'.
    """
    visible_vertices = []
    # for all group/element that has nodes
    for obs_idx in range(len(nodes)) :
        # for nodes of the same group, we want visibility only if adjacent
        if obs_idx == num_obs :
            local_nodes = nodes[obs_idx]
            first_node = local_nodes[0]
            if len(local_nodes)==2 :
                visible_vertices.append(first_node+(point-first_node+1)%len(local_nodes))
            elif len(local_nodes)>=2 :
                visible_vertices.append(first_node+(point-first_node-1)%len(local_nodes))
                visible_vertices.append(first_node+(point-first_node+1)%len(local_nodes))
        else :
            # for all node in other groups, want to check if visible
            for node in nodes[obs_idx] :
                visible = True
                # check if when drawing segment between point and node, we cross a side of obstacle
                for obs in range(1, len(nodes)-1) :
                    for side in sides[obs-1] :
                        if intersect(side[0], side[1], point, node, vertices) :
                            visible = False
                            break
                if visible :
                        visible_vertices.append(node)

    return visible_vertices


def get_edges_and_weights(nodes, vertices, sides, verbose=False) :
    """
    args   : nodes, the list of all nodes, 
             vertices, the coordinates corresponding to the nodes,
             sides, the sides of obstacles
    return : edges, a list of the edges to put on the graph,
             weights, their associated weights (euclidian distances).
    """
    edges = []
    weights = []
    node = 0
    # for all group/element that has nodes (obstacle or start or goal)
    if verbose :
        print("Visible vertices from each node :")
    for num_group in range(len(nodes)) :
        # for all node of this group
        for num_node in range(len(nodes[num_group])) :
            # find its visible neighbors
            visible_vert = get_visible_vertices(node, num_group, nodes, vertices, sides)
            if verbose :
                print(node, visible_vert)
            # add these as edges with associated weights
            for vertex in visible_vert :
                edges.append(tuple([node, vertex]))
                distance = np.sqrt((vertices[node][0]-vertices[vertex][0])**2+(vertices[node][1]-vertices[vertex][1])**2)
                weights.append(distance)
            node += 1
    if verbose :
        print("\n")
    return edges, weights

def create_graph(nodes, edges, weights, verbose=False) :
    G = nx.Graph()
    
    all_nodes = []
    for group in nodes :
        for node in group :
            all_nodes.append(node)
    
    round_weights = []
    uniq_edges = []
    for edge in range(len(edges)) :
        if edges[edge][1]>edges[edge][0] :
            uniq_edges.append(edges[edge])
            round_weights.append(round(weights[edge], 2))
    G.add_nodes_from(all_nodes)
    for edge, weight in zip(uniq_edges, round_weights):
        G.add_edge(edge[0], edge[1], weight=weight)
    pos = nx.spring_layout(G)  # Define the layout for the graph nodes
    if verbose :
        print("Weighted graph :")
        nx.draw(G, pos, with_labels=True, node_size=500, node_color='skyblue', font_weight='bold')
    
        # Add edge labels with weights
        labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    
        plt.title('Weighted Graph')
        plt.show()
    return G

