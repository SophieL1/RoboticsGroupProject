# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 15:56:02 2023

@author: Lenovo
"""
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
import networkx as nx
import holoviews as hv

from drawMaps import plot_map
from processObstacles import expand_obstacles
from VGraph import get_nodes_and_sides, get_edges_and_weights, create_graph
from dijkstraAlgorithm import dijkstra
from instructionsCoordinates import get_lengths_and_angles, get_wheretogo_coordinates, get_vertices_from_uniqpoint


def findPath(obs_list, start_point, robot_dir0, goal_point, width, width_goal, thymio_dimensions, xlim= 605, ylim=570, verbose=True) :
    xLimMap = xlim
    yLimMap = ylim
    
    # Representing Thymio and goal by their 4 vertices
    pos_Thymio = [start_point[0], start_point[1], robot_dir0]
    width_goal = width_goal
    widths_Thymio = thymio_dimensions
    start_pos, goal_pos = get_vertices_from_uniqpoint(goal_point, pos_Thymio, width_goal, widths_Thymio)
    
    # Plot the map with obstacles
    if verbose :
        plot_map(obs_list, xLimMap, yLimMap, start=start_pos, goal=goal_pos)
        
    # Expand obstacles (ideally by at least half of Thymio robot's width)
    expanded_obs = expand_obstacles(obs_list, width) # expanded_obs is now a list of #obs lists of #verticesofthisobs tuples
    plot_map(obs_list, xLimMap, yLimMap, True, expanded_obs, start=start_pos, goal=goal_pos)
    
    if verbose :
        for obs in range(len(expanded_obs)) :
            print('Obstacle ', obs)
            print(*[expanded_obs[obs]])
    
    myNodes, myVertices, mySides = get_nodes_and_sides(expanded_obs, start_point, goal_point)

    if verbose :
        print('Nodes, grouped by start-obs1-obs2-...-goal, are : ')
        print(myNodes)
        print('\n')
        
        print('Associated vertices are : ')
        for myVertex in myVertices :
            print('(', round(myVertex[0], 2), ',', round(myVertex[1], 2), ')')
        print('\n')
        
        #print(*myVertices)
        
        print('Sides of obstacles are : ')
        for obs in range(len(mySides)) :
            print('Obstacle ', obs)
            print(*[mySides[obs]])
     
    myEdges, myWeights = get_edges_and_weights(myNodes, myVertices, mySides, verbose=verbose)
    
    if verbose :
        plot_map(expanded_obs, xLimMap, yLimMap, larger=False, larger_obs=None, 
             start=start_pos, goal=goal_pos, 
             graph=True, graph_edges=myEdges, vertices=myVertices)
    
    myGraph = create_graph(myNodes, myEdges, myWeights, verbose=verbose)
    
    goal_idx = myNodes[-1][0]
    myPath, myCost = dijkstra(myGraph, 0, goal_idx, verbose=False)
    if verbose :
        print('Path nodes :', myPath, '\nPath cost :', myCost)
        
    # Getting coordinates of the points we want to go through
    myCoordinates = get_wheretogo_coordinates(myPath, myCost, myVertices)
    if verbose :
        print(myCoordinates)
        
    # Getting the lengths of segments of path, and the angles between them.
    # Negative angle means turn to the left, positive means turn to the left (in degree).
    myLengths, myAngles = get_lengths_and_angles(myPath, myCost, myVertices, robot_dir0)
    if verbose :
        print(myLengths, '\n', myAngles)
        
    return myLengths, myAngles