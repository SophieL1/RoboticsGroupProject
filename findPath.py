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
from instructionsCoordinates import get_lengths_and_angles, get_wheretogo_coordinates


def findPath(obs_list, start_pos, robot_dir0, goal_pos, width, verbose=True) :
    xLimMap = 605
    yLimMap = 570
    
    # Representing start and goal by middle point
    start_point = tuple([sum([row[0] for row in start_pos])/len(start_pos), sum([row[1] for row in start_pos])/len(start_pos)])
    goal_point = tuple([sum([row[0] for row in goal_pos])/len(goal_pos), sum([row[1] for row in goal_pos])/len(goal_pos)])
    
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