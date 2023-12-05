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
from processObstacles import expand_obstacles, sort_obstacle_vertices
from VGraph import get_nodes_and_sides, get_edges_and_weights, create_graph
from dijkstraAlgorithm import dijkstra
from instructionsCoordinates import get_lengths_and_angles, get_wheretogo_coordinates, get_vertices_from_uniqpoint


def findPath(obs_list, start_point, robot_dir0, goal_point, width, width_goal, thymio_dimensions, xlim=850, ylim=800, verbose=True) :
    xLimMap = xlim
    yLimMap = ylim
    
    # First processing of the obstacles to make sure vertices are ordered
    obs_list = sort_obstacle_vertices(obs_list)
    
    # Representing Thymio and goal by their 4 vertices
    pos_Thymio = [start_point[0], start_point[1], robot_dir0]
    width_goal = width_goal
    widths_Thymio = thymio_dimensions
    start_pos, goal_pos = get_vertices_from_uniqpoint(goal_point, pos_Thymio, width_goal, widths_Thymio)
    
    # Plot the map with obstacles
    if verbose :
        print('Schematic map :')
        plot_map(obs_list, xLimMap, yLimMap, start=start_pos, start_pt=start_point, goal=goal_pos)
        
    # Expand obstacles (ideally by at least half of Thymio robot's width)
    expanded_obs = expand_obstacles(obs_list, width) # expanded_obs is now a list of #obs lists of #verticesofthisobs tuples
    if verbose :
        print('Schematic map with expanded obstacles :')
        plot_map(obs_list, xLimMap, yLimMap, True, expanded_obs, start=start_pos, start_pt=start_point, goal=goal_pos)
    
    myNodes, myVertices, mySides = get_nodes_and_sides(expanded_obs, start_point, goal_point)
     
    
    myEdges, myWeights = get_edges_and_weights(myNodes, myVertices, mySides, verbose=verbose)
    
    if verbose :
        print("Map with visibility visualisation :\n (violet lines show visibility relations)")
        plot_map(expanded_obs, xLimMap, yLimMap, larger=False, larger_obs=None, 
             start=start_pos, start_pt=start_point, goal=goal_pos,
             graph=True, graph_edges=myEdges, vertices=myVertices)
    
    myGraph = create_graph(myNodes, myEdges, myWeights, verbose=verbose)
    
    goal_idx = myNodes[-1][0]
    myPath = nx.dijkstra_path(myGraph, 0, goal_idx, weight='weight')
    if verbose :
        print('Path nodes :', myPath)
        
    # Getting coordinates of the points we want to go through
    myCoordinates = get_wheretogo_coordinates(myPath, myVertices)
    if verbose :
        print("Coordinates of points of the path :")
        print(myCoordinates)
        
    # Getting the lengths of segments of path, and the angles between them.
    # Negative angle means turn to the left, positive means turn to the left (in degree).
    myLengths, myAngles = get_lengths_and_angles(myPath, myVertices, robot_dir0)
    if verbose :
        print("Lengths to go forward and angles to rotate :")
        print(myLengths, '\n', myAngles)
        
    initialPosition = [myCoordinates[0][0], myCoordinates[0][1], robot_dir0]
        
    return initialPosition, expanded_obs, myCoordinates[1:]