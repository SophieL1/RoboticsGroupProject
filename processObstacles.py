# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 17:22:56 2023

@author: Lenovo
"""
import numpy as np
import math
from shapely.geometry import Polygon

def calculate_centroid(vertices):
    centroid_x = sum(x for x, _ in vertices) / len(vertices)
    centroid_y = sum(y for _, y in vertices) / len(vertices)
    return centroid_x, centroid_y

def sort_obstacle_vertices(obstacle_vertices):
    """
    args : list of list of lists of coordinates of obstacles' vertices (in random order). Obstacles are supposed convex.
    return : list of list of lists of coordinates of obstacles' vertices, 
                ordered counterclockwise.
    """
    sorted_obstacles = []
    
    for obstacle in obstacle_vertices:
        centroid = calculate_centroid(obstacle)
        # Calculate angles of vertices with respect to centroid
        angles = [math.atan2(vertex[1] - centroid[1], vertex[0] - centroid[0]) for vertex in obstacle]
        # Combine vertices and angles into a list of tuples
        vertices_with_angles = list(zip(obstacle, angles))
        # Sort vertices based on angles in counterclockwise order
        sorted_vertices = [vertex[0] for vertex in sorted(vertices_with_angles, key=lambda x: x[1])]
        sorted_obstacles.append(sorted_vertices)

    return sorted_obstacles

def expand_obstacles(obstacle_vertices, border_width):
    expanded_obstacles = []

    for obstacle in obstacle_vertices:
        # Create a Shapely Polygon from the obstacle vertices
        obstacle_polygon = Polygon(obstacle)
        # Buffer the polygon to expand it by the border width
        expanded_polygon = obstacle_polygon.buffer(border_width, join_style=2)

        # Extract the coordinates of the expanded polygon
        expanded_obstacle = list(expanded_polygon.exterior.coords)
        # Remove the duplicate last vertex (which equals the first)
        expanded_obstacle.pop()

        expanded_obstacles.append(expanded_obstacle)

    return expanded_obstacles