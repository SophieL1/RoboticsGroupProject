# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 17:22:56 2023

@author: Lenovo
"""
import numpy as np
from shapely.geometry import Polygon


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