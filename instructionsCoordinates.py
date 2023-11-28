# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 19:31:13 2023

@author: Lenovo
"""

import math
import numpy as np

def get_wheretogo_coordinates(path, cost, vertices) :
    """
    args : path, sequence of vertices to visit
           cost, total distance to go
           vertices, list of coordinates of vertices
    return : coords, list of coordinates of points we want to go through
    """
    coords = []
    for node in range(len(path)-1) :
        coords.append([vertices[path[node]][0], vertices[path[node]][1]])
    
    return coords
    

def get_lengths_and_angles(path, cost, vertices, robot_dir0) :
    """
    args : path, sequence of vertices to visit
           cost, total distance to go
           vertices, list of coordinates of vertices
           robot_dir0, angle between the robot axis and the horizontal line
               (as would be in trigonometric circle)
    return : lengths, list of lengths of segments to follow
             angles, list of absolute angles of segments to follow
    """
    def distance(node1, node2) :
        return np.sqrt((vertices[node1][0]-vertices[node2][0])**2+(vertices[node1][1]-vertices[node2][1])**2)
    
    def absolute_angle(node1, node2):
        x1 = vertices[node1][0]
        y1 = vertices[node1][1]
        x2 = vertices[node2][0]
        y2 = vertices[node2][1]
        angle = math.atan2(y2 - y1, x2 - x1)
        return math.degrees(angle)  # Convert radians to degrees    
    
    lengths = []
    angles = [robot_dir0]
    for node in range(len(path)-1) :
        dist = distance(path[node], path[node+1])
        angle = absolute_angle(path[node], path[node+1])
        lengths.append(dist)
        angles.append(angle)
    
    turning_angles = []
    for i in range(len(angles)-1) :
        turning_angles.append(angles[i] - angles[i+1])
    
        
    return lengths, turning_angles