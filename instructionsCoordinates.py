# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 19:31:13 2023

@author: Lenovo
"""

import math
import numpy as np

def get_vertices_from_uniqpoint(pos_goal, pos_Thymio, width_goal, widths_Thymio) : 
    """
    args :  - pos_goal, list of coordinates of center pt of goal
            - pos_Thymio, list of coordinates of center pt of Thymio
            - width_goal, int distance from center to side of the goal square
            - widths_Thymio = [forward, left, back, right], list of distances from Thymio pt
    return : - thymio4verts, 
             - goal4verts.
        
    """
    def rotate_rectangle(rectangle_vertices, alpha, point_P):
        # Convert alpha to radians for trigonometric functions
        alpha_rad = math.radians(alpha)
    
        # Coordinates of point P
        Px, Py = point_P
    
        rotated_vertices = []
        for vertex in rectangle_vertices:
            # Translate the vertex to the origin (subtracting point P coordinates)
            translated_x = vertex[0] - Px
            translated_y = vertex[1] - Py
    
            # Apply rotation transformation
            new_x = translated_x * math.cos(alpha_rad) - translated_y * math.sin(alpha_rad)
            new_y = translated_x * math.sin(alpha_rad) + translated_y * math.cos(alpha_rad)
    
            # Translate the vertex back to its original position (add point P coordinates)
            rotated_x = new_x + Px
            rotated_y = new_y + Py
    
            rotated_vertices.append((rotated_x, rotated_y))
    
        return rotated_vertices
    
    thymio4verts = []
    goal4verts = []
    forward, left, back, right = widths_Thymio
    
    xThymio = pos_Thymio[0]
    yThymio = pos_Thymio[1]
    anglethymio = pos_Thymio[2]
    thymio4verts.append([xThymio+forward, yThymio-left])
    thymio4verts.append([xThymio+forward, yThymio+right])
    thymio4verts.append([xThymio-back, yThymio+right])
    thymio4verts.append([xThymio-back, yThymio-left])
    
    thymio4verts = rotate_rectangle(thymio4verts, anglethymio, [xThymio, yThymio]) 
    
    xGoal = pos_goal[0]
    yGoal = pos_goal[1]
    
    goal4verts.append([xGoal+width_goal, yGoal-width_goal])
    goal4verts.append([xGoal+width_goal, yGoal+width_goal])
    goal4verts.append([xGoal-width_goal, yGoal+width_goal])
    goal4verts.append([xGoal-width_goal, yGoal-width_goal])
    
    return thymio4verts, goal4verts
    

def get_wheretogo_coordinates(path, vertices) :
    """
    args : path, sequence of vertices to visit
           cost, total distance to go
           vertices, list of coordinates of vertices
    return : coords, list of coordinates of points we want to go through
    """
    coords = []
    for node in range(len(path)) :
        coords.append([vertices[path[node]][0], vertices[path[node]][1]])
    
    return coords
    

def get_lengths_and_angles(path, vertices, robot_dir0) :
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