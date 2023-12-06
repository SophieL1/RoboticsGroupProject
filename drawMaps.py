# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 17:14:19 2023

@author: Lenovo
"""
import matplotlib.pyplot as plt


def plot_map(obstacle_vertices, xAxeLim, yAxeLim ,larger=False, larger_obs=None, start=None, start_pt=None, goal=None, graph=False, graph_edges=None, vertices=None):
    plt.figure(figsize=(4, 4))
    
    i = 1
    for obstacle in obstacle_vertices :
        x_values = [vertex[0] for vertex in obstacle]
        y_values = [vertex[1] for vertex in obstacle]
        # Close the obstacle shape by adding the first vertex at the end
        x_values.append(obstacle[0][0])
        y_values.append(obstacle[0][1])
        
        plt.plot(x_values, y_values, 'k-')  # Plot obstacle boundary
        
        # Add numbers indicating vertex positions
        if graph :
            for vertex in obstacle:
                plt.text(vertex[0], vertex[1], str(i), color='blue', fontsize=8, ha='right', va='bottom')
                i = i+1


    
    if larger :
        for obstacle in larger_obs :
            x_values = [vertex[0] for vertex in obstacle]
            y_values = [vertex[1] for vertex in obstacle]
            # Close the obstacle shape by adding the first vertex at the end
            x_values.append(obstacle[0][0])
            y_values.append(obstacle[0][1])

            plt.plot(x_values, y_values, 'r-')  # Plot obstacle boundary
    
    if graph :
        for edge in graph_edges :
            x_values = [(vertices[vertex])[0] for vertex in edge]
            y_values = [(vertices[vertex])[1] for vertex in edge]
            
            plt.plot(x_values, y_values, 'm--', linewidth=0.5)
            
    if start is not None :
        x_start = [vertex[0] for vertex in start]
        y_start = [vertex[1] for vertex in start]
        # Close the start position shape by adding the first vertex at the end
        x_start.append(start[0][0])
        y_start.append(start[0][1])
        x_start_mid = start_pt[0]
        y_start_mid = start_pt[1]

        plt.plot(x_start, y_start, 'b-')  # Plot start boundary
        plt.plot(x_start_mid, y_start_mid, 'bo') # Plot center point of Thymio robot
        
        if graph :
            plt.text(start_pt[0], start_pt[1], str(0), color='blue', fontsize=8, ha='right', va='bottom')
        
    if goal is not None :
        x_goal = [vertex[0] for vertex in goal]
        y_goal = [vertex[1] for vertex in goal]
        # Close the goal position shape by adding the first vertex at the end
        x_goal.append(goal[0][0])
        y_goal.append(goal[0][1])
        x_goal_mid = sum(x_goal[:-1])/len(x_goal[:-1])
        y_goal_mid = sum(y_goal[:-1])/len(y_goal[:-1])

        plt.plot(x_goal, y_goal, 'y-')  # Plot goal boundary
        plt.plot(x_goal_mid, y_goal_mid, 'yo') # Plot center point of Thymio's goal
        
        if graph :
            plt.text(x_goal_mid, y_goal_mid, str(i), color='blue', fontsize=8, ha='right', va='bottom')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    if not larger :
        plt.title('Map with obstacles')
    else :
        plt.title('Map with larger obstacles,\n expanded for safety')
    plt.grid(True)
    plt.axis('equal')  # Set equal aspect ratio
    plt.xlim(-0.5, xAxeLim)
    plt.ylim(-0.5, yAxeLim)
    
    plt.gca().invert_yaxis()  # Invert y-axis
    plt.show()