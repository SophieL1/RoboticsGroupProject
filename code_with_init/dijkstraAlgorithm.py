# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 18:04:08 2023

@author: Lenovo
"""

# dependencies for our dijkstra's implementation
from queue import PriorityQueue
from math import inf
# graph dependency  
import networkx as nx


"""Dijkstra's shortest path algorithm"""
def dijkstra(graph: 'networkx.classes.graph.Graph', start: str, end: str, verbose=False) -> 'List':
    """Get the shortest path of nodes by going backwards through prev list
    credits: https://github.com/blkrt/dijkstra-python/blob/3dfeaa789e013567cd1d55c9a4db659309dea7a5/dijkstra.py#L5-L10"""
    def backtrace(prev, start, end):
        node = end
        path = []
        while node != start:
            path.append(node)
            node = prev[node]
        path.append(node) 
        path.reverse()
        return path
        
    """get the cost of edges from node -> node
    cost(u,v) = edge_weight(u,v)"""
    def cost(u, v):
        return graph.get_edge_data(u,v).get('weight')
        
    """main algorithm"""
    # predecessor of current node on shortest path 
    prev = {} 
    # initialize distances from start -> given node i.e. dist[node] = dist(start, node)
    dist = {v: inf for v in list(nx.nodes(graph))} 
    # nodes we've visited
    visited = set() 
    # prioritize nodes from start -> node with the shortest distance!
    ## elements stored as tuples (distance, node) 
    pq = PriorityQueue()  
    
    dist[start] = 0  # dist from start -> start is zero
    pq.put((dist[start], start))
    
    while 0 != pq.qsize():
        curr_cost, curr = pq.get()
        visited.add(curr)
        if verbose :
            print(f'visiting {curr}')
        # look at curr's adjacent nodes
        for neighbor in dict(graph.adjacency()).get(curr):
            # if we found a shorter path 
            path = dist[curr] + cost(curr, neighbor)
            if path < dist[neighbor]:
                # update the distance, we found a shorter one!
                dist[neighbor] = path
                # update the previous node to be prev on new shortest path
                prev[neighbor] = curr
                # if we haven't visited the neighbor
                if neighbor not in visited:
                    # insert into priority queue and mark as visited
                    visited.add(neighbor)
                    pq.put((dist[neighbor],neighbor))
                # otherwise update the entry in the priority queue
                else:
                    # remove old
                    _ = pq.get((dist[neighbor],neighbor))
                    # insert new
                    pq.put((dist[neighbor],neighbor))
    if verbose :
        print("=== Dijkstra's Algo Output ===")
        print("Distances")
        print(dist)
        print("Visited")
        print(visited)
        print("Previous")
        print(prev)
    # we are done after every possible path has been checked 
    return backtrace(prev, start, end), dist[end]