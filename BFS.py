import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from queue import PriorityQueue
import math
from heapq import *
import heapq
from networkx import NetworkXError
import networkx as nx
from aa import astar 


def ucs(graph, origin, goal):
    frontier = [] # heapq
    frontierIndex = {} # dict that saves node details in the following format {Key(node label): weight , [Path to the key]} 
    node = (0,origin, [origin])  #tuple (cost, cur_node, list[the path to the current node])
    Visited = set()

    # stores the current node and the cost of the path to it and the visited nodes to reach that node
    frontierIndex[node[1]] = [node[0], node[2]]
    
 
    
    #push the node inside the frontier list 
    heapq.heappush(frontier, node)

    while frontier:
        if len(frontier) == 0:
            return None
        
        #heapq always has the lowest value at index 0 
        node = heapq.heappop(frontier)
        
        # Delete from the dicitonary the key of the element that has been popped
        del frontierIndex[node[1]]
        
        # Check if the solution has been found by comparing the node key with the goal
        if node[1] == goal:
            return node[2]
        #if the key != the goal add the current key to the visited list     
        Visited.add(node[1])
        
        # Get a list of all the child nodes of the current key
        neighbours = list(graph.neighbors(node[1]))

        path = node[2]

        for child in neighbours:

            path.append(child)
            
            # create the child node that will be inserted in frontier

            childNode = (node[0] + graph.get_edge_data(node[1], child)["weight"], child, path)
            
            # Check the child node is not Visited and not in frontier thorugh the dictionary
            if child not in Visited and child not in frontierIndex:
                heapq.heappush(frontier, childNode)
                frontierIndex[child] = [childNode[0], childNode[2]]

            elif child in frontierIndex:
                # Checks if the child node that has already been visited has a lower path cost than the node already in frontier
                if childNode[0] < frontierIndex[child][0]:
                
                    nodeToRemove = (frontierIndex[child][0], child, frontierIndex[child][1])
                    frontier.remove(nodeToRemove)
                    heapq.heapify(frontier)
                    del frontierIndex[child]

                    heapq.heappush(frontier, childNode)
                    frontierIndex[child] = [childNode[0], childNode[2]]
            path = path[:-1]

    


def bfs(graph,StartNode, goal):
    Visited = {node: False for node in graph.nodes}
    Queue = [StartNode]
    Visited[StartNode]=True
    Result = []
    while Queue:
        cur_node = Queue.pop(0)
        Result.append(cur_node)
        if cur_node == goal:
            break
        for node in graph.neighbors(cur_node):
            if not Visited[node]:
                Queue.append(node)
                Visited[node] = True
    return Result




def dfs(visited,graph,Result, node,goal):
    if node not in visited and goal not in visited:
        Result.append(node)
        visited.add(node)
        for neighbour in graph[node]:
            dfs(visited, graph, Result,neighbour,goal)
    return Result


def dist(a, b):
    x1,y1=a
    x2,y2=b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

                
if __name__ == '__main__':



    Result = []
    visited = set()
    G = nx.DiGraph()

    G.add_node("S")
    G.add_node("3")
    G.add_node("1")
    G.add_node("2")
    G.add_node("4")
    G.add_node("5")
    G.add_node("G")

    G.nodes["1"]['pos'] = (0, 1)
    G.nodes["2"]['pos'] = (2, 2)
    G.nodes["3"]['pos'] = (2, -2)
    G.nodes["4"]['pos'] = (5, 2)
    G.nodes["5"]['pos'] = (5, -2)
    G.nodes["S"]['pos'] = (7, 0)
    G.nodes["G"]["pos"] = (10, 0)
    
    G.add_edge('S','1', weight=2)   
    G.add_edge('S','3', weight=5)
    G.add_edge('3','1', weight=5)
    G.add_edge('3','G', weight=6)
    G.add_edge('3','4', weight=2)
    G.add_edge('1','G', weight=1)
    G.add_edge('2','1', weight=4)
    G.add_edge('4','2', weight=4)
    G.add_edge('4','5', weight=3)
    G.add_edge('5','G', weight=3)
    G.add_edge('5','2', weight=6)

    bfspath = bfs(G,"S", "G")
    dfspath = dfs(visited, G,Result, "S", "5")
    ucspath = ucs(G, 'S', '5')
    astarpath = astar(G, "S", "G", weight="weight")
    print("bfs", bfspath)
    print("dfs ", dfspath)
    print("ucs", ucspath)
    print("astar ",astarpath) 
    dfsx = nx.depth_first_search.dfs_tree(G, "G")

    
    sp = bfspath

    # The positions of each node are stored in a dictionary
    node_pos = nx.get_node_attributes(G, 'pos')
    # The edge weights of each arcs are stored in a dictionary
    arc_weight = nx.get_edge_attributes(G, 'weight')
    # Create a list of arcs in the shortest path using the zip command and store it in red edges
    red_edges = list(zip(sp, sp[1:]))
    # If the node is in the shortest path, set it to red, else set it to white color
    node_col = ['blue' if not node in sp else 'green' for node in G.nodes()]
    # If the edge is in the shortest path set it to red, else set it to white color
    edge_col = ['black' if not edge in red_edges else 'green' for edge in G.edges()]
    # Draw the nodes
    nx.draw_networkx(G, node_pos, node_color=node_col)
    # Draw the node labels
    #nx.draw_networkx_labels(G, node_pos,node_color= node_col)
    # Draw the edges
    nx.draw_networkx_edges(G, node_pos, edge_color=edge_col)
    # Draw the edge labels
    nx.draw_networkx_edge_labels(G, node_pos, edge_labels=arc_weight)
    # Remove the axis
    plt.axis('off')
    # Show the plot
    plt.show()
   
    

