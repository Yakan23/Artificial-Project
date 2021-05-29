from networkx.algorithms.shortest_paths.weighted import _weight_function
from networkx import NetworkXError
import networkx as nx

import matplotlib.pyplot as plt

from heapq import *
import heapq

from itertools import count




                                        ######## Astar search algorithm #########

"""
    Description
    -----------
    At each iteration of its main loop, A* needs to determine which of its paths to extend. 
    It does so based on the cost of the path and an estimate of the cost required to extend the path all the way to the goal. 
    Specifically, A* selects the path that minimizes f(n)=g(n)+h(n).


    Parameters
    ----------
    G : NetworkX graph

    start : node
       Starting node for path

    goal : node
       Ending node for path

    heuristic h(n) : function
       A function to evaluate the estimate of the distance
       from the a node to the target.  The function takes
       two nodes arguments and must return a number.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.
"""



def astar(G, start, goal, heuristic=None, weight="weight"):
    
    if heuristic is None:
         # If heuristic is none then it works as Dijkstra's algorithm
        def heuristic(u, v):
            return 0
    
    
    
    #renaming heapq: heappush, heappop functions
    push = heappush
    pop = heappop
    
    #getting the weights of the graph edges using the built in weight function 
    weight = _weight_function(G, weight)
    
    
    #iterator 
    c = count()
    queue = [(0, next(c), start, 0, None)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to goal. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the start.
    explored = {}

    while queue:
        print(queue)
        # Pop the smallest item from queue.
        _, __, curnode, dist, parent = pop(queue)

        if curnode == goal:
            
            #add the current node to a list 
            path = [curnode]
            node = parent
            
            # Traverses back in the path until we reach the start node adding 
            # each node to the path list
            while node is not None:
                path.append(node)
                node = explored[node]
                
            # upon reaching the start node we have the path from goal to start
            # we reverse the path list and return the path from start to goal
            path.reverse()
            return path

        if curnode in explored:
            # Do not override the parent of starting node
            if explored[curnode] is None:
                continue

            # Skip bad paths that were enqueued before finding a better one
            qcost, h = enqueued[curnode]
            if qcost < dist:
                continue

        explored[curnode] = parent

        for neighbor in G.neighbors(curnode):
            ncost = dist + (G[curnode][neighbor]["weight"])
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                # if qcost <= ncost, a less costly path from the
                # neighbor to the start was already determined.
                # Therefore, we won't attempt to push this neighbor
                # to the queue
                if qcost <= ncost:
                    continue
            else:
                h = heuristic(neighbor, goal)
            enqueued[neighbor] = ncost, h
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode))

    raise nx.NetworkXNoPath(f"Node {goal} not reachable from {start}")




                            ################## Greedy Best First Search ########################

"""
    Description
    -----------
    Using a greedy algorithm, expand the first successor of the parent. After a successor is generated:
        
    1- If the successor's heuristic is better than its parent, 
       the successor is set at the front of the queue (with the parent reinserted directly behind it), and the loop restarts.
       
    2-Else, the successor is inserted into the queue (in a location determined by its heuristic value).
      The procedure will evaluate the remaining successors (if any) of the parent.
  

    Parameters
    ----------
    G : NetworkX graph

    start : node
       Starting node for path

    goal : node
       Ending node for path

    heuristic h(n) : function
       A function to evaluate the estimate of the distance
       from the a node to the target.  The function takes
       two nodes arguments and must return a number.

"""



def GBFS(G, start, goal, heuristic=None):
    
    if heuristic is None:
        # If heuristic is none then it works as Dijkstra's algorithm
        def heuristic(u, v):
            return 0

    push = heappush
    pop = heappop

    c = count()
    #priotiy, counter, node , parent
    queue = [(0, next(c), start, None)]
    
    
    # Maps enqueued nodes to  the computed heuristics to goal.
    # We avoid computing the heuristics more than once 
    # and inserting the node into the queue too many times.
    enqueued = {}
    
    # Maps explored nodes to parent closest to the start.
    explored = {}
    
    while queue:
        print(queue)
        # Pop the smallest item from queue.
        _,__, curnode, parent = pop(queue)

        

        if curnode == goal:
            
            #add the current node to a list 
            path = [curnode]
            node = parent
            
            # Traverses back in the path until we reach the start node adding 
            # each node to the path list
            while node is not None:
                path.append(node)
                node = explored[node]
                
            # upon reaching the start node we have the path from goal to start
            # we reverse the path list and return the path from start to goal
            path.reverse()
            return path
        
        
        if curnode in explored:
            if explored[curnode] is None:
                continue

        explored[curnode] = parent

        for neighbor in G.neighbors(curnode):
        
            if neighbor in enqueued:
                continue 
            else:
                h = heuristic(neighbor, goal)


            enqueued[neighbor] = h
            push(queue, (h, next(c), neighbor, curnode))
            

    raise nx.NetworkXNoPath(f"Node {goal} not reachable from {start}")
 
    
 
 
                                    ############## Uniform Cost Search ################
    
"""
    Description
    -----------
    Uniform-cost search is an uninformed search algorithm that uses the lowest cumulative cost to find a path from the source to the destination. 
    Nodes are expanded, starting from the root, according to the minimum cumulative cost. 
    The uniform-cost search is then implemented using a Priority Queue.

    Parameters
    ----------
    G : NetworkX graph

    start : node
       Starting node for path

    goal : node
       Ending node for path


"""

    

def ucs(graph, origin, goal):
    frontier = [] # heapq
    
    # Dict that saves node details in the following format {Key(node label): weight , [Path to the key]} 
    frontierIndex = {} 
    
    #tuple (cost, cur_node, list[the path to the current node])
    node = (0,origin, [origin])  
    
    Visited = set()

    # uses the current node as a key to map to it the cost of the path to it 
    # and the path to reach that node
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
            
            # create the child node that will be inserted in frontier by adding the curnode path cost 
            # to the weight of the edge from the curnode to the child
            # the child and the path to the child
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

    


# def bfsvisitednodes(graph,StartNode, goal):
#     Visited = {node: False for node in graph.nodes}
#     Queue = [StartNode]
#     Visited[StartNode]=True
#     Result = []
#     while Queue:
#         cur_node = Queue.pop(0)
#         Result.append(cur_node)
#         if cur_node == goal:
#             break
#         for node in graph.neighbors(cur_node):
#             if not Visited[node]:
#                 Queue.append(node)
#                 Visited[node] = True
#     return Result


"""
    Description
    -----------
    Depth-first search is an algorithm for traversing or searching tree or graph data structures.
    The algorithm starts at the given start node and explores as far as possible along each branch before backtracking.
    So the basic idea is to start form the given start node and mark the node and move to the adjacent unmarked node 
    and continue this loop until there is no unmarked adjacent node. Then backtrack and check for other unmarked nodes 
    and traverse them. Finally print the nodes in the path.

    Parameters
    ----------
    G : NetworkX graph

    start : node
       Starting node for path

    goal : node
       Ending node for path


"""



def dfs(graph, start, goal):
    #create a stack of tuples containing the current node and the path to reach that node
    stack = [(start, [start])]

    while stack:
        # pop the first element of the stack assigning the current node value to the var: vertex
        # and the path to the current node to var: path 
        (vertex, path) = stack.pop(0)
        
        #iterate over the children of the current node
        for neighbor in graph.neighbors(vertex):
            #check if the current neighbor is the goal if it is the goal then return the path to that node and add the node to the path
            if neighbor == goal:
                return [path + [neighbor]]
            
            # if the current neighbor is not the goal append that node and tha path to it in the stack 
            else:
                stack.append((neighbor, path + [neighbor]))





"""
    Description
    -----------
    Breadth-first search (BFS) is an algorithm for traversing or searching tree or graph data structures.
    It starts at the given start node and explores all of the neighbor nodes at the present depth prior to moving on to the nodes at the next depth level.
    It uses the opposite strategy of depth-first search, which instead explores the node branch as far as possible 
    before being forced to backtrack and expand other nodes.


    Parameters
    ----------
    G : NetworkX graph

    start : node
       Starting node for path

    goal : node
       Ending node for path


"""



def bfs(graph, start, goal):
    
    #llst of nodes
    frontier = []

    # dict that saves node details in the following format {Key(node label): [Path to the key]}
    frontierIndex = {}

    # tuple (cur_node, list[the path to the current node])
    node = (start, [start])
    Visited = set()

    #Maps the path to reach node x , to key = node x
    frontierIndex[node[0]] = [node[1]]

    #push the node inside the frontier list
    frontier.append(node)

    while frontier:
        if len(frontier) == 0:
                return None


        node = frontier.pop(0)

        # Delete from the dicitonary the key of the element that has been popped
        del frontierIndex[node[0]]

        # Check if the solution has been found by comparing the node key with the goal
        if node[0] == goal:
            return node[1]
    
        #if the key != the goal add the current key to the visited list
        Visited.add(node[0])

        # Get a list of all the child nodes of the current key
        neighbours = list(graph.neighbors(node[0]))
        
        #path is assigned the path to the current node
        path = node[1]

        for child in neighbours:

            path.append(child)

            # create the child node that will be inserted in frontier
            childNode = (child, path)

            # Check the child node is not Visited and not in frontier thorugh the dictionary
            if child not in Visited and child not in frontierIndex:
                frontier.append(childNode)
                frontierIndex[child] = [childNode[0], childNode[1]]
                    
            path = path[:-1]



    
    

   
    

