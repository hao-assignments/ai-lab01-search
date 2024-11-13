import numpy as np
from collections import deque
import heapq
import math

def BFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO: 
   
    path = []
    visited = {}
    queue = deque([start])
    
    # Trace of each visited vertex and the previous vertex
    visited[start] = None
    
    while queue:
        node = queue.popleft()
        if node == end:
            break
        
        for neighbor in range(len(matrix[node])):
            if matrix[node][neighbor] > 0 and neighbor not in visited:
                visited[neighbor] = node
                queue.append(neighbor)
    
    # Iterate back from the destination vertex to the starting vertex to find the path.
    current = end
    while current is not None:
        path.insert(0, current)
        current = visited[current]
    
    return visited, path

def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO:   
    path = []
    visited = {}
    
    def dfs_recursive(node):
        if node == end:
            return True
        for neighbor in range(len(matrix[node])):
            if matrix[node][neighbor] > 0 and neighbor not in visited:
                visited[neighbor] = node
                if dfs_recursive(neighbor):
                    return True
        return False
    
    visited[start] = None
    dfs_recursive(start)
    
    # Create a path from the destination vertex to the starting vertex
    current = end
    while current is not None:
        path.insert(0, current)
        current = visited[current]
    
    return visited, path

def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    path=[]
    visited={}
    # Priority queue for the frontier, order by the path_cost
    num_nodes = len(matrix)
    prio_queue = []
    heapq.heappush(prio_queue,(0, start))

    # Dictionary to store the cost of the shortest  path to each node
    cost_so_far = {start:0}

    # Dictionary to store the path
    visited[start] = None
    while prio_queue:
        # Get the node with the lowest cost
        current_cost, current_node = heapq.heappop(prio_queue)
        if current_node == end:
            while current_node is not None:
                path.append(current_node)
                current_node = visited[current_node]
            path.reverse()
            return visited, path
    
        # Expand the current node
        for neighbor in range (num_nodes):
            if matrix[current_node][neighbor]!=0 :
                new_cost = current_cost + matrix[current_node][neighbor]
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(prio_queue, (new_cost,neighbor))
                    visited[neighbor] = current_node
    return  visited, path

def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path=[]
    visited={}
    # Priority queue for the frontier, order by the path_cost
    num_nodes = len(matrix)
    prio_queue = []
    heapq.heappush(prio_queue,(0, start))

    # Dictionary to store the cost of the shortest to path to each node

    # Dictionary to store the path
    visited[start] = None
    while prio_queue:
        # Get the node with the lowest cost
        current_cost, current_node = heapq.heappop(prio_queue)

        if current_node == end:
            while current_node is not None:
                path.append(current_node)
                current_node = visited[current_node]
            path.reverse()
            return visited, path
    
        # Expand the current node
        for neighbor in range (num_nodes):
            if matrix[current_node][neighbor]!=0 and neighbor not in visited :
                heapq.heappush(prio_queue, (matrix[current_node][neighbor],neighbor))
                visited[neighbor] = current_node
    return  visited, path

def Astar(matrix, start, end, pos):
    """     
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    def euclid_distance (a,b):
        return math.sqrt((pos[a][0] - pos[b][0]) ** 2 + (pos[a][1] - pos[b][1]) ** 2)
    path=[]
    visited={}
    # Priority queue for the frontier, order by the path_cost
    num_nodes = len(matrix)
    prio_queue = []
    heapq.heappush(prio_queue,(euclid_distance(start, end), start))

    # Dictionary to store the cost of the shortest  path of each node 
    cost_so_far = {start:euclid_distance(start,end)}

    # Dictionary to store the path
    visited[start] = None
    while prio_queue:
        # Get the node with the lowest cost
        current_cost, current_node = heapq.heappop(prio_queue)

        if current_node == end:
            while current_node is not None:
                path.append(current_node)
                current_node = visited[current_node]
            path.reverse()
            return visited, path
    
        # Expand the current node
        for neighbor in range (num_nodes):
            if matrix[current_node][neighbor]!=0 :
                new_cost = cost_so_far[current_node] + matrix[current_node][neighbor]
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority =  euclid_distance(neighbor, end) + new_cost
                    heapq.heappush(prio_queue, (priority,neighbor))
                    visited[neighbor] = current_node
    return  visited, path