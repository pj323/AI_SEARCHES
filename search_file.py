import igraph
def read30node():
    f = open("50test.txt", "r")
    return f


class Graph:
    # Initialize the class
    def __init__(self, graph_dict=None, directed=True):
        self.graph_dict = graph_dict or {}
        self.directed = directed
        if not directed:
            self.make_undirected()
    # Create an undirected graph by adding symmetric edges
    def make_undirected(self):
        for a in list(self.graph_dict.keys()):
            for (b, dist) in self.graph_dict[a].items():
                self.graph_dict.setdefault(b, {})[a] = dist
    # Add a link from A and B of given distance, and also add the inverse link if the graph is undirected
    def connect(self, A, B, distance=1):
        self.graph_dict.setdefault(A, {})[B] = distance
        if not self.directed:
            self.graph_dict.setdefault(B, {})[A] = distance
    # Get neighbors or a neighbor
    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)
    # Return a list of nodes in the graph
    def nodes(self):
        s1 = set([k for k in self.graph_dict.keys()])
        s2 = set([k2 for v in self.graph_dict.values() for k2, v2 in v.items()])
        nodes = s1.union(s2)
        return list(nodes)
# This class represent a node
class Node:
    # Initialize the class
    def __init__(self, name:str, parent:str):
        self.name = name
        self.parent = parent
        self.g = 0 # Distance to start node
        self.h = 0 # Distance to goal node
        self.f = 0 # Total cost
    # Compare nodes
    def __eq__(self, other):
        return self.name == other.name
    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))
# Best-first search
def best_first_search(graph, heuristics, start, end):
    
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    
    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name + ': ' + str(start_node.g))
            # Return reversed path
            return path[::-1]
        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = Node(key, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Calculate cost to goal
            neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
            neighbor.h = heuristics.get(neighbor.name)
            neighbor.f = neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True
def depth_first_search(graph, start, end):
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    
    # Loop until the open list is empty
    while len(open) > 0:
        # Get the last node (LIFO)
        current_node = open.pop(-1)
        # Add the current node to the closed list
        closed.append(current_node)
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name + ': ' + str(start_node.g))
            # Return reversed path
            return path[::-1]
        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = Node(key, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Check if neighbor is in open list and if it has a lower f value
            if(neighbor in open):
                continue
            # Calculate cost so far
            neighbor.g = str(current_node.g) + str(graph.get(current_node.name, neighbor.name))
            # Everything is green, add neighbor to open list
            open.append(neighbor)
    # Return None, no path is found
    return None
def breadth_first_search(graph, start, end):
     # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    
    # Loop until the open list is empty
    while len(open) > 0:
        # Get the first node (FIFO)
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name + ': ' + str(start_node.g))
            # Return reversed path
            return path[::-1]
        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = Node(key, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Check if neighbor is in open list and if it has a lower f value
            if(neighbor in open):
                continue
            # Calculate cost so far
            neighbor.g = str(current_node.g) + str(graph.get(current_node.name, neighbor.name))
            # Everything is green, add neighbor to open list
            open.append(neighbor)
    # Return None, no path is found
    return None

def cal_heuristics(graph,start,end):
    heuristics = {}
    
        # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name + ': ' + str(start_node.g))
            # Return reversed path
            return path[::-1]
        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = Node(key, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Calculate cost so far
            neighbor.g = str(current_node.g) + str(graph.get(current_node.name, neighbor.name))
            neighbor.h = 0
            neighbor.f = neighbor.g
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True

    
# A* search
def astar(graph, heuristics, start, end):
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    #heuristics = cal_heuristics(graph,start)
    # Loop until the openList is empty
    while len(open) > 0:
        # Sort the openList to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closedList
        closed.append(current_node)
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name + ': ' + str(start_node.g))
            # Return reversed path
            return path[::-1]
        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = Node(key, current_node)
            # Check if the neighbor is in the closedList
            if(neighbor in closed):
                continue
            # Calculate full path cost
            neighbor.g = str(current_node.g) + str(graph.get(current_node.name, neighbor.name))
            neighbor.h = heuristics.get(neighbor.name)
            neighbor.f = str(neighbor.g) + str(neighbor.h)
            # Check if neighbor is in openList and if it has a lower f value
            if(add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to openList 
                open.append(neighbor)
    # Return None, no path is found
    return None
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True
# astar_path = astar(graph,heuristics,"'H'","'K'")
# print(astar_path)


graph = Graph()    
lines = read30node()  
adj = []
for i in lines:
  adj = []
  adj = i.split(",")
  graph.connect(adj[0].replace("(",""),adj[1],adj[2])
  graph.make_undirected()
print(graph.graph_dict)
    # Create heuristics (straight-line distance, air-travel distance)
  # heuristics = {}
  # path = best_first_search(graph, heuristics, "'U'","'C'")
  # print(path)
print("--------------------------------------------------------------------------")
print("  WE ARE NOW PRINTING DFS SEARCH OUTPUT FOR THE PATH BETWEEN K AND H")
dfs_path = depth_first_search(graph, "'AG'", "'F'")
print(dfs_path)

print("--------------------------------------------------------------------------")
print("  WE ARE NOW PRINTING BFS SEARCH OUTPUT FOR THE PATH BETWEEN K AND H")
bfs_path = breadth_first_search(graph, "'AG'", "'F'")
print(bfs_path)

#graph.make_undirected()
path = cal_heuristics(graph, "'V'", "'AA'")
#print("shortest path")
#print(path)

heuristics = {}
for i in path:
    val = i.split(":")
    #print(val[0])
    #print(val[1].replace(" ",",").split(",")[-1])
    heuristics[val[0]] = val[1].replace(" ",",").split(",")[-1]
    #print("------")
#print(heuristics)

astar_path = astar(graph,heuristics,"'V'","'AA'")
# print("Astar")
# print(astar_path)

# print(type(path))
def search_stats(path):
    # print("The srtat node is:" ,start)
    goal_path_length = len(path)
    print("Length of the goal path",goal_path_length)
    goal_path_cost = path[-1]
    nodes_explored = []
    for i in path:
        val = i.split(":")
        if(i == path[-1]):
            print("Cost of the path :",val[1].replace(" ",",").split(",")[-1])
        nodes_explored.append(val[0])
    max_depth = len(nodes_explored)  
    print("Max depth:" ,max_depth - 1) 
    print("Nodes explored : ",nodes_explored) 
    #print("Final goal path cost : ",list(goal_path_cost.split(":")[1]))
print("BREADTH")
search_stats(bfs_path)
print("DEPTH")
search_stats(dfs_path)
print("A*")
search_stats(astar_path)

  
# Tell python to run main method
#if __name__ == "__main__": main()