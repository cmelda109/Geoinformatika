import osmnx as ox
from math import inf, radians, sin, cos, sqrt, atan2
from queue import PriorityQueue


# Create the graph (smaller)
def data():
    north, south, east, west = 50.20, 50.17, 12.20, 12.17
    graph = ox.graph_from_bbox(north, south, east, west, network_type='drive')
    return graph

# Function to calculate Euclidean distance between two nodes
def euclidean_distance(node1, node2):
    lat1, lon1 = node1['y'], node1['x']  
    lat2, lon2 = node2['y'], node2['x']
    return sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

# Calculate Euclidean distance between two nodes in metres
def euclidean_distance(node1, node2):
    lat1, lon1 = radians(node1['y']), radians(node1['x'])  
    lat2, lon2 = radians(node2['y']), radians(node2['x'])
    R = 6371000                            # Radius of the Earth in meters
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))    
    distance = R * c
    return distance

# Custom weight function based on road type and speed limits
def custom_weight(u, v, data):
    
    # Define speed limits for different road types (in km/h)
    speed_limits = {
        'residential': 50,
        'primary': 90,
        'primary_link': 90,
        'secondary': 90,
        'secondary_link': 90,
        'tertiary': 90,
        'tertiary_link': 90,
        'unclassified': 30}

    road_type = data.get('highway')
    if type(road_type) == list:             # Use the first element if it's a list
        road_type = road_type[0]
    speed = speed_limits.get(road_type)
    distance = data['length']

    # Calculate Euclidean distance
    start_node = graph.nodes[u]
    end_node = graph.nodes[v]
    euclidean_dist = euclidean_distance(start_node, end_node)
    
    # Calculate sinuosity penalty based on the ratio of actual distance to Euclidean distance
    sinuosity_penalty = distance / euclidean_dist
    
    # Adjust the distance based on sinuosity penalty
    adjusted_distance = distance * sinuosity_penalty    
    return (adjusted_distance/1000 / speed) *60  

# Create a dictionary to store the edges in the desired format
def create_dict(graph):
    edges_dict = {}
    # Iterate through the edges and build the dictionary with custom weights
    for u, v, data in graph.edges(data=True):
        if u not in edges_dict:
            edges_dict[u] = {}
        edges_dict[u][v] = custom_weight(u, v, data)
    return edges_dict 

# Dijkstra
def dijkstra(G, start, end):
    d = {node: inf for node in G}           # Set infinite distance for all nodes
    p = {node: None for node in G}          # No predecessors
    Q = PriorityQueue()                     # Priority queue
    Q.put((0, start))                       # Add start vertex
    d[start] = 0                            # Start d[s] = 0
    while not Q.empty():                    # Repeat until queue is empty 
        du, u = Q.get()                     # Get node and distance
        for v, wuv in G[u].items():         # Go through all neighbors
            if d[v] > d[u] + wuv:           # Relaxation
                d[v] = d[u] + wuv           # Update distance
                p[v] = u                    # Update predecessor
                Q.put((d[v], v))            # Add node to queue
    return p

# Find and print shortest paths between all pairs of nodes
def all_pairs():    
    for start_node in edges_dict.keys():
        for end_node in edges_dict.keys():
            if start_node != end_node:              # Exclude self-pair nodes
                parents = dijkstra(edges_dict, start_node, end_node)
                path = []
                current_node = end_node
                while current_node is not None:
                    path.insert(0, current_node)
                    current_node = parents[current_node]
                if path[0] != -1:
                    print(f"Start Node: {start_node}, End Node: {end_node}, Shortest Path: {path}")
    return path

graph = data()
edges_dict = create_dict(graph)
all_pairs()




