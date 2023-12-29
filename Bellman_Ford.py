import osmnx as ox
from math import inf

# Get data from osmnx
def data():
    # Create the graph covering ORP Aš 
    north, south, east, west = 50.220, 50.120, 12.330, 12.150
    graph = ox.graph_from_bbox(north, south, east, west, network_type='drive')
    return graph

# Custom weight function based on road type and speed limits
def custom_weight(u, v, data):
    # Define speed limits for different road types (in km/h)
    speed_limits = {
        'residential': 50,
        'primary': 90,
        'primary_link': 90,
        'secondary': 90,
        'secondary_link': -90,              # Negative weight
        'tertiary': 90,
        'tertiary_link': 90,
        'unclassified': 30}

    road_type = data.get('highway')
    if type(road_type) == list:             # Use the first element if it's a list
        road_type = road_type[0]
    speed = speed_limits.get(road_type)
    distance = data['length']
    return (distance/1000) / speed*60   

# Create a dictionary to store the edges in the desired format
def create_dict():
    edges_dict = {}
    # Iterate through the edges and build the dictionary with custom weights
    for u, v, data in graph.edges(data=True):
        if u not in edges_dict:
            edges_dict[u] = {}
        edges_dict[u][v] = custom_weight(u, v, data)
    return edges_dict
    
# Bellman-Ford algorithm
def bellman_ford(G, source):
    U = len(G)                                              # Number of vertices in the graph      
    distance = {node: inf for node in G}                    # Initialize distances with 'inf' for all nodes    
    predecessors = {node: None for node in G}               # Initialize predecessors with 'None' for all nodes    
    distance[source] = 0                                    # Set the distance of the source node to 0
    for _ in range(U - 1):                                  # Relaxation phase        
        for u, neighbors in G.items():                      # Iterate over each node and its neighbors
            for v, weight in neighbors.items():             # Relax the edge (update distance if a shorter path is found)  
                if distance[u] + weight < distance[v]:
                    distance[v] = distance[u] + weight
                    predecessors[v] = u
    for u, neighbors in G.items():                          # Check for negative cycles 
        for v, weight in neighbors.items():                 # If a shorter path is found, there is a negative cycle 
            if distance[u] + weight < distance[v]:
                raise ValueError("Graph contains a negative cycle")   
    return predecessors

# Create path
def createPath(p, u, v):
    path = []                              
    while v != u and v != -1:
        path.insert(0, v)                   
        v = p[v]
    path.insert(0, v)
    return path

node_start = 412695093
node_end = 4406578975

graph = data()
edges_dict = create_dict()
p = bellman_ford(edges_dict, node_start)
t = createPath(p, node_start, node_end)
print("The shortest path goes through these nodes: ", t)

# Plot the graph and the route
fig, ax = ox.plot_graph(graph, bgcolor='black', edge_color='gray', node_color='royalblue', show=False, close=False)
ox.plot_graph_route(graph, t, node_size=0, bgcolor='k', ax=ax)

