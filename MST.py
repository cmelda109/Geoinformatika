import osmnx as ox
from math import inf, radians, sin, cos, sqrt, atan2

# Get data from osmnx
def data():
    # Create the graph covering ORP Aš 
    north, south, east, west = 50.220, 50.120, 12.330, 12.150
    graph = ox.graph_from_bbox(north, south, east, west, network_type='drive')

    # Get coordinates of nodes, edges 
    nodes, edges = ox.graph_to_gdfs(graph)
    return graph, nodes

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

# Custom weight function based on road type, speed limits and sinusoity
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
    
    # Check if Euclidean distance is zero or very small
    if euclidean_dist <= 0.001:  
        return inf  
    
    # Calculate sinuosity penalty based on the ratio of actual distance to Euclidean distance
    sinuosity_penalty = distance / euclidean_dist
    
    # Adjust the distance based on sinuosity penalty
    adjusted_distance = distance * sinuosity_penalty    
    return (adjusted_distance/1000 / speed) *60       
        
# Create lists to store the edges in the desired format
def create_lists():      
    list_of_nodes = nodes.index.tolist()    
    list_of_edges = []
    for u, v, data in graph.edges(data=True):
        if u in list_of_nodes or v in list_of_nodes:
            weight = custom_weight(u, v, data)
            list_of_edges.append([u, v, weight])
    return list_of_nodes,list_of_edges

# Union (weighted union)
def union(u, v, p, rank):
    root_u = find(u, p)                     # Find root for u + compress
    root_v = find(v, p)                     # Find root for v + compress
    if root_u != root_v:                    # u, v in different subtrees
        if rank[root_u] < rank[root_v]:     # u subtree is longer
            p[root_u] = root_v              # Connect v to u
        elif rank[root_u] > rank[root_v]:   # v subtree is longer
            p[root_v] = root_u              # Connect u to v
        else:                               # u, v have equal lengths
            p[root_u] = root_v              # Connect u to v
            rank[root_v] += 1               # Increment rank

# Find (path compression)        
def find(u, p):
    if p[u] != u:                           # Move to grandparent
        p[u] = find(p[u], p)                # Predecessor is grandparent
    return p[u]

# Make set
def make_set(u, p, rank):
    p[u] = u
    rank[u] = 0

# MST
def mst(V, E):
    T = []                                  # Empty tree
    wt = 0                                  # Sum of weights of T
    p = {}                                  # Dictionary to represent the disjoint-set forest
    rank = {}                               # Dictionary to keep track of the rank of each set                        
    for v in V:                             # Make set
        make_set(v, p, rank)                # Initialize p and rank    
    ES = sorted(E, key=lambda it: it[2])    # Sort edges by w
    for e in ES:                            # Process all edges
        u, v, w = e                         # Take an edge
        if find(u, p) != find(v, p):        # Roots u, v in different trees?
            union(u, v, p, rank)            # Create union
            T.append([u, v, w])             # Add edge to tree
            wt = wt + w                     # Compute weight of T
    return wt, T

graph, nodes = data()
list_of_nodes, list_of_edges = create_lists()
wt, T = mst(list_of_nodes, list_of_edges)
print(wt)

# Visualize the graph 
spanning_tree = [[u, v] for u, v, _ in T]
fig, ax = ox.plot_graph_routes(graph, spanning_tree, route_colors='red', route_linewidth=1)


