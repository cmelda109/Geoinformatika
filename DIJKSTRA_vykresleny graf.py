import osmnx as ox
import matplotlib.pyplot as plt
from math import inf
from queue import PriorityQueue

# Define the bounding box coordinates for ORP Aš, Czech Republic
north, south, east, west = 50.220, 50.120, 12.330, 12.150

# Create the road network graph within the ORP boundary
graph = ox.graph_from_bbox(north, south, east, west, network_type='drive')

# Extract restaurant points within the bounding box
mypois = ox.features.features_from_bbox(north, south, east, west, {"amenity": 'restaurant'} ) 

# Get coordinates of nodes and restaurants
nodes, edges = ox.graph_to_gdfs(graph)

# Get the coordinates of nodes and restaurants
nodes_coordinates = nodes[['y', 'x']]
restaurants_coordinates = mypois['geometry'].centroid.apply(lambda geom: (geom.y, geom.x))

# Find the nearest nodes to restaurants
nearest_nodes = [ox.distance.nearest_nodes(graph, lon, lat) for lat, lon in restaurants_coordinates]

# Convert nearest nodes to restaurants
nearest_nodes_new = nodes.loc[nearest_nodes]

# Create a dictionary to store the edges in the desired format
edges_dict = {}

# Iterate through the edges and build the dictionary
for u, v, data in graph.edges(data=True):
    if u not in edges_dict:
        edges_dict[u] = {}
    edges_dict[u][v] = data['length']

# Print the edges in the desired format
print("Edges in the desired format:")
for node in edges_dict:
    print(f"{node}: {edges_dict[node]}")


# Dijkstra
def dijkstra(G, start, end):
    d = {node: inf for node in G}  # Set infinite distance for all nodes
    p = {node: -1 for node in G}   # No predecessors
    Q = PriorityQueue()  # Priority queue
    Q.put((0, start))  # Add start vertex
    d[start] = 0  # Start d[s] = 0
    while not Q.empty():
        du, u = Q.get()  # Pop first element
        for v, wuv in G[u].items():  # Relaxation, all (u,v)
            if d[v] > d[u] + wuv:  # We found a better way
                d[v] = d[u] + wuv  # Update distance
                p[v] = u  # Update predecessor
                Q.put((d[v], v))  # Add to Q
    return p

def createPath(p, u, v):
    path = []
    while v != u and v != -1:
        path.insert(0, v)  # Insert at the beginning of the list for correct order
        v = p[v]
    path.insert(0, v)
    return path


p = dijkstra(edges_dict, 10950835180, 2552640224)
t = createPath(p, 10950835180, 2552640224)
print(t)

# Plot the road network and restaurant points
fig, ax = ox.plot_graph(graph, bgcolor='black', edge_color='gray', node_color='royalblue', show=False, close=False)
mypois.plot(ax=ax, color='red', alpha=0.7, markersize=15, marker='x')
nearest_nodes_new.plot(ax=ax, color='lime', alpha=0.7, markersize=15, marker='x')

# Plot the graph with the specified route
ox.plot_graph_route(graph, t, node_size=0, bgcolor='k', ax=ax)

plt.tight_layout()



import math

def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

point_A = graph.nodes[11055455009]['x'], graph.nodes[11055455009]['y']
point_B = graph.nodes[11055455022]['x'], graph.nodes[11055455022]['y']

distance_AB = euclidean_distance(point_A, point_B)
print(f"The Euclidean distance between A and B is: {distance_AB}")




from math import radians, sin, cos, sqrt, atan2

def haversine_distance(coord1, coord2):
    # Convert decimal degrees to radians
    lat1, lon1 = map(radians, coord1)
    lat2, lon2 = map(radians, coord2)

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    
    # Earth radius in kilometers
    radius_earth_km = 6371
    distance_km = radius_earth_km * c
    
    # Convert distance to meters
    distance_meters = distance_km * 1000

    return distance_meters

# Example usage:
point_A = graph.nodes[11055455009]['x'], graph.nodes[11055455009]['y']
point_B = graph.nodes[11055455022]['x'], graph.nodes[11055455022]['y']

distance_AB = haversine_distance(point_A, point_B)
print(f"The Haversine distance between A and B is: {distance_AB} meters")
