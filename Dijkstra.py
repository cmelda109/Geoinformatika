import osmnx as ox
from math import inf, radians, sin, cos, sqrt, atan2
from queue import PriorityQueue

# Get data from osmnx
def data():
    # Create the graph covering ORP Aš 
    north, south, east, west = 50.220, 50.120, 12.330, 12.150
    graph = ox.graph_from_bbox(north, south, east, west, network_type='drive')

    # Extract restaurant points 
    mypois = ox.features.features_from_bbox(north, south, east, west, {"amenity": 'restaurant'} ) 
    mypois_points = mypois[mypois.geometry.type == 'Point']              # Filter out only point features

    # Get coordinates of nodes, edges and restaurants
    nodes, edges = ox.graph_to_gdfs(graph)
    restaurants_coordinates = mypois_points['geometry'].apply(lambda geom: (geom.y, geom.x))

    # Find the nearest nodes to restaurants and convert them to restaurants
    nearest_nodes = [ox.distance.nearest_nodes(graph, lon, lat) for lat, lon in restaurants_coordinates]
    nearest_nodes_new = nodes.loc[nearest_nodes]

    return graph, mypois_points, nearest_nodes_new

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
    
    # Calculate sinuosity penalty based on the ratio of actual distance to Euclidean distance
    sinuosity_penalty = distance / euclidean_dist
    
    # Adjust the distance based on sinuosity penalty
    adjusted_distance = distance * sinuosity_penalty    
    return (adjusted_distance/1000 / speed) *60

# Create dictionaries to store the edges in the desired format
def create_dicts(graph):
    edges_dict_eucl = {}                    # Dictionary for euclidean distance 
    edges_dict_speed = {}                   # Dictionary for custom weights 
        
    for u, v, data in graph.edges(data=True):
        if u not in edges_dict_eucl:        
            edges_dict_eucl[u] = {}
        edges_dict_eucl[u][v] = data['length']

        if u not in edges_dict_speed:       
            edges_dict_speed[u] = {}
        edges_dict_speed[u][v] = custom_weight(u, v, data)   
    
    return edges_dict_eucl, edges_dict_speed

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

# Create path
def createPath(p, u, v):
    path = []                               # Create empty path
    while v != u and v != -1:
        path.insert(0, v)                   
        v = p[v]
    path.insert(0, v)
    return path

# Call dijkstra, plot the graph and calculate travel time and distance
def call_plot_calculate(graph, dict,calculate_travel_time=True,plot=True):
    node_start = 412695093
    node_end = 4406578975
    
    # Call functions
    p = dijkstra(dict, node_start, node_end)
    t = createPath(p, node_start, node_end)
    print("The best path goes through these nodes: ", t)

    # Plot the road network, restaurant points and their nearest nodes, and the specified route
    if plot:
        fig, ax = ox.plot_graph(graph, bgcolor='black', edge_color='gray', node_color='royalblue', show=False, close=False)
        mypois_points.plot(ax=ax, color='red', alpha=0.7, markersize=30, marker='o')
        nearest_nodes_new.plot(ax=ax, color='lime', alpha=1, markersize=30, marker='x')
        ox.plot_graph_route(graph, t, node_size=0, bgcolor='k', ax=ax)   

    # Calculate the total distance
    total_distance = 0
    for i in range(len(t) - 1):        
        edge_length = graph[t[i]][t[i + 1]][0]['length']        
        total_distance += edge_length
    print(f"Total Distance: {total_distance/1000:.2f} km")         

    # Calculate the total travel time in minutes
    if calculate_travel_time:
        total_travel_time_minutes = 0
        for i in range(len(t) - 1):
            u = t[i]
            v = t[i + 1]
            weight = dict[u][v]
            total_travel_time_minutes += weight
        print(f"Total Travel Time: {total_travel_time_minutes:.2f} minutes")     
    return t   

graph, mypois_points, nearest_nodes_new = data()
edges_dict_eucl, edges_dict_speed = create_dicts(graph)
t = call_plot_calculate(graph,edges_dict_eucl,plot=False)

# Custom weight function based on road type and speed limits
def custom_weight_2(u, v, data):
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
    return (distance/1000) / speed*60

# Calculate travel time of the shortest path
def calculate_time_shortest_path():    
    dict_travel_time = {}                               # Create a new dictionary only with nodes (IDs and weights) of the shortest path
    for u, v, data in graph.edges(data=True):
        if u in t and v in t:            
            dict_travel_time[u] = {}
            dict_travel_time[u][v] = custom_weight_2(u, v, data)    
    weights = [v if not isinstance(v, dict) else list(v.values())[0] for v in dict_travel_time.values()] # Dictionary containing only the weights (travel time between nodes)
    total_sum = sum(weights)    
    print("Total travel time:", round(total_sum,2), "minutes")

call_plot_calculate(graph, edges_dict_eucl, calculate_travel_time=False) 
calculate_time_shortest_path()
call_plot_calculate(graph, edges_dict_speed)




