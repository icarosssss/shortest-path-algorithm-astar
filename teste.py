import osmnx as ox
import heapq
import folium
from shapely.geometry import Point, LineString
import tracemalloc

tracemalloc.start()

place_name = "São Mateus, Brazil"
graph = ox.graph_from_place(place_name, network_type="drive")

origin_point = Point(-39.852714323004996, -18.715579416899136)
destination_point = Point(-39.84672401668952, -18.724541518174274)

def nearest_node(point, graph):
    node_ids = list(graph.nodes)
    nodes = [Point(graph.nodes[node]['x'], graph.nodes[node]['y']) for node in node_ids]
    nearest_point = min(nodes, key=lambda x: point.distance(x))
    return node_ids[nodes.index(nearest_point)]

origin_node = nearest_node(origin_point, graph)
destination_node = nearest_node(destination_point, graph)

def a_star(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while frontier:
        current = heapq.heappop(frontier)[1]
        if current == goal:
            break
        
        for next_node in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph[current][next_node].get('length', 1)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + ox.distance.euclidean_dist_vec(graph.nodes[next_node]['y'], graph.nodes[next_node]['x'], graph.nodes[goal]['y'], graph.nodes[goal]['x'])
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
    
            currentM, peakM = tracemalloc.get_traced_memory()
            print(f"Uso atual de memória: {currentM / 10**6}MB; Uso máximo de memória: {peakM / 10**6}MB")
    
    return came_from, cost_so_far

came_from, cost_so_far = a_star(graph, origin_node, destination_node)

route_nodes = []
current = destination_node
while current != origin_node:
    route_nodes.append(current)
    current = came_from[current]
route_nodes.append(origin_node)
route_nodes.reverse()

route_coordinates = []
for i in range(len(route_nodes) - 1):
    u, v = route_nodes[i], route_nodes[i+1]
    route_coordinates.append((graph.nodes[u]['y'], graph.nodes[u]['x']))
route_coordinates.append((graph.nodes[destination_node]['y'], graph.nodes[destination_node]['x']))

route_geometry = LineString(route_coordinates)

m = folium.Map(location=[-3.771467, -38.478307], zoom_start=14)

for u, v, data in graph.edges(keys=False, data=True):
    if 'geometry' in data:
        xs, ys = data['geometry'].xy
        coords = zip(ys, xs)
        folium.PolyLine(coords, color="gray", weight=1, opacity=0.5).add_to(m)

folium.PolyLine(route_coordinates, color="red", weight=5, opacity=0.7).add_to(m)

m.save("map2121.html")

tracemalloc.stop()