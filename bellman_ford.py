from geopy.geocoders import Nominatim
import osmnx as ox
from itertools import permutations
from geopy.distance import great_circle
import csv
from datetime import datetime, timedelta
import json
from pytz import timezone
import heapq
import time as tm

# Create a Nominatim geocoder instance
geolocator = Nominatim(user_agent="myGeocoder")
# Define the default speed in m/s (50 km/h)
default_speed = 50 / 3.6  # 50 km/h in m/s

# Data for Weekdays / Weekend and Time
# Define timezone
singapore = timezone('Asia/Singapore')

# Get the current date and time in Singapore
day_type = "Weekdays"
vehicle_type = "Heavy Goods Vehicles/Small Buses"
# Open the CSV file
with open('data/gantry-locations-updated.csv', 'r') as file:
    # Create a CSV reader object
    csv_reader = csv.reader(file)

    # Skip the header row
    next(csv_reader)

    # Store the values in a variable
    csv_values = [row for row in csv_reader]

with open('data/test_erp.json', 'r') as file:
    data = json.load(file)

# Bellman Ford's algorithm to find the shortest path between two points
def bellman_ford(graph, start_node, end_node):
    # Initialize distances and predecessors
    distances = {node: float('infinity') for node in graph.nodes}
    distances[start_node] = 0
    parent = {node: None for node in graph.nodes}

    # Create a priority queue
    priority_queue = [(0, start_node)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # Ignore outdated entries
        if current_distance > distances[current_node]:
            continue

        for neighbor, data in graph.adj[current_node].items():
            weight = data[0]['length'] / float(data[0].get('maxspeed', default_speed))
            distance = distances[current_node] + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                parent[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    # Check for negative-weight cycles
    for node in graph.nodes:
        for neighbor, data in graph.adj[node].items():
            weight = data[0]['length'] / float(data[0].get('maxspeed', default_speed))
            distance = distances[node] + weight

            if distance < distances[neighbor]:
                raise ValueError('Graph contains a negative-weight cycle')

    return distances[end_node], path_from_parent(parent, end_node)

# Function to calculate distances between points
def calculate_distances(graph, start_node, end_node):
    distances = {node: float('infinity') for node in graph.nodes}
    distances[start_node] = 0

    pq = [(0, start_node)]  # Priority queue to store nodes based on their distance
    parent = {node: None for node in graph.nodes}
    visited = set()

    while pq:
        curr_dist, curr_node = heapq.heappop(pq)
        if curr_node in visited:
            continue
        visited.add(curr_node)

        if curr_node == end_node:  # if we reached to the destination, we break the loop
            break

        for neighbor, data in graph.adj[curr_node].items():
            distance = data[0]['length']
            max_speed = data[0].get('maxspeed', default_speed)
            if isinstance(max_speed, float):
                max_speed = int(max_speed)
            elif isinstance(max_speed, str) and max_speed.isdigit():
                max_speed = int(max_speed)
            else:
                max_speed = default_speed
            time = distance / max_speed  # time = distance/speed

            new_dist = distances[curr_node] + time
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                parent[neighbor] = curr_node
                heapq.heappush(pq, (new_dist, neighbor))

    # get path
    path = []
    current = end_node
    while current is not None:
        path.append(current)
        current = parent[current]
    path = path[::-1]  # reverse path

    return distances[end_node], path

# Function to calculate total time taken for a given path
def calculate_path_time(graph, path):
    total_time = 0
    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]
        try:
            data = graph.adj[current_node][next_node]
        except KeyError:
            print(f"Node {current_node} and {next_node} are not connected.")
        distance = data[0]['length']
        current_coords = (graph.nodes[current_node]['y'], graph.nodes[current_node]['x'])
        next_coords = (graph.nodes[next_node]['y'], graph.nodes[next_node]['x'])
        actual_distance = great_circle(current_coords, next_coords).kilometers
        max_speed = data[0].get('maxspeed', default_speed)
        if isinstance(max_speed, float):  # speed is in km
            max_speed = int(max_speed)
        elif isinstance(max_speed, str) and max_speed.isdigit():
            max_speed = int(max_speed)
        else:
            max_speed = default_speed
        if max_speed > 70:
            max_speed = 70
        time = actual_distance / max_speed
        total_time += time
    total_time = total_time * 60  # convert to minutes
    return total_time

# Function to find shortest path for multiple destinations and calculate ERP cost
def get_shortest_path(graph, destinations, returnToChangi, travelTime):
    start_time_algo = tm.time()
    location1 = geolocator.geocode("Changi Airport Terminal 3, Singapore")
    start_node = ox.distance.nearest_nodes(graph, location1.longitude, location1.latitude)
    changi_location = (1.355560, 103.987313)
    changi_node = ox.distance.nearest_nodes(graph, changi_location[1], changi_location[0])
    start_time = datetime.strptime(travelTime, "%H:%M")

    # Case when there is only one destination
    if len(destinations) == 1:
        stops_name = []
        location = geolocator.geocode(destinations[0] + ", Singapore")
        end_node = ox.distance.nearest_nodes(graph, location.longitude, location.latitude)
        distance, path = bellman_ford(graph, start_node, end_node)
        shortest_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in path]
        actual_distance = calculate_actual_distance(shortest_path_coords)
        stops_coords = [(graph.nodes[end_node]['y'], graph.nodes[end_node]['x'])]
        total_time = calculate_path_time(graph, path)
        stops_name.append(destinations[0])

        if returnToChangi:
            back_distance, back_path = bellman_ford(graph, end_node, changi_node)
            back_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in back_path]
            back_actual_distance = calculate_actual_distance(back_path_coords)
            back_total_time = calculate_path_time(graph, back_path)

            shortest_path_coords.extend(back_path_coords[1:])
            stops_coords.append((graph.nodes[changi_node]['y'], graph.nodes[changi_node]['x']))
            actual_distance += back_actual_distance

            total_time += back_total_time
            stops_name.append("Changi Airport Terminal 3")

        path_array = [node for node in path]
        times = time(graph, path_array)
        erp_reach_time = []
        erp_id = []
        # erp_indexes = []
        for i in range(len(csv_values)):
            for j in range(len(path_array)):
                if int(csv_values[i][7]) == path_array[j]:
                    print(csv_values[i][0], csv_values[i][1], csv_values[i][2])
                    # erp_indexes.append(j)
                    erp_id.append(csv_values[i][1])
                    erp_time = start_time + timedelta(minutes=int(times[j]))
                    erp_reach_time.append(erp_time)

        total_erp = erp_cost(erp_id, erp_reach_time)

        end_time_algo = tm.time()
        execution_time = end_time_algo - start_time_algo

        print("Code execution time: {:.2f} seconds".format(execution_time))
        print("Total time taken for path: {:.2f} Minutes".format(total_time))
        print("Total Distance for path: {:.2f} Meters".format(actual_distance))

        # Return both the path and the stops
        return {
            "path_coords": shortest_path_coords,
            "stops_coords": stops_coords,
            "stops_name": stops_name,
            "distance": actual_distance,
            "erp_cost": total_erp,
            "time": total_time,
            "execution_time": execution_time
        }

    # Case when there are multiple destinations
    waypoints = []
    dest_node = dict()
    for destination in destinations:
        location = geolocator.geocode(destination + ", Singapore")
        waypoint = ox.distance.nearest_nodes(graph, location.longitude, location.latitude)
        waypoints.append(waypoint)
        dest_node[destination] = waypoint

    shortest_distance = float('inf')
    shortest_path_nodes = None
    shortest_waypoint_order = None
    for waypoint_order in permutations(waypoints):
        total_distance = 0
        path = []

        for i in range(len(waypoint_order)):
            if i == 0:
                start = start_node
                end = waypoint_order[0]
            else:
                start = waypoint_order[i - 1]
                end = waypoint_order[i]

            distance, partial_path = bellman_ford(graph, start, end)
            total_distance += distance
            path.extend(partial_path)

        if total_distance < shortest_distance:
            shortest_distance = total_distance
            shortest_path_nodes = path
            shortest_waypoint_order = waypoint_order

    if shortest_path_nodes is None:
        return [], float('inf')

    shortest_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path_nodes]
    stops_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path_nodes if
                    node in waypoints]

    stops_name = []
    for node in shortest_path_nodes:
        for key, value in dest_node.items():
            if value == node:
                stops_name.append(key)

    # Calculate the actual distance of the shortest path
    actual_distance = calculate_actual_distance(shortest_path_coords)
    if returnToChangi:
        back_distance, back_path = bellman_ford(graph, shortest_waypoint_order[-1], changi_node)
        back_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in back_path]
        back_actual_distance = calculate_actual_distance(back_path_coords)

        shortest_path_coords.extend(back_path_coords[1:])
        stops_coords.append((graph.nodes[changi_node]['y'], graph.nodes[changi_node]['x']))

        actual_distance += back_actual_distance
        shortest_path_nodes.extend(back_path)
        stops_name.append("Changi Airport Terminal 3")

    totalTime = calculate_path_time(graph, shortest_path_nodes)

    path_array = [node for node in shortest_path_nodes]
    times = time(graph, path_array)
    erp_reach_time = []
    erp_id = []
    # erp_indexes = []
    for i in range(len(csv_values)):
        for j in range(len(path_array)):
            if int(csv_values[i][7]) == path_array[j]:
                print(csv_values[i][0], csv_values[i][1], csv_values[i][2])
                # erp_indexes.append(j)
                erp_id.append(csv_values[i][1])
                erp_time = start_time + timedelta(minutes=int(times[j]))
                erp_reach_time.append(erp_time)

    total_erp = erp_cost(erp_id, erp_reach_time)
    end_time_algo = tm.time()
    execution_time = end_time_algo - start_time_algo
    print("Code execution time: {:.2f} seconds".format(execution_time))
    print("Total time taken for path: {:.2f} Minutes".format(totalTime))
    print("Total Distance for path: {:.2f} Meters".format(actual_distance))

    return {
        "path_coords": shortest_path_coords,
        "stops_coords": stops_coords,
        "stops_name": stops_name,
        "distance": actual_distance,
        "erp_cost": total_erp,
        "time": totalTime,
        "execution_time": execution_time
    }

# Function to generate the path from the parent
def path_from_parent(parent, node):
    path = []
    current = node
    while current is not None:
        path.append(current)
        current = parent[current]
    return path[::-1]

# Function to calculate actual distance covered in a path
def calculate_actual_distance(path):
    total_distance = 0
    for i in range(len(path) - 1):
        # The geopy.distance.great_circle function expects coordinates in
        # the order (latitude, longitude).
        distance = great_circle(path[i], path[i + 1]).meters  # distance in meters
        total_distance += distance
    return total_distance

# Function to calculate time taken for each leg of the route
def time(graph, route):
    total_time = 0.0
    total_distance = 0.0
    time_array = []
    for i in range(len(route) - 1):
        source = route[i]
        target = route[i + 1]
        edge_data = graph.get_edge_data(source, target)
        # print(type(edge_data[0]['maxspeed']))
        # print(edge_data[0]['maxspeed'])
        if edge_data is not None:
            length = float(edge_data[0]['length']) / 1000
            if 'maxspeed' in edge_data[0]:
                if isinstance(edge_data[0]['maxspeed'], str):
                    maxspd = float(edge_data[0]['maxspeed'])
                elif isinstance(edge_data[0]['maxspeed'], list):
                    maxspd = 0.0
                    for i in edge_data[0]['maxspeed']:
                        maxspd += float(i)
                    maxspd /= len(edge_data[0]['maxspeed'])
                else:
                    maxspd = None

            # max_speed = float(edge_data[0]['maxspeed']) if 'maxspeed' in edge_data[0] else None
            # print(max_speed)
            # Calculate time based on distance and speed
            if maxspd is not None:
                time_taken = (length / (maxspd - 20.0)) * 60
            else:
                # If max speed is not available, assume a default speed
                default_speed = 30.0  # You can adjust the default speed as needed
                time_taken = (length / default_speed) * 60

            total_distance += length
            total_time += time_taken
            time_array.append(int(total_time))

            # print(" num: ",count," total distance: ",total_distance," total time: ", total_time)

    return time_array


def erp_cost(zoneId, erpTime):
    total_erp = 0.0
    for i in range(len(zoneId)):
        # print(erpTime[i])
        for entry in data:
            if zoneId[i] == entry["ZoneID"] and day_type == entry["DayType"] and vehicle_type == entry["VehicleType"]:
                # Convert the StartTime and EndTime strings to datetime objects
                stime = datetime.strptime(entry["StartTime"], "%H:%M")
                etime = datetime.strptime(entry["EndTime"], "%H:%M")
                # print(stime <= erpTime[i], erpTime[i] <= etime)
                # Check if the compare_time falls between the start_time and end_time
                if stime.time() <= erpTime[i].time() <= etime.time():
                    total_erp += float(entry["ChargeAmount"])
                    # print(entry["ZoneID"], entry["DayType"], entry["StartTime"], entry["EndTime"],
                    # entry["VehicleType"], entry["ChargeAmount"])

    return total_erp

