from geopy.geocoders import Nominatim
import osmnx as ox
from itertools import permutations
import heapq
from geopy.distance import great_circle
import math
import csv
from datetime import datetime, timedelta
import json
from pytz import timezone
import time as tm

# Initialize geolocator with user agent "myGeocoder"
geolocator = Nominatim(user_agent="myGeocoder")
# Define default speed as 50 km/h converted to m/s
default_speed = 50 / 3.6  # 50 km/h in m/s

# Data for Weekdays / Weekend and Time
# Define timezone as Singapore
singapore = timezone('Asia/Singapore')

# Set day type and vehicle type
day_type = "Weekdays"
vehicle_type = "Heavy Goods Vehicles/Small Buses"
# Open the CSV file containing gantry locations
with open('data/gantry-locations-updated.csv', 'r') as file:
    # Create a CSV reader object
    csv_reader = csv.reader(file)

    # Skip the header row
    next(csv_reader)

    # Store the values in a variable
    csv_values = [row for row in csv_reader]

with open('data/test_erp.json', 'r') as file:
    data = json.load(file)


# Define the heuristic function for A* algorithm
def heuristic(node1, node2):
    dx = node1['x'] - node2['x']
    dy = node1['y'] - node2['y']
    return math.sqrt(dx * dx + dy * dy)

# Function for A* algorithm to calculate distances on graph
def calculate_distances(graph, start_node, end_node):
    # Initialization
    distances = {node: float('infinity') for node in graph.nodes}
    distances[start_node] = 0
    parent = {node: None for node in graph.nodes}
    pq = [(0, start_node)]
    # Main A* algorithm loop
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        for neighbor, data in graph.adj[current_node].items():
            distance = data[0]['length']
            max_speed = data[0].get('maxspeed', default_speed)
            if isinstance(max_speed, float):
                max_speed = int(max_speed)
            elif isinstance(max_speed, str) and max_speed.isdigit():
                max_speed = int(max_speed)
            else:
                max_speed = default_speed
            if max_speed > 70:
                max_speed = 70
            time = distance / max_speed  # time = distance/speed
            old_time = distances[neighbor]
            new_time = distances[current_node] + time
            if new_time < old_time:
                distances[neighbor] = new_time
                parent[neighbor] = current_node
                h_value = heuristic(graph.nodes[neighbor], graph.nodes[end_node])
                heapq.heappush(pq, (distances[neighbor] + h_value, neighbor))

    # Reconstruct the shortest path
    path = []
    current = end_node
    while current is not None:
        path.append(current)
        current = parent[current]
    path = path[::-1]

    return distances[end_node], path


# Function to calculate the total time taken for the path
def calculate_path_time(graph, path):
    total_time = 0
    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]
        data = graph.adj[current_node][next_node]
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


# Function implementing a version of the TSP. It calculates the shortest path
# between a set of destinations and an optional return to Changi
def get_shortest_path(graph, destinations, returnToChangi, travelTime):
    start_time_algo = tm.time()
    location1 = geolocator.geocode("Changi Airport Terminal 3, Singapore")
    start_node = ox.distance.nearest_nodes(graph, location1.longitude, location1.latitude)
    changi_location = (1.355560, 103.987313)
    changi_node = ox.distance.nearest_nodes(graph, changi_location[1], changi_location[0])
    start_time = datetime.strptime(travelTime, "%H:%M")

    # Single destination case
    if len(destinations) == 1:
        stops_name = []
        location = geolocator.geocode(destinations[0] + ", Singapore")
        end_node = ox.distance.nearest_nodes(graph, location.longitude, location.latitude)
        distance, path = calculate_distances(graph, start_node, end_node)
        shortest_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in path]
        total_time = calculate_path_time(graph, path)

        actual_distance = calculate_actual_distance(shortest_path_coords)
        stops_coords = [(graph.nodes[end_node]['y'], graph.nodes[end_node]['x'])]
        stops_name.append(destinations[0])
        if (returnToChangi):
            back_distance, back_path = calculate_distances(graph, end_node, changi_node)
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

        for i in range(len(csv_values)):
            for j in range(len(path_array)):
                if int(csv_values[i][7]) == path_array[j]:
                    print(csv_values[i][0], csv_values[i][1], csv_values[i][2])
                    erp_id.append(csv_values[i][1])
                    erp_time = start_time + timedelta(minutes=int(times[j]))
                    erp_reach_time.append(erp_time)

        total_erp = erp_cost(erp_id, erp_reach_time)

        end_time_algo = tm.time()
        execution_time = end_time_algo - start_time_algo
        print("Code execution time: {:.2f} seconds".format(execution_time))
        print("Total time taken for path: {:.2f} Minutes".format(total_time))
        print("Total Distance for path: {:.2f} Meters".format(actual_distance))

        return {
            "path_coords": shortest_path_coords,
            "stops_coords": stops_coords,
            "stops_name": stops_name,
            "distance": actual_distance,
            "erp_cost": total_erp,
            "time": total_time,
            "execution_time": execution_time
        }

    # Multiple destinations case, requiring a TSP solution
    waypoints = []
    dest_node = dict()
    for destination in destinations:
        location = geolocator.geocode(destination + ", Singapore")
        waypoint = ox.distance.nearest_nodes(graph, location.longitude, location.latitude)
        waypoints.append(waypoint)
        dest_node[destination] = waypoint

    if returnToChangi:
        waypoints.append(changi_node)
        dest_node["Changi Airport Terminal 3"] = changi_node

    shortest_distance = float('inf')
    shortest_path_nodes = None
    # Generate all permutations
    all_permutations = list(permutations(waypoints))

    # Keep only those permutations where the last element is new_node
    filtered_permutations = [perm for perm in all_permutations if
                             perm[-1] == changi_node] if returnToChangi else all_permutations

    for waypoint_order in filtered_permutations:
        total_distance = 0
        path = [start_node]
        first_node = waypoint_order[0]
        distance, sub_path = calculate_distances(graph, start_node, first_node)
        total_distance += distance
        path.extend(sub_path[1:])

        for i in range(1, len(waypoint_order)):
            distance, sub_path = calculate_distances(graph, waypoint_order[i - 1], waypoint_order[i])
            total_distance += distance
            path.extend(sub_path[1:])

        if total_distance < shortest_distance:
            shortest_distance = total_distance
            shortest_path_nodes = path

    if shortest_path_nodes is None:
        print("No path found.")
        return {
            "path_coords": [],
            "stops_coords": [],
            "distance": float('inf')
        }
    shortest_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path_nodes]
    actual_distance = calculate_actual_distance(shortest_path_coords)
    stops_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path_nodes if
                    node in waypoints]
    stops_name = []
    for node in shortest_path_nodes:
        for key, value in dest_node.items():
            if value == node:
                stops_name.append(key)

    totalTime = calculate_path_time(graph, shortest_path_nodes)

    path_array = [node for node in shortest_path_nodes]
    times = time(graph, path_array)
    erp_reach_time = []
    erp_id = []

    for i in range(len(csv_values)):
        for j in range(len(path_array)):
            if int(csv_values[i][7]) == path_array[j]:
                print(csv_values[i][0], csv_values[i][1], csv_values[i][2])
                erp_id.append(csv_values[i][1])
                erp_time = start_time + timedelta(minutes=int(times[j]))
                erp_reach_time.append(erp_time)

    total_erp = erp_cost(erp_id, erp_reach_time)

    end_time_algo = tm.time()
    execution_time = end_time_algo - start_time_algo

    print("Code execution time: {:.2f} seconds".format(execution_time))
    print("Total time taken for path: {:.2f} Minutes".format(totalTime))
    print("Total Distance for path: {:.2f} Meters".format(actual_distance))

    # Return both the path and the stops
    return {
        "path_coords": shortest_path_coords,
        "stops_coords": stops_coords,
        "stops_name": stops_name,
        "distance": actual_distance,
        "erp_cost": total_erp,
        "time": totalTime,
        "execution_time": execution_time
    }
# Functions to calculate actual distance, time and ERP cost
def calculate_actual_distance(path):
    total_distance = 0
    for i in range(len(path) - 1):
        # The geopy.distance.great_circle function expects coordinates in
        # the order (latitude, longitude).
        distance = great_circle(path[i], path[i + 1]).meters  # distance in meters
        total_distance += distance
    return total_distance


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
