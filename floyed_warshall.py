from geopy.distance import geodesic
from geopy.geocoders import Nominatim
import osmnx as ox
from itertools import permutations
import heapq
from geopy.distance import great_circle
import math
import csv
from datetime import datetime, timedelta
import json
import pandas as pd
from pytz import timezone

# Initial speed setup
geolocator = Nominatim(user_agent="myGeocoder")
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


# Implementation of Floyd-Warshall algorithm
def calculate_distances(graph, start_node, end_node):
    # Initialization
    distances = {node: {other_node: float('infinity') for other_node in graph.nodes} for node in graph.nodes}
    next_node = {node: {other_node: None for other_node in graph.nodes} for node in graph.nodes}

    # Set the distance from a node to itself as 0
    for node in graph.nodes:
        distances[node][node] = 0

    # Update distances and next nodes based on graph edges
    for node in graph.nodes:
        for neighbor, data in graph.adj[node].items():
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

            distances[node][neighbor] = time
            next_node[node][neighbor] = neighbor

    # Update distances and next nodes based on Floyd-Warshall algorithm
    for k in graph.nodes:
        for i in graph.nodes:
            for j in graph.nodes:
                if distances[i][j] > distances[i][k] + distances[k][j]:
                    distances[i][j] = distances[i][k] + distances[k][j]
                    next_node[i][j] = next_node[i][k]
                    
    # Reconstruct the shortest path
    if distances[start_node][end_node] == float('infinity'):
        print("No path found.")
        return float('infinity'), []

    path = []
    current_node = start_node
    while current_node != end_node:
        path.append(current_node)
        current_node = next_node[current_node][end_node]
        if current_node is None:
            print("No path found.")
            return float('infinity'), []
    path.append(end_node)

    return distances[start_node][end_node], path


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
def get_shortest_path(graph, destinations,returnToChangi,travelTime):
    location1 = geolocator.geocode("Changi Airport Terminal 3, Singapore")
    start_node = ox.distance.nearest_nodes(graph, location1.longitude, location1.latitude)
    changi_location = (1.355560, 103.987313)
    changi_node = ox.distance.nearest_nodes(graph, changi_location[1], changi_location[0])
    start_time = datetime.strptime(travelTime, "%H:%M")

    # Single destination case
    if len(destinations) == 1:
        location = geolocator.geocode(destinations[0] + ", Singapore")
        end_node = ox.distance.nearest_nodes(graph, location.longitude, location.latitude)
        distance, path = calculate_distances(graph, start_node, end_node)
        shortest_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in path]
        total_time = calculate_path_time(graph, path)

        actual_distance = calculate_actual_distance(shortest_path_coords)
        stops_coords = [(graph.nodes[end_node]['y'], graph.nodes[end_node]['x'])]
        if(returnToChangi):
            back_distance, back_path = calculate_distances(graph, end_node, changi_node)
            back_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in back_path]
            back_actual_distance = calculate_actual_distance(back_path_coords)
            back_total_time = calculate_path_time(graph, back_path)

            shortest_path_coords.extend(back_path_coords[1:])
            stops_coords.append((graph.nodes[changi_node]['y'], graph.nodes[changi_node]['x']))
            actual_distance += back_actual_distance


            total_time += back_total_time

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
        # print(erp_indexes)
        # print(erp_id)
        # print(erp_reach_time)
        # print(total_erp)
        # Eric End
        print("Total time taken for path: ")
        print(total_time)
        return {
            "path_coords": shortest_path_coords,
            "stops_coords": stops_coords,
            "distance": actual_distance,
            "erp_cost": total_erp,   # Eric
            "time": total_time
        }

    # Multiple destinations case, requiring a TSP solution
    waypoints = []
    for destination in destinations:
        location = geolocator.geocode(destination + ", Singapore")
        waypoint = ox.distance.nearest_nodes(graph, location.longitude, location.latitude)
        waypoints.append(waypoint)

    if returnToChangi:
        waypoints.append(changi_node)

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
    totalTime = calculate_path_time(graph,shortest_path_nodes)


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
    # print(erp_indexes)
    # print(erp_id)
    # print(erp_reach_time)
    # print(total_erp)
    print("Total time taken for path: ")
    print(totalTime)
    return {
        "path_coords": shortest_path_coords,
        "stops_coords": stops_coords,
        "distance": actual_distance,
        "erp_cost": total_erp,  # Eric
        "time": totalTime
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
