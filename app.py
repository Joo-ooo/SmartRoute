# app.py

from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
import osmnx as ox
from dijkstra import get_shortest_path as dijkstra_path
from a_star import get_shortest_path as a_star_path
from bellman_ford import get_shortest_path as bellman_ford_path  # Import the function
import pandas as pd
import requests


# Define the base URL of the API endpoints
url = 'http://datamall2.mytransport.sg/ltaodataservice/TrafficIncidents'

# Define the headers, including your API key
headers = {
    'AccountKey': 'bfbtTCDRSUadxR/Ves5pdQ==',  # replace 'YOUR_ACCOUNT_KEY' with your actual account key
    'accept': 'application/json'
}

# Fetch and save data from the TrafficIncidents endpoint
response = requests.get(url, headers=headers)
data = response.json()
df = pd.DataFrame(data['value'])
df.to_csv('data/traffic_incidents.csv', index=False)
url = 'http://datamall2.mytransport.sg/ltaodataservice/ERPRates'
headers = {
    'AccountKey': 'bfbtTCDRSUadxR/Ves5pdQ==',  # replace 'YOUR_ACCOUNT_KEY' with your actual account key
    'accept': 'application/json'
}

# Fetch and save data from the ERPRates endpoint, handling pagination
data = []
page_no = 0
while True:
    # Make the GET request with page number
    response = requests.get(url + '?$skip=' + str(page_no * 500), headers=headers)

    # Check if the status code indicates success
    if response.status_code != 200:
        break

    # Convert the response to JSON
    json_response = response.json()

    # If no data is returned, then exit loop
    if not json_response['value']:
        break

    # Append the fetched data to list
    data.extend(json_response['value'])

    # Increment the page number
    page_no += 1

# Convert the JSON data to a pandas DataFrame
df = pd.DataFrame(data)

# Save the DataFrame to a CSV file
df.to_json('data/test_erp.json', orient='records')

# Load hotel names from an Excel file
df = pd.read_excel('data/hotel.xlsx')
hotel_names = df['Hotel Name'].tolist()

# Initialize Flask app and allow CORS
app = Flask(__name__)
CORS(app)  # This will allow our server to respond to requests from our webpage

# Load the graph for Singapore
place_name = "Singapore"
graph = ox.graph_from_place(place_name, network_type='drive',retain_all=True, truncate_by_edge=True,simplify=False)
# graph = ox.load_graphml('data/singapore_map_data.graphml')

# Define the root endpoint
@app.route("/")
def base():
    return render_template("index.html", hotel_names=hotel_names)

# Define the shortest_path endpoint
@app.route('/shortest_path', methods=['POST'])
def shortest_path():
    # Extract the request data
    data = request.get_json()
    destination = request.json['destination']
    algorithm = request.json['algorithm']
    destination = list(filter(None, destination))
    returnToChangi = request.json['returnToChangi'] # Add this
    travelTime = request.json['travelTime']

    # Load traffic incidents data and calculate nearest nodes
    print(destination)
    traffic_incidents = pd.read_csv("data/traffic_incidents.csv")
    incident_coords = traffic_incidents[['Latitude', 'Longitude']].values
    incident_nodes = [ox.distance.nearest_nodes(graph, coord[1], coord[0]) for coord in incident_coords]

    # Update the lengths of the edges connected to the incident nodes
    for node in incident_nodes:
        for neighbor, data in graph.adj[node].items():
            for key in data.keys():
                data[key]['length'] *= 500

    # Call the appropriate function based on the selected algorithm
    if algorithm == 'dijkstra':
        response = dijkstra_path(graph, destination,returnToChangi,travelTime)
    elif algorithm == 'a_star':
        response = a_star_path(graph, destination,returnToChangi,travelTime)
    elif algorithm == 'bellman_ford':
        response = bellman_ford_path(graph, destination,returnToChangi,travelTime)
    elif algorithm == 'all_distance':
        algorithms = ['dijkstra', 'a_star', 'bellman_ford',]

        results = {}
        route_results = {}

        for algorithm in algorithms:
            print(" ")
            print(algorithm)
            route_result = calculate_route(algorithm, graph, destination,returnToChangi,travelTime)
            print(route_result)

            # Add the distance to results
            results[algorithm] = route_result['distance']
            route_results[algorithm] = route_result

        best_algorithm = min(results, key=results.get)
        best_route_result = route_results[best_algorithm]

        print(" ")

        print("Best algorithm: " + best_algorithm)
        formatted_dict = {}

        # Loop through the original dictionary and format the values
        for key, value in results.items():
            formatted_value = round(value / 1000, 2) # Round the value to 2 decimal places
            formatted_dict[key] = formatted_value
        print("Best algorithm Distance to Destination ", (formatted_dict[best_algorithm]), "Km")

        return jsonify(best_route_result)
    elif algorithm == 'all_speed':
        algorithms = ['dijkstra', 'a_star', 'bellman_ford']
        results = {}
        route_results = {}

        for algorithm in algorithms:
            print(" ")
            print(algorithm)
            route_result = calculate_route(algorithm, graph, destination,returnToChangi,travelTime)
            print(route_result)

            # Add the distance to results
            results[algorithm] = route_result['time']
            route_results[algorithm] = route_result

        best_algorithm = min(results, key=results.get)
        best_route_result = route_results[best_algorithm]


        print(" ")
        print("Best algorithm: " + best_algorithm)
        formatted_dict = {}

        # Loop through the original dictionary and format the values
        for key, value in results.items():
            formatted_value = round(value, 2)  # Round the value to 2 decimal places
            formatted_dict[key] = formatted_value
        print("Best algorithm Time to Destination ", (formatted_dict[best_algorithm]), "Minutes")

        return jsonify(best_route_result)

    elif algorithm == 'Fastest_Run_Time':
        algorithms = ['dijkstra', 'a_star', 'bellman_ford']
        results = {}
        route_results = {}

        for algorithm in algorithms:
            print(" ")
            print(algorithm)
            route_result = calculate_route(algorithm, graph, destination, returnToChangi, travelTime)
            print(route_result)

            # Add the distance to results
            results[algorithm] = route_result['execution_time']
            route_results[algorithm] = route_result
            print(results[algorithm])

        best_algorithm = min(results, key=results.get)
        best_route_result = route_results[best_algorithm]

        print(" ")
        print("Best algorithm: " + best_algorithm)

        formatted_dict = {}

        # Loop through the original dictionary and format the values
        for key, value in results.items():
            formatted_value = round(value, 4)  # Round the value to 2 decimal places
            formatted_dict[key] = formatted_value
        print("Best algorithm Run Time ", (formatted_dict[best_algorithm]), "Seconds")

        return jsonify(best_route_result)
    # Append the incident nodes to the response
    response["incident_coords"] = incident_coords.tolist()

    print(response)
    return jsonify(response)

# Define a function that calls the appropriate routing algorithm based on the input
def calculate_route(algorithm,graph,destination,returnToChangi, travelTime):
    if algorithm == 'a_star':
        return a_star_path(graph,destination,returnToChangi,travelTime)
    elif algorithm == 'dijkstra':
        return dijkstra_path(graph,destination,returnToChangi,travelTime)
    elif algorithm == 'bellman_ford':
        return bellman_ford_path(graph,destination,returnToChangi,travelTime)
    # handle 'all_distance', 'all_speed', and 'Fastest_Run_Time' algorithms...
    else:
        return {}

if __name__ == '__main__':
    app.run(debug=True)
