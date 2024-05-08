# SmartRoute

## Team Members

- Eric Koh Meng Hwan
- Lester Tan Yi Teck
- De Chavez Karlo Miguel Bautist
- Tan Heng Joo

## Project Objectives
SmartRoute brings to you an unparalleled, intelligent transportation solution, designed for seamless transfers from Singapore's Changi Airport Terminal 3 to a selection of prestigious hotels. As a transportation service provider, we appreciate the critical role that fuel and ERP expenses play in influencing operating costs. SmartRoute isn't merely an instrument for fuel economy; it is an innovation designed to revolutionize your transportation experience.

Using the power of advanced algorithms, SmartRoute provides the most efficient routes for drivers to undertake. This not only optimizes fuel consumption but also ensures a superior, seamless travel experience for the passengers. SmartRoute is our commitment to efficiency, budget, time efficient, and resource optimization, poised to redefine the norms of the transportation industry.

SmartRoute is a comprehensive all-in-one platform. It allows transportation companies to strategize their routes efficiently, while also offering an additional feature such as accurately predicting cost and fuel consumption based on the travel distance. An additional convenience that SmartRoute offers to bus drivers is real-time map access, which greatly enhances their navigation experience.

We acknowledge the crucial role of pre-travel planning and real-time guidance in ensuring seamless journey management. To allow SmartRoute to be more dynamic it is equipped to take into account real-time road conditions, including accidents and roadworks. This ensures a more reliable, safer, and time-efficient journey, improving overall travel management significantly.

By considering factors like fuel consumption, distance, Electronic Road Pricing (ERP), and real-time road conditions, SmartRoute simplifies complex decision-making, ensuring a more economical and efficient journey. With SmartRoute, resources can be focused on more intricate issues, leading to improved operational efficiency, strategic resource allocation, and superior financial management.

## Algorithms Implemented
1) Dijkstra's Algorithm
2) A* Algorithm
3) Bellman-Ford Algorithm
4) Permutation Method for Travelling Salesman Problem (TSP)


### Datasets Used
1. DataMall: DataMall is an online data resource managed by the Singapore government, and it provides a wide array of datasets. The ERP rates are necessary to account for the varying costs of driving in different parts of Singapore. Traffic incident data is crucial in adjusting the routes based on real-time traffic situations.

2. OSMnx library is used to work with OpenStreetMap data. This helps to find the shortest path between several destinations, calculate the actual distance of a path etc. Which is crucial for path optimization algorithms.

3. The geopy library is being used to perform geocoding and compute the geodesic distance between two points, which is used in calculating the actual distance of the path and finding the nearest nodes to specified geolocations.

4. Manually Scraped Hotel and Gantry Locations Data