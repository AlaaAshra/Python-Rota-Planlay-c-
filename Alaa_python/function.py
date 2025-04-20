import xmltodict
from haversine import haversine
import time
import numpy as np
from sklearn.neighbors import KDTree
import math
s = time.time()
doc = {}
with open('adana_graph.graphml') as fd:
    doc = xmltodict.parse(fd.read())
print(time.time()-s)

hss_nodes = []
latlon_nodes = []
# Function to calculate the distance between two points (Haversine formula)
def haversine1(lat1, lon1, lat2, lon2):
    R = 6371e3  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance
def diagonal_distance(lat1, lon1, lat2, lon2):
    lat_diff = abs(lat2 - lat1)
    lon_diff = abs(lon2 - lon1)
    return max(lat_diff, lon_diff)

def diagonal_distance11(x1, y1, x2, y2):
    D=0.0001
    D2=0.0001
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    distance = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    return distance

# Function to check if a point is inside the circle
def is_point_in_circle(point,circle_center, radius):
    radius = radius + 50
    distance = haversine1(circle_center[0], circle_center[1], point[0], point[1])
    return distance <= radius

def is_point_in_circle_asy(point,hssler,oran):

    for hss in hssler:
        circle_center = (hss["hssEnlem"], hss["hssBoylam"])
        radius = hss["hssYaricap"]

        radius = radius + oran
        distance = haversine1(circle_center[0], circle_center[1], point[0], point[1])
        if distance <= radius:
            return True
        
    return False


def filter_node(hss_koordinat_bilgileri):
    nodes = doc['graphml']['graph']['node']
    filtered_nodes = []  # List to store nodes that are not within HSS circles

    for eachNode in range(len(nodes)):
        lat = float(nodes[eachNode]["data"][0]["#text"])
        lon = float(nodes[eachNode]["data"][1]["#text"])
        node_in_hss = False

        for hss in hss_koordinat_bilgileri:
            circle_center = (hss["hssEnlem"], hss["hssBoylam"])
            radius = hss["hssYaricap"] 

            if is_point_in_circle((lat, lon), circle_center, radius):
                node_in_hss = True
                break

        if not node_in_hss:
            filtered_nodes.append(nodes[eachNode])

    doc['graphml']['graph']['node'] = filtered_nodes  # Update the doc with filtered nodes
    return filtered_nodes


def filter_node11 (hss_koordinat_bilgileri):
    nodes = doc['graphml']['graph']['node']
    for eachNode in range(len(nodes)):
       lat = float(nodes[eachNode]["data"][0]["#text"])
       lon = float(nodes[eachNode]["data"][1]["#text"])
       id = nodes[eachNode]["data"][2]["#text"]
       
       for hss in hss_koordinat_bilgileri:
                circle_center = (hss["hssEnlem"], hss["hssBoylam"])
                radius = hss["hssYaricap"] 
                if is_point_in_circle((lat,lon), circle_center, radius):
                   
                   latlon_nodes.append((lat,lon))
                   hss_nodes.append(id)
                 # nodes[eachNode]["data"][3]["#text"] = "False"
    return latlon_nodes
       
     
def getLatLon(OSMId):
    lat, lon = 0, 0
    nodes = doc['graphml']['graph']['node']
    for eachNode in range(len(nodes)):
        if(nodes[eachNode]["@id"] == str(OSMId)):
            lat = float(nodes[eachNode]["data"][0]["#text"])
            lon = float(nodes[eachNode]["data"][1]["#text"])
    return (lat, lon)

def getOSMId(lat, lon):
    OSMId = 0
    nodes = doc['graphml']['graph']['node']
    for eachNode in range(len(nodes)):
        if(nodes[eachNode]["data"][0]["#text"]==str(lat)):
            if(nodes[eachNode]["data"][1]["#text"]==str(lon)):
                OSMId = nodes[eachNode]["data"][2]["#text"]
            
    return OSMId

import math

def calculateHeuristic11111(curr, destination, hss_points):
    """
    Calculates the heuristic distance from the current point to the destination,
    avoiding restricted areas (HSS points).
    
    :param curr: tuple (latitude, longitude) of the current point
    :param destination: tuple (latitude, longitude) of the destination point
    :param hss_points: list of tuples [(lat, lon), ...] representing restricted areas
    :return: heuristic cost considering restricted areas
    """
    
    # Straight-line (Euclidean) distance between current point and destination
    straight_line_distance = math.sqrt((curr[0] - destination[0])**2 + (curr[1] - destination[1])**2)
    
    # Penalty for being close to any restricted area
    penalty = 0
    
    for hss in hss_points:
        # Calculate distance to the restricted point
        distance_to_hss = math.sqrt((curr[0] - hss[0])**2 + (curr[1] - hss[1])**2)
        
        # If the distance is small, add a penalty to the heuristic
        if distance_to_hss < 0.01:  # Threshold for proximity to restricted area
            penalty += (0.01 - distance_to_hss) * 100  # Increase penalty as the point gets closer to the restricted area
    
    # The heuristic is the straight-line distance plus any penalty
    heuristic = straight_line_distance + penalty
    
    return heuristic

    
def calculateHeuristic(curr,destination,hss):

    return (haversine1(curr[0],curr[1],destination[0],destination[1]))

def getNeighbours(OSMId, destinationLetLon, hss_koordinat_bilgileri):
    neighbourDict = {}
    tempList = []
    edges = doc['graphml']['graph']['edge']
    for eachEdge in range(len(edges)):
        if(edges[eachEdge]["@source"]==str(OSMId)):
            
            temp_nbr = {}
            
            neighbourCost = 0
            neighbourId = edges[eachEdge]["@target"]
            #if neighbourId in hss_nodes:
             #   continue
            neighbourLatLon = getLatLon(neighbourId) 
            dataPoints = edges[eachEdge]["data"]

            
            
            for eachData in range(len(dataPoints)):
                if(dataPoints[eachData]["@key"]=="d4"):
                    neighbourCost = dataPoints[eachData]["#text"]
            
            neighborHeuristic = calculateHeuristic(neighbourLatLon, destinationLetLon,latlon_nodes)
            
            temp_nbr[neighbourId] = [neighbourLatLon, neighbourCost, neighborHeuristic]
            tempList.append(temp_nbr)
            
    neighbourDict[OSMId] = tempList
    return (neighbourDict)

def getNeighbourInfo(neighbourDict):
    neighbourId = 0
    neighbourHeuristic = 0
    neighbourCost = 0
    for key, value in neighbourDict.items():
        
        neighbourId = key
        neighbourHeuristic = float(value[2])
        neighbourCost = float(value[1])/1000
        neighbourLatLon = value[0]
        
    return neighbourId, neighbourHeuristic, neighbourCost, neighbourLatLon

#Argument should be tuple

def getKNN(pointLocation):
    nodes = doc["graphml"]["graph"]["node"]
    locations = []
    for eachNode in range(len(nodes)):
        locations.append((nodes[eachNode]["data"][0]["#text"],nodes[eachNode]["data"][1]["#text"]))

    locations_arr = np.asarray(locations, dtype=np.float32)
    point = np.asarray(pointLocation, dtype=np.float32)

    tree = KDTree(locations_arr, leaf_size=2)
    dist, ind = tree.query(point.reshape(1,-1), k=3) 
    
    nearestNeighbourLoc = (float(locations[ind[0][0]][0]), float(locations[ind[0][0]][1]))
    
    return nearestNeighbourLoc
    
def getResponsePathDict(paths, source, destination):
    finalPath = []
    child = destination
    parent = ()
    cost = 0
    print(destination)
    while(parent!=source):
        tempDict = {}
        cost = cost + float(paths[str(child)]["cost"])
        parent = paths[str(child)]["parent"]
        parent = tuple(float(x) for x in parent.strip('()').split(','))
        
        tempDict["lat"] = parent[0]
        tempDict["lng"] = parent[1]
        
        finalPath.append(tempDict)
        child = parent
        
    return finalPath, cost
def draw_nodes_from_graphml():
    centerlist = []
    nodes = doc['graphml']['graph']['node']
    for eachNode in range(len(nodes)):
            lat = float(nodes[eachNode]["data"][0]["#text"])
            lon = float(nodes[eachNode]["data"][1]["#text"])
        # Find the x and y coordinates for each node
            centerlist.append((lat,lon))
    #print(centerlist)        
    return centerlist    
        
#(41.55534330484295, 32.296519652480924)
#41.5553430 32.2965203
#(41.55534330484295, 32.296519652480924)

def moving_average_smoothing(path, window_size=3):
    smoothed_path = []
    
    for i in range(len(path)):
        if i < window_size // 2 or i > len(path) - window_size // 2 - 1:
            smoothed_path.append(path[i])
        else:
            avg_lat = np.mean([point[0] for point in path[i - window_size // 2:i + window_size // 2 + 1]])
            avg_lon = np.mean([point[1] for point in path[i - window_size // 2:i + window_size // 2 + 1]])
            smoothed_path.append((avg_lat, avg_lon))
    
    return smoothed_path






"""def is_point_in_circle11(point, circle_center, radius):
    

    radius_in_degrees = (radius / 111320) * 1.7
  # Approximation: 1 degree ~ 111.32 km

    # Calculate the distance between the point and the circle center
    distance = math.sqrt((point[0] - circle_center[0])**2 + (point[1] - circle_center[1])**2)

    return distance <= radius_in_degrees"""
