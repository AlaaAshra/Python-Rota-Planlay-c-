#import helpers_fun as cl
import heapq as heap
import time
import math
import function as cj

def draw_filled_circle(map_widget, center, radius):
    radius = radius 
    num_points = 100
    circle_points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        point = (center[0] + (dx / 111320), center[1] + (dy / (111320 * math.cos(math.radians(center[0])))))
        circle_points.append(point)
        
    map_widget.set_polygon(circle_points, outline_color="red", fill_color="red")

def diagonal_distance(lat1, lon1, lat2, lon2):
    lat_diff = abs(lat2 - lat1)
    lon_diff = abs(lon2 - lon1)
    return max(lat_diff, lon_diff)
    # Set the path and fill the area with red color
    map_widget.set_polygon(circle_points, outline_color="red", fill_color="red")
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


# Function to check if a point is inside the circle
def is_point_in_circle(point,circle_center, radius):
    distance = haversine1(circle_center[0], circle_center[1], point[0], point[1])
    return distance <= radius

def aStar(source, destination, hss_koordinat_bilgileri):
    node_hss = cj.filter_node(hss_koordinat_bilgileri)

    open_list = []
    g_values = {}
    path = {}
    closed_list = {}
    
    sourceID = cj.getOSMId(source[0], source[1])
    print(sourceID)
    destID = cj.getOSMId(destination[0], destination[1])

   
    g_values[sourceID] = 0
    h_source = cj.calculateHeuristic(source, destination,node_hss)
    
    open_list.append((h_source, sourceID))
            #closed_list = node_hss

    s = time.time()
    while len(open_list) > 0:
        curr_state = open_list[0][1]
        heap.heappop(open_list)
        closed_list[curr_state] = ""
        
        if curr_state == destID:
            print("Hedafe geldik")
            break 
        
        nbrs = cj.getNeighbours(curr_state, destination, hss_koordinat_bilgileri)
        
        values = nbrs[curr_state]
        #print (values)
        for eachNeighbour in values:
            neighbourId, neighbourHeuristic, neighbourCost, neighbourLatLon = cj.getNeighbourInfo(eachNeighbour)
            current_inherited_cost = g_values[curr_state] + neighbourCost
            
            if neighbourId in closed_list:
                continue
            else:
                g_values[neighbourId] = current_inherited_cost
                neighbourFvalue = neighbourHeuristic + current_inherited_cost
                
                open_list.append((neighbourFvalue, neighbourId))
            path[str(neighbourLatLon)] = {"parent": str(cj.getLatLon(curr_state)), "cost": neighbourCost}
            # Check if the neighbor is inside any red circle (air defense system)
        
                                                
        open_list = list(set(open_list))
        heap.heapify(open_list)
        #print(time.time()-s)
        #print(type(s))
    
    print("Time taken to find path(in second): "+str(time.time()-s))
    print (f"lenghth of list{len(path)}")
    if len(path)>360:
        return []
    return path


   
          