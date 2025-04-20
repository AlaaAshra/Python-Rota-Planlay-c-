import tkinter
import tkintermapview
#from PIL import Image, ImageTk
import requests
import aStar as algo    
import math
import numpy as np
from function import *
# create map widget
# Step 1: Define the bounds
min_bound = (36.942314, 35.563323)
max_bound = (36.937864, 35.562873)
# create tkinter window

root_tk = tkinter.Tk()
root_tk.geometry(f"{1000}x{700}")
root_tk.title("Map View Example")
map_widget = tkintermapview.TkinterMapView(root_tk, width=1000, height=700, corner_radius=0)
map_widget.pack(fill="both", expand=True)
map_widget.set_position((min_bound[0] + max_bound[0]) / 2, (min_bound[1] + max_bound[1]) / 2)
map_widget.set_zoom(15)

def borders_to_map(map_widget):
    map_widget.delete_all_polygon()  # Mevcut tüm poligonları temizle
def update_hss_operations(hss_koordinat_bilgileri):
    map_widget.delete_all_polygon()  # Clear existing circles
    borders_to_map(map_widget)
    
    # Update HSS data
    for item in hss_koordinat_bilgileri:
        lat = item["hssEnlem"]
        lon = item["hssBoylam"]
        radius = item["hssYaricap"]
        if radius > 0:
            algo.draw_filled_circle(map_widget, (lat, lon), radius)

hss_koordinat_bilgileri = []
try:
        response = requests.get("http://yazilimtesti.name.tr/api/hss_koordinatlari", verify=False)
        response.raise_for_status()
        data = response.json()
        hss_koordinat_bilgileri = data.get("hss_koordinat_bilgileri", [])
except requests.exceptions.RequestException as e:
        print(f"Failed to fetch hss_koordinatlari data: {e}") 

print(hss_koordinat_bilgileri)
# Step 2: Create a grid of nodes
def draw_polygon(center, radius=0.0001, color="blue"):
    num_points = 10
    circle_points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        point = (center[0] + (dx / 111320), center[1] + (dy / (111320 * math.cos(math.radians(center[0])))))
        circle_points.append(point)
    map_widget.set_polygon(circle_points, outline_color=color, fill_color=color)  
    
def is_inside_any_circle(waypoint, circles):
    for hss in circles:
        circle_center = (hss["hssEnlem"], hss["hssBoylam"])
        radius = hss["hssYaricap"]+50
        if algo.is_point_in_circle(waypoint, circle_center, radius):
            return True
    return False

def a_star_pathfinding(start, goal, hss_koordinat_bilgileri):
   
   sor = start
   des= goal
   print (f"from {start} to {goal}")
   source = getKNN(sor)
   destination = getKNN(des)
   pathes = algo.aStar(source, destination, hss_koordinat_bilgileri)
   #print(f"pathesssssssssssssssssss{pathes}")
   if len(pathes)>0:
    finalPath, cost = getResponsePathDict(pathes, source, destination)
    path_list = [(point['lat'], point['lng']) for point in finalPath]
   #path_list.insert(0,destination)
    path_list.pop()
    path_list[0] = goal 
    path_list [-1] = start
   else:
       return []
   return path_list
"""
waypoint_list = [(-35.37694699999999,149.15868539999997),
                 (-35.3729093, 149.1608780),
                 (-35.3684935, 149.1651413),
                 (-35.3687800, 149.1599767),               
                 (-35.3657752, 149.1611660),
                 (-35.3637978, 149.1627968)]

waypoint_list = [(41.55463759175291, 32.29338954040092),
                (41.55467639092765, 32.2944135413057),
                (41.554841287160414, 32.29556716257816),
                (41.5550825, 32.2984926),
                (41.5562101, 32.2984240),
                (41.5557645, 32.2954896)
                ]                 
"""
with open("new.waypoints", "r") as file:
    lines = file.readlines()

# Initialize an empty list to store lat and long
waypoint_list = []
excluded_list = []
first_waypoints = False 
# Loop through the lines, skipping the first and last lines
for line in lines[2:]:  # Start from the third line, stop before the last line
    # Split the line into fields
    fields = line.strip().split("\t")
    
    # Extract the latitude and longitude (columns 9 and 10)
    if fields[3] == '16' and not first_waypoints:
     latitude = float(fields[8])
     longitude = float(fields[9])
     waypoint_list.append((latitude, longitude))
    elif fields[3] == '177':
        first_waypoints = True
        excluded_list.append(fields)
    elif first_waypoints:
        excluded_list.append(fields)


print(f"lat_long_list:{waypoint_list}")



def interpolate_points(start, goal, num_points=10):
    
    lat_points = np.linspace(start[0], goal[0], num_points)
    lon_points = np.linspace(start[1], goal[1], num_points)
    return list(zip(lat_points, lon_points))

def path_between_waypoints(updated_waypoints, hss_koordinat_bilgileri):
    final_path = []
    passing = False
    nopass_marker = 0
    # Iterate through each pair of consecutive waypoints
    for i in range(len(updated_waypoints) - 1):
        
        if passing:            
          start = updated_waypoints[i-nopass_marker]
          goal = updated_waypoints[i + 1]
        else:              
          start = updated_waypoints[i]
          goal = updated_waypoints[i + 1]

        passing = False
        # Generate intermediate points between start and goal
        path_points = interpolate_points (start, goal, num_points=20) #hayali

        # Flag to check if any point in the path intersects restricted zones
        intersects = False

        for point in path_points:
            for hss in hss_koordinat_bilgileri:
                circle_center = (hss["hssEnlem"], hss["hssBoylam"])
                radius = hss["hssYaricap"]

                if is_point_in_circle(point, circle_center, radius): #çizgi kontrol
                    intersects = True
                    break
            if intersects:
                break
        if intersects:
            # Run A* pathfinding to find a path between start and goal

            dogru_yol = []
            yeni_yol = [] 


            sub_path = a_star_pathfinding(start, goal, hss_koordinat_bilgileri)
            if not sub_path:
                nopass_marker=nopass_marker+1
                passing = True
                continue
            print(f"sub_path = {sub_path}")
            duzgun_flag = sub_path[0]
            yeni_yol.append(duzgun_flag)
            bitir = 0
            for i in range(len(sub_path) - 1):
            
                print(f"duzgun_flag = {duzgun_flag}")
                print(f" a_yildiz_nodlari[i+1] = {sub_path[i+1]}")
                
                asy_iki_nokta_arasindaki_hayali_yolu_olusturan_noktalar = interpolate_points(duzgun_flag, sub_path[i+1], num_points=20) #hayali

                    # Buradaki noktaların bir sonrakine for içerisinde erişebildiğini test etmelisin
                carptim = False
                for j , asy_hayali_cizginin_noktasi in enumerate(asy_iki_nokta_arasindaki_hayali_yolu_olusturan_noktalar):
                            if carptim:
                                continue
                            if bitir == 1:
                                break
                                

                            if is_point_in_circle_asy(asy_hayali_cizginin_noktasi, hss_koordinat_bilgileri, 50): #çizgi kontrol
                                    print("HSS'ye çarptım")
                                        
                                    if len(dogru_yol) > 0:
                                        duzgun_flag = dogru_yol[-1]
                                        dogru_yol = []
                                        yeni_yol.append(duzgun_flag)
                                        carptim = True
                                        
                                    else:
                                        carptim = True
                                        
                            else: 
                                    if asy_hayali_cizginin_noktasi == asy_iki_nokta_arasindaki_hayali_yolu_olusturan_noktalar[len(asy_iki_nokta_arasindaki_hayali_yolu_olusturan_noktalar) - 1]:
                                        print("flag ile çizilen nokta arasında herhangi bir hss tespit edilemediğinden dolayı son nokta dogru_yol'a eklendi")
                                        dogru_yol.append(sub_path[i])
                                   
                                    if sub_path[i+1] == sub_path[-1]:
                                        print("son elemani yeni yola ekledim")
                                        yeni_yol.append(sub_path[i+1])
                                        bitir = 1

            print(f"Yeni Yollarim: {yeni_yol} uzunluğu: {len(yeni_yol)}")
            sub_path = yeni_yol
            
            sub_path.reverse()
            final_path.extend(sub_path)
        else:
            if i == 0:
                final_path.append(start)           # No intersection, add all points in this segment
            final_path.append(goal)
            
    return final_path


for way in waypoint_list:
    map_widget.set_marker(way[0],way[1])
updated_waypoints = []
update_hss_operations(hss_koordinat_bilgileri)
filter_node(hss_koordinat_bilgileri)


for waypoint in waypoint_list:
   if not is_inside_any_circle(waypoint, hss_koordinat_bilgileri):
    updated_waypoints.append(waypoint)



final_path = path_between_waypoints(updated_waypoints, hss_koordinat_bilgileri)
print(f"sonnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn{len(final_path)}")
print(f"path'in son hali {final_path}")

#donus_path=[]
#baslama = final_path [-1]
#son = final_path[0]
#donus_path = a_star_pathfinding(baslama,son,hss_koordinat_bilgileri)
#donus_path.append(baslama)
#donus_path.insert(0,son)
#print(f"final list :{final_path}")

#smoothed_path = moving_average_smoothing(final_path)

map_widget.set_path(final_path, width=4, name="Gidiş")
#map_widget.set_path(donus_path, width=2, color='red', name="dönüş")

for point in final_path:
    map_widget.set_marker(point[0],point[1])



graphml_file='adana_graph.graphml'
pathess= draw_nodes_from_graphml()
#print(f"pathess : {pathess}")
for node in pathess:
    lat = node[0]
    lon = node[1]    
    center =(lat,lon)
    draw_polygon(center=center, radius=0.0001, color="blue")    



waypoints_file = "points.waypoints"
# Open the existing file to read its content
with open(waypoints_file, mode='r') as file:
    existing_content = file.readlines()

# Write the path_list to the .waypoints file in the specified format
with open(waypoints_file, mode='w') as file:
    # Write the header
    #file.write("QGC WPL 110\n")
    #file.write("0\t0\t0\t16\t0.000000\t0.000000\t0.000000\t0.000000\t-35.362680\t149.165182\t584.049988\t1\n")
    #file.writelines(existing_content[0])
    #file.writelines(existing_content[1])
    file.write(lines[0])  # Write the header line
    file.write(lines[1])  # Write the second line
    # Write each waypoint
    counter  = 0
    for index, (lat, lon) in enumerate(final_path, start=1):
        altitude = 100.0  # Default altitude, you can change this as needed
        file.write(f"{index}\t0\t3\t16\t0.000000\t0.000000\t0.000000\t0.000000\t{lat:.6f}\t{lon:.6f}\t{altitude:.6f}\t1\n")
        counter = index
    counter = counter +1    
    for index , fields in enumerate(excluded_list, start=counter):
        # Join the fields back into a tab-separated string and write it to the file
        fields[0] = str(index)
        file.write("\t".join(fields) + "\n")
print(f"Waypoint file '{waypoints_file}' has been created successfully.")


root_tk.mainloop()
