"""
To get Roadpoints as per the pipeline for Roads digitized manually with 
reference to Google Maps, making sure the GSV images are available in them.
"""
import requests
import numpy as np
import math
import pandas as pd
import geopandas
from shapely.geometry import LineString, Point, Polygon
from os.path import join

# make sure the shp is Line geometry in 4326 projection
root_folder = r"C:\Users\atree\Desktop\GSV\Vidhya"
SHAPEFILE_PATH = join(root_folder, "lines", "shps_4326", "CWS_Roads_4326.shp")
output_folder = join(root_folder, "roadPoints") # output CSV files
EARTH_RADIUS = 6371e3  # in meters
DISTANCE_DELTA = 0.0001  # used for interpolating points along the road
PERPENDICULAR_DISTANCE = 15  # 30m distance for calculating field points

def compute_bearing(from_point, to_point):
    """Calculate the bearing from one geographic point to another."""
    y = math.sin(to_point[1] - from_point[1]) * math.cos(to_point[0])
    x = math.cos(from_point[0]) * math.sin(to_point[0]) - \
        math.sin(from_point[0]) * math.cos(to_point[0]) * math.cos(to_point[1] - from_point[1])
    θ = math.atan2(y, x)
    bearing = (θ * 180 / math.pi + 360) % 360
    return bearing

def compute_point_on_field(from_point, theta, distance):
    """Calculate a point at a certain distance and bearing from a given point."""
    angular_distance = distance / EARTH_RADIUS
    theta = math.radians(theta)
    lat1 = math.radians(from_point[0])
    lon1 = math.radians(from_point[1])
    lat2 = math.asin(math.sin(lat1) * math.cos(angular_distance) + \
                     math.cos(lat1) * math.sin(angular_distance) * math.cos(theta))
    lon2 = lon1 + math.atan2(math.sin(theta) * math.sin(angular_distance) * math.cos(lat1),
                             math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2))
    return (math.degrees(lat2), math.degrees(lon2))

def process_road_data(shapefile_path):
    geo_data = geopandas.read_file(shapefile_path)
    road_points, field_points, original_points = [], [], []
    for geo_idx, line in enumerate(geo_data.geometry):
        print('GEO Index ', geo_idx)
        try:
            process_way_element(line, road_points, field_points, original_points)
        except Exception as e:
            print(e)

        save_to_csv(road_points, join(output_folder, f"roadPointsNW4_{geo_idx}.csv"), "y,x,b,x1,y1,x2,y2")
        # save_to_csv(field_points, join(output_folder, f"fieldPointsNW4_{geo_idx}.csv"), "y,x,b,yr,xr")
        # save_to_csv(original_points, join(output_folder, f"osmRoadsNW4_{geo_idx}.csv"), "y,x")

def save_to_csv(data, filename, header):
    """Save data to a CSV file."""
    np.savetxt(filename, data, delimiter=",", fmt='%f', header=header, comments='')

def process_way_element(line, road_points, field_points, original_points):
    distances = np.arange(0, line.length, DISTANCE_DELTA)
    points = [line.interpolate(distance) for distance in distances]
    if line.boundary.length > 1:
        points.append(line.boundary[1])
    if len(points) < 2:
        return
    new_line = LineString(points)
    process_line_points(new_line, road_points, field_points)

def process_line_points(line, road_points, field_points):
    """Process points along a line and compute adjacent field points."""
    old_x, old_y = None, None

    for j, (x, y) in enumerate(line.coords):
        if j > 3 and j< len(line.coords)-3:

            from_point = (old_y, old_x)#(old_x, old_y)
            to_point = (y, x)#(x, y)
            bearing = compute_bearing(from_point, to_point)
            p1 = compute_point_on_field(to_point, (bearing + 90) % 360, PERPENDICULAR_DISTANCE)
            p2 = compute_point_on_field(to_point, (bearing + 270) % 360, PERPENDICULAR_DISTANCE)
            field_points.append((p1[0], p1[1], (bearing + 90) % 360, x, y))
            field_points.append((p2[0], p2[1], (bearing + 270) % 360, x, y))
            road_points.append((y,x,bearing,p1[1],p1[0],p2[1],p2[0]))#(x, y, bearing, p1[0], p1[1], p2[0], p2[1]))
        old_x, old_y = x, y

if __name__ == "__main__":
    process_road_data(SHAPEFILE_PATH)
    # print(compute_bearing((14.096708, 77.366654), (14.096716, 77.366554)))
    # print(compute_point_on_field((14.096716, 77.366554), (333.1840360399445+90)%360, 30))
    # print(compute_bearing((22.691664, 86.110429), (22.691653, 86.110529))) # 261.8141881821835
    # print(compute_bearing((86.110429, 22.691664), (86.110529, 22.691653))) # 1.7609774839085048
    # print(compute_point_on_field((22.691507,86.113419),(262.591463+90)%360,30))
    # print(compute_point_on_field((22.694224,86.115453),(3.754053+90)%360,30))
