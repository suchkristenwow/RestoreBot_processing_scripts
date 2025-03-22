import numpy as np
from geopy.distance import geodesic
from geopy import Point
import math
import os
import csv
import matplotlib.pyplot as plt


def inverseVincenty(lat1, lon1, lat2, lon2):
    """
    Calculate the range (in meters) and bearing (in degrees) from the first GPS coordinate to the second.
    """
    point1 = (lat1, lon1)
    point2 = (lat2, lon2)
    range_m = geodesic(point1, point2).meters

    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    lon1_rad = math.radians(lon1)
    lon2_rad = math.radians(lon2)

    delta_lon = lon2_rad - lon1_rad
    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    bearing_rad = math.atan2(x, y)
    bearing_deg = (math.degrees(bearing_rad) + 360) % 360

    return range_m, bearing_deg


def calculate_new_gps(lat, lon, range_m, bearing_deg):
    """
    Calculate a new GPS coordinate given an initial point, range (in meters), and bearing (in degrees).
    """
    start_point = Point(lat, lon)
    destination = geodesic(meters=range_m).destination(start_point, bearing_deg)
    return destination.latitude, destination.longitude


# Load measurements and compass data
measurements = np.genfromtxt("May_results/2drill/yawHeading_data.csv", delimiter=",", skip_header=1)
compass_data = np.genfromtxt("May_results/2drill/processed_compass_heading.csv", delimiter=",", skip_header=1)  # t, compassHeading


# Initialize results file if it doesn't exist
if not os.path.exists("May_results/2drill/results.csv"):
    with open("May_results/2drill/results.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp', 'lat', 'lon', 'pointer_lat', 'pointer_lon',
                         'ouster_lat', 'ouster_lon', 'frontLeftWheel_lat', 'frontLeftWheel_lon',
                         'frontRightWheel_lat', 'frontRightWheel_lon', 'rearLeftWheel_lat', 'rearLeftWheel_lon',
                         'rearRightWheel_lat', 'rearRightWheel_lon', 'cam_lat', 'cam_lon',
                         'topRight_frame_lat', 'topRight_frame_lon', 'topLeft_frame_lat', 'topLeft_frame_lon',
                         'bottomRight_frame_lat', 'bottomRight_frame_lon', 'bottomLeft_frame_lat', 'bottomLeft_frame_lon'])

'''
[msg_tstamp, self.yaw, self.angularVel, heading, 
lat, lon, 
self.filtered_lat, self.filtered_lon, 
self.position_covar[0], self.position_covar[1], self.position_covar[2],
self.position_covar[3], self.position_covar[4], self.position_covar[5],
self.position_covar[6], self.position_covar[7], self.position_covar[8]]
'''
for measurement in measurements:
    t = measurement[0]
    print("t:",t)
    lat, lon = measurement[4], measurement[5]
    print("lat: {}, lon: {}".format(lat,lon)) 

    # Validate GPS coordinates
    if not isinstance(lat, float) or not isinstance(lon, float):
        continue
    if np.isnan(lat) or np.isnan(lon):
        continue
    if int(lat) != 38 or int(lon) != -109:
        continue

    # Get compass heading
    idx = np.argmin(np.abs(compass_data[:, 0] - t))
    if np.abs(compass_data[idx, 0] - t) > 0.1:
        if idx > 0 and idx < len(compass_data) - 1:
            compass_heading_t = np.mean([compass_data[idx - 1, 1], compass_data[idx + 1, 1]])
        else:
            raise OSError("No valid compass heading interpolation.")
    else:
        compass_heading_t = compass_data[idx, 1]

    if np.isnan(compass_heading_t): 
        print("WARNING: compass heading is nan ...")
        continue 

    print("compass_heading_t: ",compass_heading_t) 

    compass_heading_rad = np.deg2rad(compass_heading_t)

    # Calculate GPS coordinates for components
    pointer_lat, pointer_lon = calculate_new_gps(lat, lon, 1.5, compass_heading_t)

    ouster_range = np.linalg.norm([0.664, 0.192])
    ouster_bearing_rad = compass_heading_rad + (np.pi / 2 - np.arctan2(0.664, 0.192))

    ouster_lat, ouster_lon = calculate_new_gps(lat, lon, ouster_range, np.rad2deg(ouster_bearing_rad % (2 * np.pi)))

    wheels = {
        'frontLeft': (0.557, 0.097),
        'frontRight': (0.557, -0.474),
        'rearLeft': (0.045, 0.097),
        'rearRight': (0.045, -0.474)
    }

    wheel_coords = {}
    for wheel, (x, y) in wheels.items():
        range_m = np.linalg.norm([x, y])
        bearing_rad = compass_heading_rad - (np.pi / 2 - np.arctan2(x, y))
        lat_lon = calculate_new_gps(lat, lon, range_m, np.rad2deg(bearing_rad % (2 * np.pi)))
        wheel_coords[wheel] = lat_lon

    cam_range = np.linalg.norm([0.703, 0.192])
    cam_bearing_rad = compass_heading_rad + (np.pi / 2 - np.arctan2(0.703, 0.192))
    cam_lat, cam_lon = calculate_new_gps(lat, lon, cam_range, np.rad2deg(cam_bearing_rad % (2 * np.pi)))

    frame_corners = {
        'topRight': (0.0724, 0.78),
        'bottomRight': (0.0747, -1.07),
        'bottomLeft': (0.0745, -2.34),
        'topLeft': (0.0721, 2.09)
    }

    frame_coords = {}
    for corner, (range_m, bearing_rad) in frame_corners.items():
        corner_bearing_rad = compass_heading_rad + bearing_rad
        frame_coords[corner] = calculate_new_gps(cam_lat, cam_lon, range_m, np.rad2deg(corner_bearing_rad % (2 * np.pi)))
        print("frame_coords[corner]: ",frame_coords[corner]) 

    # Write results
    with open("May_results/2drill/results.csv", mode='a', newline='') as file:
        writer = csv.writer(file)
        print("writing row: ",[t, lat, lon, pointer_lat, pointer_lon,
                         ouster_lat, ouster_lon,
                         wheel_coords['frontLeft'][0], wheel_coords['frontLeft'][1],
                         wheel_coords['frontRight'][0], wheel_coords['frontRight'][1],
                         wheel_coords['rearLeft'][0], wheel_coords['rearLeft'][1],
                         wheel_coords['rearRight'][0], wheel_coords['rearRight'][1],
                         cam_lat, cam_lon,
                         frame_coords['topRight'][0], frame_coords['topRight'][1],
                         frame_coords['topLeft'][0], frame_coords['topLeft'][1],
                         frame_coords['bottomRight'][0], frame_coords['bottomRight'][1],
                         frame_coords['bottomLeft'][0], frame_coords['bottomLeft'][1]]) 
        print()
        writer.writerow([t, lat, lon, pointer_lat, pointer_lon,
                         ouster_lat, ouster_lon,
                         wheel_coords['frontLeft'][0], wheel_coords['frontLeft'][1],
                         wheel_coords['frontRight'][0], wheel_coords['frontRight'][1],
                         wheel_coords['rearLeft'][0], wheel_coords['rearLeft'][1],
                         wheel_coords['rearRight'][0], wheel_coords['rearRight'][1],
                         cam_lat, cam_lon,
                         frame_coords['topRight'][0], frame_coords['topRight'][1],
                         frame_coords['topLeft'][0], frame_coords['topLeft'][1],
                         frame_coords['bottomRight'][0], frame_coords['bottomRight'][1],
                         frame_coords['bottomLeft'][0], frame_coords['bottomLeft'][1]])