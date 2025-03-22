#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PolygonStamped, Point32
from collections import deque
import time
from std_msgs.msg import Bool 
import os 
import csv  
from lio_sam.msg import yaw_radius, yaw_radius_array 

def get_cardinal_direction(heading_degrees):
    """
    Determine the cardinal direction based on the heading in degrees.
    """
    if (heading_degrees >= 345 or heading_degrees < 15):
        return "N"
    elif (heading_degrees >= 15 and heading_degrees < 75):
        return "NE"
    elif (heading_degrees >= 75 and heading_degrees < 105):
        return "E"
    elif (heading_degrees >= 105 and heading_degrees < 165):
        return "SE"
    elif (heading_degrees >= 165 and heading_degrees < 195):
        return "S"
    elif (heading_degrees >= 195 and heading_degrees < 255):
        return "SW"
    elif (heading_degrees >= 255 and heading_degrees < 285):
        return "W"
    elif (heading_degrees >= 285 and heading_degrees < 345):
        return "NW"

class GPSProjector:
    def __init__(self):
        rospy.init_node("gps_projector_node")
        # Subscribers
        rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, self.odom_callback)
        rospy.Subscriber("/nmea_sentence", Sentence, self.nmea_callback)
        rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback) 
        #rospy.Subscriber("/right_camera_fov_polygon",PolygonStamped, self.right_fov_callback) 
        #rospy.Subscriber("/left_camera_fov_polygon",PolygonStamped, self.left_fov_callback)

        self.yaw = None
        self.angularVel = None
        self.yaw_window = deque()  # Deque to store (timestamp, yaw) pairs

        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(10))

        self.filtered_lat = None 
        self.filtered_lon = None 

        self.position_covar = []

        if not os.path.exists("May_results/2control/yawHeading_data.csv"):
            with open("May_results/2control/yawHeading_data.csv", mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["timestamp", "yaw", "angular_vel (yaw)", "true_course", 
                                 "lat","lon", 
                                 "filtered_lat", "filtered_lon", 
                                 "position_covar0", "position_covar1", "position_covar2", 
                                 "position_covar3", "position_covar4", "position_covar5",
                                   "position_covar6", "position_covar7","position_covar8"]) 

        
        self.saved_static_transforms = False 

    def euler_from_quaternion(self, q):
        """
        Helper function to convert quaternion to Euler angles (yaw).
        """
        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]  # Return yaw angle

    def left_fov_callback(self,msg): 
        corner_projections = msg.polygon.points #these projections are in the map frame 
        trans, rot = self.tf_listener.lookupTransform("map","H03/horiz_ouster_sensor",rospy.Time(0))
        rotation_matrix= tf.transformations.quaternion_matrix(rot)[:3,:3] 
        translation_vector = np.array(trans) 

        self.left_cam_fov_yaw_radius = [] 

        transformed_corners = []
        #apply transform to get camera fov corners relative to the GPS receiver 
        for i,corner in enumerate(corner_projections): 
            # Convert the point to a numpy array
            point_map_frame = np.array([corner.x, corner.y, corner.z])

            # Apply the rotation and translation
            point_navsat_frame = np.dot(rotation_matrix, point_map_frame) + translation_vector

            corner_x = point_navsat_frame[0]; corner_y = point_navsat_frame[1] 
            corner_radius = np.linalg.norm([corner_x,corner_y])
            corner_yaw = np.arctan2(corner_y,corner_x) 
            self.left_cam_fov_yaw_radius.append((corner_radius,corner_yaw)) 

            # Store the transformed point
            transformed_corners.append(point_navsat_frame)
        #print("transformed_corners: ",transformed_corners) 

        center_x = np.mean([x[0] for x in transformed_corners]); center_y = np.mean([x[1] for x in transformed_corners] ) 
        center_radius = np.linalg.norm([center_x,center_y]) 
        center_yaw = np.arctan2(center_y,center_x)      
        self.left_cam_fov_yaw_radius.append((center_radius,center_yaw))

    def right_fov_callback(self,msg): 
        corner_projections = msg.polygon.points #these projections are in the map frame 
        trans, rot = self.tf_listener.lookupTransform("map","H03/horiz_ouster_sensor",rospy.Time(0))
        rotation_matrix= tf.transformations.quaternion_matrix(rot)[:3,:3] 
        translation_vector = np.array(trans) 

        self.right_cam_fov_yaw_radius = [] 

        transformed_corners = []
        #apply transform to get camera fov corners relative to the GPS receiver 
        for i,corner in enumerate(corner_projections): 
            # Convert the point to a numpy array
            point_map_frame = np.array([corner.x, corner.y, corner.z])

            # Apply the rotation and translation
            point_navsat_frame = np.dot(rotation_matrix, point_map_frame) + translation_vector

            corner_x = point_navsat_frame[0]; corner_y = point_navsat_frame[1] 
            corner_radius = np.linalg.norm([corner_x,corner_y])
            corner_yaw = np.arctan2(corner_y,corner_x) 
            self.right_cam_fov_yaw_radius.append((corner_radius,corner_yaw)) 

            # Store the transformed point
            transformed_corners.append(point_navsat_frame)
        #print("transformed_corners: ",transformed_corners) 

        center_x = np.mean([x[0] for x in transformed_corners]); center_y = np.mean([x[1] for x in transformed_corners] ) 
        center_radius = np.linalg.norm([center_x,center_y]) 
        center_yaw = np.arctan2(center_y,center_x)      
        self.right_cam_fov_yaw_radius.append((center_radius,center_yaw))

        #print("self.cam_fov_yaw_radius: ",self.cam_fov_yaw_radius)

    def gps_callback(self,msg): 
        #print("entered gps_callback!")
        self.filtered_lat = msg.latitude
        self.filtered_lon = msg.longitude 
        self.position_covar = msg.position_covariance 

    def get_navsat_transforms(self):
        #rospy.loginfo("getting transforms to ouster and wheels ...")
        if self.saved_static_transforms:
            return 
        
        try:
            (gps2ouster_trans, gps2ouster_rot) = self.tf_listener.lookupTransform("navsat_link", "H03/horiz_ouster_sensor", rospy.Time(0))
            ouster_radius = np.linalg.norm(gps2ouster_trans[:2]) 
            ouster_yaw = np.arctan2(gps2ouster_trans[1],gps2ouster_trans[0])

            (frontLeftWheel_trans, frontLeftWheel_rot) = self.tf_listener.lookupTransform("navsat_link", "H03/front_left_wheel_link", rospy.Time(0))
            frontLeftWheel_radius = np.linalg.norm(frontLeftWheel_trans[:2]) 
            frontLeftWheel_yaw = np.arctan2(frontLeftWheel_trans[1],frontLeftWheel_trans[0]) 

            (frontRightWheel_trans, frontRightWheel_rot) = self.tf_listener.lookupTransform("navsat_link", "H03/front_right_wheel_link", rospy.Time(0))
            frontRightWheel_radius = np.linalg.norm(frontRightWheel_trans[:2]) 
            frontRightWheel_yaw = np.arctan2(frontRightWheel_trans[1],frontRightWheel_trans[0]) 

            (rearLeftWheel_trans, rearLeftWheel_rot) = self.tf_listener.lookupTransform("navsat_link", "H03/rear_left_wheel_link" , rospy.Time(0))
            rearLeftWheel_radius = np.linalg.norm(rearLeftWheel_trans[:2]) 
            rearLeftWheel_yaw = np.arctan2(rearLeftWheel_trans[1],rearLeftWheel_trans[0]) 

            (rearRightWheel_trans, rearRightWheel_rot) = self.tf_listener.lookupTransform("navsat_link", "H03/rear_right_wheel_link", rospy.Time(0))
            rearRightWheel_radius = np.linalg.norm(rearRightWheel_trans[:2]) 
            rearRightWheel_yaw = np.arctan2(rearRightWheel_trans[1],rearRightWheel_trans[0]) 

            with open("May_results/2control/robotTransforms.csv", mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([ouster_radius,ouster_yaw,
                                 frontLeftWheel_radius,frontLeftWheel_yaw,
                                 frontRightWheel_radius,frontRightWheel_yaw,
                                 rearLeftWheel_radius,rearLeftWheel_yaw, 
                                 rearRightWheel_radius,rearRightWheel_yaw])
                
            self.saved_static_transforms = True 

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to lookup navsat transform: {e}")
            return None
        
    def odom_callback(self, msg):
        msg_tstamp = msg.header.stamp.to_sec()
        print("odom callback ... this is timestamp: ",msg_tstamp)
        orientation = msg.pose.pose.orientation
        yaw = self.euler_from_quaternion(orientation)

        self.yaw = yaw 

        # Update the sliding window
        self.yaw_window.append((msg_tstamp, yaw))

        # Remove outdated entries (older than 1 second)
        while self.yaw_window and (msg_tstamp - self.yaw_window[0][0] > 1.0):
            self.yaw_window.popleft()

        # Compute angular velocity if we have at least two points
        if len(self.yaw_window) >= 2:
            t_start, yaw_start = self.yaw_window[0]
            t_end, yaw_end = self.yaw_window[-1]
            time_diff = t_end - t_start

            if time_diff > 0:
                # Compute angular velocity (change in yaw / time difference)
                # Normalize yaw difference to handle wrap-around between -pi and pi
                yaw_diff = (yaw_end - yaw_start + np.pi) % (2 * np.pi) - np.pi
                self.angularVel = yaw_diff / time_diff

        #print(f"yaw: {self.yaw}, angular_vel: {self.angularVel}")  
        self.get_navsat_transforms() 

    def nmea_callback(self, msg):
        """
        Callback function to process NMEA sentence messages and publish compass heading.
        """
        msg_tstamp = msg.header.stamp.to_sec()
        print("nmea callback ... this is msg_tstamp: ",msg_tstamp)

        # Check if the sentence is a GNRMC sentence
        if msg.sentence.startswith("$GNRMC"):
            try:
                parts = msg.sentence.split(',')
                heading = float(parts[8]) if parts[8] else None
                heading += float(parts[10])
                lat = 10**(-2)*float(parts[3]); lon = -10**(-2)*float(parts[5]) 
                if heading is not None and heading != 0:
                    cardinal_direction = get_cardinal_direction(heading)
                    print(f"measured heading: {heading}, cardinal direction: {cardinal_direction}")
                    # Write to CSV
                    with open("May_results/2control/yawHeading_data.csv", mode='a', newline='') as file:
                        writer = csv.writer(file)
                        '''
                        print("writing row: ")
                        print([msg_tstamp, self.yaw, self.angularVel, heading, 
                                         lat, lon, 
                                         self.filtered_lat, self.filtered_lon, 
                                         self.position_covar[0], self.position_covar[1], self.position_covar[2],
                                         self.position_covar[3], self.position_covar[4], self.position_covar[5],
                                         self.position_covar[6], self.position_covar[7], self.position_covar[8]])
                        '''
                        writer.writerow([msg_tstamp, self.yaw, self.angularVel, heading, 
                                         lat, lon, 
                                         self.filtered_lat, self.filtered_lon, 
                                         self.position_covar[0], self.position_covar[1], self.position_covar[2],
                                         self.position_covar[3], self.position_covar[4], self.position_covar[5],
                                         self.position_covar[6], self.position_covar[7], self.position_covar[8]])
                        
            except (IndexError, ValueError) as e: 
                rospy.logwarn(f"Failed to parse GNRMC sentence: {msg.sentence}, Error: {e}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    projector = GPSProjector()
    projector.run()
