#!/usr/bin/env python3
import rospy
import csv
import numpy as np
import subprocess
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import struct
import os 

bag_dir = "/media/kristen/easystore2/RestorebotData/May2022_final/1conmod"
# File Paths
CSV_FILE = os.path.join(bag_dir, "liosam_odometry.csv")
GLOBAL_MAP_FILE = os.path.join(bag_dir, "global_map.bin")
LOCAL_MAP_FILE = os.path.join(bag_dir, "local_map.bin")
BAG_FILE = os.path.join(bag_dir, "1conmod_May2022.bag")

# Topics to Play
BAG_TOPICS = ["/H03/horiz/os_cloud_node/points", "/H03/imu/data","/nmea_sentence"]

# Storage for odometry data
odometry_data = []

# Callback function to record odometry
def odom_callback(msg):
    timestamp = msg.header.stamp.to_sec()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    odometry_data.append([timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])

# Function to write odometry to CSV
def write_csv():
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(odometry_data)
    rospy.loginfo(f"Odometry saved to {CSV_FILE}")

# Function to extract and save a point cloud to bin
def save_pointcloud_bin(msg, filename):
    points = []
    for i in range(0, len(msg.data), msg.point_step):
        x, y, z = struct.unpack_from("fff", msg.data, i)
        points.append((x, y, z))

    np_points = np.array(points, dtype=np.float32)
    np_points.tofile(filename)
    rospy.loginfo(f"Saved {len(points)} points to {filename}")

# Main function
def main():
    rospy.init_node("liosam_recorder", anonymous=True)
    
    # Subscribe to odometry
    rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, odom_callback)
    
    rospy.loginfo("Starting LIOSAM and playing selected topics from the bag...")

    # Start LIOSAM
    liosam_proc = subprocess.Popen(["roslaunch", "lio_sam", "run_restorebot_May.launch"])
    time.sleep(5)  # Allow LIOSAM to start

    # Play only selected topics from the bag
    rosbag_proc = subprocess.Popen(["rosbag", "play", BAG_FILE, "--topics"] + BAG_TOPICS)

    rospy.loginfo("Recording LIOSAM odometry...")

    # Instead of rospy.spin(), monitor the bag playback
    while not rospy.is_shutdown():
        time.sleep(1)  # Sleep to reduce CPU usage
        if rosbag_proc.poll() is not None:  # If rosbag has finished
            break

    rospy.loginfo("Rosbag finished. Stopping recording...")

    # Save odometry data
    write_csv()

    # Save final maps
    rospy.loginfo("Saving maps...")

    try:
        global_map = rospy.wait_for_message("/lio_sam/mapping/map_global", PointCloud2, timeout=10)
        save_pointcloud_bin(global_map, GLOBAL_MAP_FILE)
    except rospy.ROSException:
        rospy.logwarn("Timeout waiting for /lio_sam/mapping/map_global!")

    try:
        local_map = rospy.wait_for_message("/lio_sam/mapping/map_local", PointCloud2, timeout=20)
        save_pointcloud_bin(local_map, LOCAL_MAP_FILE)
    except rospy.ROSException:
        rospy.logwarn("Timeout waiting for /lio_sam/mapping/map_local!")

    rospy.loginfo("Finished recording LIOSAM data.")

    # Shutdown LIOSAM
    liosam_proc.terminate()

if __name__ == "__main__":
    main()
