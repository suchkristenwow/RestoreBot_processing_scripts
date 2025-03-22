import os
import argparse
import rosbag
import cv2
import numpy as np
import pandas as pd
import pcl
from tqdm import tqdm
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from nmea_msgs.msg import Sentence

# Topics of interest
TOPICS = {
    "nmea": "/nmea_sentence",
    "imu": "/H03/imu/data",
    "pointcloud": "/H03/horiz/os_cloud_node/points",
    "down_image": "/down_camera/color/image_raw",
    "front_image": "/front_camera/color/image_raw"
}

# Initialize CV Bridge
bridge = CvBridge()

# Function to convert ROS timestamp to nanoseconds
def to_nano(timestamp):
    return int(timestamp.to_nsec())

# Function to create required folders
def create_folders(base_path):
    folders = {
        "pointcloud": os.path.join(base_path, "PointClouds"),
        "images/front": os.path.join(base_path, "Front_Facing_Images"),
        "images/down": os.path.join(base_path, "Down_Facing_Images")
    }
    
    for path in folders.values():
        os.makedirs(path, exist_ok=True)
    
    return folders

# Function to extract IMU data
def parse_imu(msg, timestamp):
    return [
        to_nano(timestamp),
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
    ]

# Function to extract NMEA sentences
def parse_nmea(msg, timestamp):
    return [to_nano(timestamp), msg.sentence]

# Function to save point clouds
def save_pointcloud(msg, timestamp, folder):
    cloud = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    cloud_array = np.array(cloud, dtype=np.float32)
    filename = os.path.join(folder, f"{to_nano(timestamp)}.bin")
    cloud_array.tofile(filename)

# Function to save images
def save_image(msg, timestamp, folder):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    filename = os.path.join(folder, f"{to_nano(timestamp)}.png")
    cv2.imwrite(filename, cv_image)

# Parse a single ROS bag
def parse_rosbag(bag_file):
    bag_dir = os.path.dirname(bag_file)  # Save data in the same directory as the bag
    bag_name = os.path.splitext(os.path.basename(bag_file))[0]
    session_path = bag_dir

    print(f"Processing {bag_file} -> {session_path}")
    folders = create_folders(session_path)

    imu_data, nmea_data = [], []

    bag = rosbag.Bag(bag_file)
    
    for topic, msg, timestamp in tqdm(bag.read_messages(topics=TOPICS.values()), desc=f"Extracting {bag_name}"):
        if topic == TOPICS["nmea"]:
            nmea_data.append(parse_nmea(msg, timestamp))
        elif topic == TOPICS["imu"]:
            imu_data.append(parse_imu(msg, timestamp))
        elif topic == TOPICS["pointcloud"]:
            save_pointcloud(msg, timestamp, folders["pointcloud"])
        elif topic == TOPICS["down_image"]:
            save_image(msg, timestamp, folders["images/down"])
        elif topic == TOPICS["front_image"]:
            save_image(msg, timestamp, folders["images/front"])

    bag.close()

    # Save CSV files
    if imu_data:
        imu_df = pd.DataFrame(imu_data, columns=[
            "timestamp_ns", "qx", "qy", "qz", "qw",
            "wx", "wy", "wz", "ax", "ay", "az"
        ])
        imu_df.to_csv(os.path.join(session_path, "imu.csv"), index=False)

    if nmea_data:
        nmea_df = pd.DataFrame(nmea_data, columns=["timestamp_ns", "nmea_sentence"])
        nmea_df.to_csv(os.path.join(session_path, "nmea_sentences.csv"), index=False)

    print(f"✅ Extraction complete for {bag_name}!")

# Function to process multiple bags
def process_bags(bag_paths):
    for bag_file in bag_paths:
        if os.path.exists(bag_file):
            parse_rosbag(bag_file)
        else:
            print(f"❌ Skipping {bag_file}: File not found.")

# Command-line arguments
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse multiple ROS bags into Restorebot dataset format.")
    parser.add_argument("--bags", nargs="+", required=True, help="List of bag files or a text file containing paths")

    args = parser.parse_args()

    # If a text file is provided, read the list of bags
    if len(args.bags) == 1 and args.bags[0].endswith(".txt"):
        with open(args.bags[0], "r") as f:
            bag_list = [line.strip() for line in f.readlines()]
    else:
        bag_list = args.bags

    process_bags(bag_list)
    print("✅ All bags processed successfully!")