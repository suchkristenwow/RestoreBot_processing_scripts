#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import argparse

def save_images_from_bag(bag_file, output_folder, image_topic):
    """
    Extracts images from a ROS bag and saves them as files with the timestamp as filenames.

    :param bag_file: Path to the bag file.
    :param output_folder: Path to the output folder where images will be saved.
    :param image_topic: Topic from which images are to be extracted.
    """
    # Check if output folder exists, create if not
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    print("initting cv bridge...")
    # Initialize CvBridge
    bridge = CvBridge()

    # Open the bag file
    print("opening the bag!")
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            #if isinstance(msg, Image):
            try:
                # Convert ROS Image message to OpenCV format
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                
                # Save the image using the timestamp in nanoseconds as the filename
                timestamp = msg.header.stamp.to_nsec()
                image_filename = os.path.join(output_folder, f"{timestamp}.png")
                cv2.imwrite(image_filename, cv_image)
                rospy.loginfo(f"Saved image: {image_filename}")
            except Exception as e:
                rospy.logwarn(f"Failed to save image: {e}") 
          

if __name__ == "__main__":
    rospy.init_node('save_images_from_bag')

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Extract and save images from a bag file.")
    parser.add_argument("bag_file", help="Path to the input ROS bag file.")
    parser.add_argument("output_folder", help="Path to the output folder where images will be saved.")
    parser.add_argument("image_topic", help="Name of the image topic to extract images from.")

    args = parser.parse_args()

    try:
        save_images_from_bag(args.bag_file, args.output_folder, args.image_topic)
    except rospy.ROSInterruptException:
        pass
