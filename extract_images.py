import rosbag
import cv2
import os
from cv_bridge import CvBridge

def extract_images_from_rosbag(bag_file, output_dir, image_topic):
    """
    Extracts images from a ROS bag and saves them to a specified directory with filenames based on timestamps.
    
    Parameters:
        bag_file (str): Path to the ROS bag file.
        output_dir (str): Path to the directory where images will be saved.
        image_topic (str): Topic name where images are published in the ROS bag.
    """
    # Create the output directory if it does not exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Initialize CvBridge for ROS image message to OpenCV conversion
    bridge = CvBridge()
    
    # Open the ROS bag
    print(f"Opening ROS bag: {bag_file}")
    bag = rosbag.Bag(bag_file, 'r')
    
    # Iterate through the messages in the specified topic
    try:
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            # Convert the ROS image message to an OpenCV image
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                print(f"Error converting image at {t}: {e}")
                continue
            
            # Use the timestamp in nanoseconds as the filename
            timestamp_ns = t.to_nsec()
            image_filename = os.path.join(output_dir, f"{timestamp_ns}.jpg")
            
            # Save the image to the output directory
            cv2.imwrite(image_filename, cv_image)
            print(f"Saved image: {image_filename}")
    finally:
        bag.close()
    
    print(f"Images extracted and saved to {output_dir}")

if __name__ == "__main__":
    # Path to your ROS bag file
    bag_file = "/media/kristen/easystore2/RestorebotData/May2022_backups/2control/_2022-04-24-12-15-49_0.bag"
    
    # Output directory for images
    output_dir = "/media/kristen/easystore2/RestorebotData/May2022_backups/2control/front_cam_imgs"
    
    # Topic containing the image messages
    image_topic = "/front_camera/color/image_raw"  # Update this with the correct topic from your bag
    
    # Extract images
    extract_images_from_rosbag(bag_file, output_dir, image_topic)
