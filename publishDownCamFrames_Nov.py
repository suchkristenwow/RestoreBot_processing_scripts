#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PolygonStamped, Point32
from scipy.spatial.transform import Rotation as R
import image_geometry
from lio_sam.msg import yaw_radius, yaw_radius_array

class CameraFOVPublisher:
    def __init__(self, depth):
        rospy.init_node("camera_fov_polygon_publisher")
        self.depth = depth

        self.cam_model = image_geometry.PinholeCameraModel()

        # Set the camera intrinsics directly
        K = [617.08935546875, 0.0, 325.5894775390625,
             0.0, 617.3055419921875, 241.2794189453125,
             0.0, 0.0, 1.0]
        D = [0.0, 0.0, 0.0, 0.0, 0.0]
        R = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
        P = [617.08935546875, 0.0, 325.5894775390625, 0.0,
             0.0, 617.3055419921875, 241.2794189453125, 0.0,
             0.0, 0.0, 1.0, 0.0]

        # Initialize the camera model
        from sensor_msgs.msg import CameraInfo
        camera_info = CameraInfo()
        camera_info.height = 480
        camera_info.width = 640
        camera_info.distortion_model = "plumb_bob"
        camera_info.D = D
        camera_info.K = K
        camera_info.R = R
        camera_info.P = P
        camera_info.binning_x = 0
        camera_info.binning_y = 0
        self.cam_model.fromCameraInfo(camera_info)

        # TF listener
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(30))

        # Publisher for the PolygonStamped message
        self.left_fov_pub = rospy.Publisher("left_camera_fov_polygon", PolygonStamped, queue_size=10)
        self.right_fov_pub = rospy.Publisher("right_camera_fov_polygon", PolygonStamped, queue_size=10)

        # Variables
        #self.camera_pose = None
        self.left_camera_pose = None 
        self.right_camera_pose = None 
        self.pose_timestamp = None 
        self.msg_stamp = None 
    
    def lookup_camera_transform(self,camera_side):
        print("looking up camera transform for this side: " + camera_side)
        try:
            # Look up the most recent transform
            if camera_side == "left": 
                (trans, rot) = self.tf_listener.lookupTransform("map", "down_camera0_color_optical_frame", rospy.Time(0)) 
                self.left_camera_pose = list(trans) + list(rot)
            else:
                (trans, rot) = self.tf_listener.lookupTransform("map", "down_camera1_color_optical_frame", rospy.Time(0)) 
                self.right_camera_pose = list(trans) + list(rot)
            # Update timestamp to match the TF data
            self.pose_timestamp = rospy.Time(0)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to lookup transform: {e}")
            return False

    @staticmethod
    def pose_to_transformation_matrix(pose):
        tf_matrix = np.eye(4)
        rotation = R.from_quat(pose[3:])
        tf_matrix[:3, :3] = rotation.as_matrix()
        tf_matrix[:3, 3] = pose[:3]
        return tf_matrix

    def project_pixel(self, pixel):
        ray = np.asarray(self.cam_model.projectPixelTo3dRay(pixel))
        return ray * self.depth

    def convert_optical_to_nav(self, cam_point):
        return np.array([cam_point[2], -cam_point[0], -cam_point[1], 1.0])

    def apply_cam_transformation(self, point, side):
        if side == "left": 
            cam_tf = self.pose_to_transformation_matrix(self.left_camera_pose) 
        else: 
            cam_tf = self.pose_to_transformation_matrix(self.right_camera_pose)  
        return np.dot(cam_tf, point)[:3]

    def get_fov_polygon(self,side):
        if self.left_camera_pose is None:
            self.lookup_camera_transform("left")
            if self.left_camera_pose is None: 
                rospy.logwarn("Left Camera pose is not available.")
                return None

        if self.right_camera_pose is None:
            self.lookup_camera_transform("right") 
            if self.right_camera_pose is None:
                rospy.logwarn("Right Camera pose is not available.")
                return None
        
        # Define the corners of the image
        width = self.cam_model.width
        height = self.cam_model.height
        corners = [(0, 0), (width, 0), (width, height), (0, height)]
        points = []

        for corner in corners:
            cam_point = self.project_pixel(corner)
            cam_point_frame = self.convert_optical_to_nav(cam_point)
            world_point = self.apply_cam_transformation(cam_point_frame,side)
            points.append(world_point)

        # Create the PolygonStamped message
        polygon = PolygonStamped()
        polygon.header.frame_id = "map"
        polygon.header.stamp = self.pose_timestamp

        for point in points:
            polygon_point = Point32(x=point[0], y=point[1], z=point[2])
            polygon.polygon.points.append(polygon_point)

        return polygon, points 

   
    def publish_fov(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.lookup_camera_transform("left"):
                rospy.sleep(0.1)
                continue
            
            if self.left_camera_pose is None:
                rospy.logwarn("Left Camera pose is not available.")
                rospy.sleep(0.1)
                continue
            
            left_fov_polygon, _ = self.get_fov_polygon("left") 

            if left_fov_polygon:
                rospy.loginfo("Publishing FOV polygon.")
                self.left_fov_pub.publish(left_fov_polygon)  

            if not self.lookup_camera_transform("right"):
                rospy.sleep(0.1)
                continue

            if self.right_camera_pose is None:
                rospy.logwarn("Right Camera pose is not available.")
                rospy.sleep(0.1)
                continue

            right_fov_polygon, _ = self.get_fov_polygon("right") 

            if right_fov_polygon:
                rospy.loginfo("Publishing FOV polygon.")
                self.right_fov_pub.publish(right_fov_polygon)   

            rate.sleep()

if __name__ == "__main__":
    depth = -0.135  # Distance from camera to ground
    fov_publisher = CameraFOVPublisher(depth)
    fov_publisher.publish_fov()