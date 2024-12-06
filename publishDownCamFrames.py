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
        self.fov_pub = rospy.Publisher("camera_fov_polygon", PolygonStamped, queue_size=10)

        # Variables
        self.camera_pose = None
        self.pose_timestamp = None 
        self.msg_stamp = None 
    
    def lookup_camera_transform(self):
        try:
            # Look up the most recent transform
            (trans, rot) = self.tf_listener.lookupTransform("map", "down_camera_color_optical_frame", rospy.Time(0))
            self.camera_pose = list(trans) + list(rot)
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

    def apply_cam_transformation(self, point):
        cam_tf = self.pose_to_transformation_matrix(self.camera_pose)
        return np.dot(cam_tf, point)[:3]

    def get_fov_polygon(self):
        if self.camera_pose is None:
            rospy.logwarn("Camera pose is not available.")
            return None

        # Define the corners of the image
        width = self.cam_model.width
        height = self.cam_model.height
        corners = [(0, 0), (width, 0), (width, height), (0, height)]
        points = []

        for corner in corners:
            cam_point = self.project_pixel(corner)
            cam_point_frame = self.convert_optical_to_nav(cam_point)
            world_point = self.apply_cam_transformation(cam_point_frame)
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
            if not self.lookup_camera_transform():
                rospy.sleep(0.1)
                continue

            fov_polygon, fov_points = self.get_fov_polygon() 

            if fov_polygon:
                rospy.loginfo("Publishing FOV polygon.")
                self.fov_pub.publish(fov_polygon)
            rate.sleep()

if __name__ == "__main__":
    depth = 0.135  # Distance from camera to ground
    fov_publisher = CameraFOVPublisher(depth)
    fov_publisher.publish_fov()
