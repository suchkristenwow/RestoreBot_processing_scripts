#!/usr/bin/env python

import rospy
import tf
import message_filters
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tft

class ImuTransformer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('imu_transformer')

        # Parameters
        self.imu_frame = rospy.get_param('~imu_frame', 'H03/imu_link')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Subscriber and Publisher
        self.imu_sub = message_filters.Subscriber('/H03/imu/data', Imu)
        self.imu_pub = rospy.Publisher('/imu_correct', Imu, queue_size=10)

        # Use ApproximateTimeSynchronizer to sync IMU data with the transform
        self.sync = message_filters.ApproximateTimeSynchronizer([self.imu_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.imu_callback)

    def imu_callback(self, imu_msg):
        print("received imu message!")
        try:
            # Lookup the latest transform
            (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, self.imu_frame, imu_msg.header.stamp)

            # Convert IMU orientation to a quaternion
            imu_orientation = imu_msg.orientation
            imu_quat = [imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w]

            # Transform IMU orientation to the base frame
            rot_quat = tft.quaternion_multiply(tft.quaternion_multiply(rot, imu_quat), tft.quaternion_conjugate(rot))

            # Create a new IMU message
            transformed_imu = Imu()
            transformed_imu.header.stamp = imu_msg.header.stamp
            transformed_imu.header.frame_id = self.base_frame
            transformed_imu.orientation = Quaternion(*rot_quat)

            # Copy over the linear acceleration and angular velocity without transformation
            transformed_imu.linear_acceleration = imu_msg.linear_acceleration
            transformed_imu.angular_velocity = imu_msg.angular_velocity
            
            print("publishing transformed imu ...")
            # Publish the transformed IMU message
            self.imu_pub.publish(transformed_imu)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform IMU data: {e}")

if __name__ == '__main__':
    try:
        imu_transformer = ImuTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
