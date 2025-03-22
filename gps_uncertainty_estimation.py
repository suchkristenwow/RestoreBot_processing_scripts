#!/usr/bin/env python

import rospy
import pandas as pd
import numpy as np
from nmea_msgs.msg import Sentence
import os 

class GpsCovarianceCalculator:
    def __init__(self, csv_file):
        rospy.init_node('gps_covariance_calculator', anonymous=True)
        self.gsv_snrs = []
        self.hdop = 1.0  # Default value for HDOP
        self.csv_data = pd.read_csv(csv_file)  # Load CSV dataf
        self.distance = 1.0  # Distance to GPS measurement (e.g., lever arm length in meters)
        
        self.compass_annotations = np.genfromtxt("/home/kristen/Documents/2control_compass_annotations.csv",delimiter="",skip_header=True) 

        self.yawHeading_data = np.genfromtxt("May_results/2control/yawHeading_data.csv",delimiter=",",skip_header=True)  


        # Subscribe to NMEA Sentence messages
        rospy.Subscriber('/nmea_sentence', Sentence, self.nmea_callback)
        rospy.spin()


    def nmea_callback(self, msg):
        # Use bag time (msg.header.stamp) instead of current ROS time
        timestamp = msg.header.stamp.to_sec()
        #print("received nmea sentence!")
        # Check if the sentence is GSV, GGA, or GSA
        sentence = msg.sentence
        if sentence.startswith('$GPGSV'):
            self.parse_gsv(sentence)
        elif sentence.startswith('$GPGGA') or sentence.startswith('$GPGSA'):
            self.parse_hdop(sentence)
        
        # Compute covariance matrix if we have valid data
        if self.gsv_snrs:
            self.compute_covariance_matrix(msg.header.stamp)
        else:
            print("gsv is empty ...") 

    def parse_gsv(self, sentence):
        """
        Parse GSV message to extract SNR values.
        """
        print("parsing gsv...")
        try:
            parts = sentence.split(',')
            for i in range(7, len(parts), 4):  # SNR values are at indices 7, 11, 15, etc.
                if parts[i].isdigit():
                    self.gsv_snrs.append(int(parts[i]))
                else:
                    print("parts: ",parts)
        except Exception as e:
            rospy.logwarn(f"Error parsing GSV message: {e}")

    def parse_hdop(self, sentence):
        """
        Parse GGA or GSA message to extract HDOP.
        """
        print("parsing hdop")
        try:
            parts = sentence.split(',')
            if sentence.startswith('$GPGGA'):
                self.hdop = float(parts[8]) if parts[8] else self.hdop
            elif sentence.startswith('$GPGSA'):
                self.hdop = float(parts[16]) if parts[16] else self.hdop
        except Exception as e:
            rospy.logwarn(f"Error parsing GGA/GSA message: {e}")

    def get_compass_data(self, timestamp, tolerance=0.1):
        """
        Fetch the compass heading and uncertainty from the CSV based on the timestamp.
        """
        # Find rows within the timestamp tolerance
        close_rows = self.csv_data[np.abs(self.csv_data['timestamp'] - timestamp) <= tolerance]
        
        if not close_rows.empty:
            # Use the closest row within tolerance
            closest_row = close_rows.iloc[0]
            heading = np.radians(closest_row['updated_heading'])  # Convert degrees to radians
            sigma_theta = closest_row['uncertainty']  # Uncertainty in heading
            return heading, sigma_theta

        # If no close rows are found, interpolate between the closest two timestamps
        before = self.csv_data[self.csv_data['timestamp'] < timestamp].tail(1)
        after = self.csv_data[self.csv_data['timestamp'] > timestamp].head(1)

        if not before.empty and not after.empty:
            # Perform linear interpolation
            t1, h1, u1 = before.iloc[0]['timestamp'], before.iloc[0]['updated_heading'], before.iloc[0]['uncertainty']
            t2, h2, u2 = after.iloc[0]['timestamp'], after.iloc[0]['updated_heading'], after.iloc[0]['uncertainty']
            
            # Interpolating heading and uncertainty
            heading = np.radians(h1 + (h2 - h1) * (timestamp - t1) / (t2 - t1))  # Linear interpolation
            sigma_theta = u1 + (u2 - u1) * (timestamp - t1) / (t2 - t1)  # Linear interpolation
            return heading, sigma_theta

        # If no data is available for interpolation
        rospy.logwarn(f"No compass data found for timestamp {timestamp}, even for interpolation.")
        return None, None

    def compute_covariance_matrix(self, timestamp):
        """
        Compute the covariance matrix combining HDOP and heading uncertainty.
        """
        if not self.gsv_snrs:
            rospy.logwarn("No SNR values available for covariance computation.")
            return
        
        # Fetch compass data for the current bag timestamp
        #heading, sigma_theta = self.get_compass_data(timestamp.to_sec())
        heading, _ = self.get_compass_data(timestamp.to_sec())

        #sigma_theta = np.deg2rad(45)  
        timestamp_in_range = False 

        if len(self.compass_annotations.shape) == 1:
            timestamp_in_range = True 
        else: 
            if np.any((timestamp.to_sec()>= self.compass_annotations[:, 0]) & (timestamp.to_sec()<= self.compass_annotations[:, 1])): 
                timestamp_in_range = True  

        if timestamp_in_range:
            if len(self.compass_annotations.shape) > 1:
                matches = np.where((timestamp.to_sec() >= self.compass_annotations[:, 0]) & (timestamp.to_sec() <= self.compass_annotations[:, 1]))[0] 
                annotated_idx = matches[0] #the index of the row containing this timestamp 
                annotated_compass_min = self.compass_annotations[annotated_idx,2]; annotated_compass_max = self.compass_annotations[annotated_idx,3] 
            else:
                annotated_compass_min = self.compass_annotations[2]; annotated_compass_max = self.compass_annotations[3] 
                
            tstep_idx = np.argmin(np.abs(self.yawHeading_data[:,0] - timestamp.to_sec()))  
            delta_t = self.yawHeading_data[tstep_idx,0] - timestamp.to_sec() 
            #print("yawHeading tstep:{}, timestamp: {}".format(self.yawHeading_data[tstep_idx,0],timestamp.to_sec())) 
            #print("delta_t: {}".format(delta_t))
            if abs(delta_t) > 0.3:
                if abs(delta_t) < 1:
                    sigma_theta = np.deg2rad(120)  
                else:
                    raise OSError 
            else: 
                true_course = self.yawHeading_data[tstep_idx,3]
                #print("true_course:",true_course) 
                if true_course <= annotated_compass_max and annotated_compass_min <= true_course: 
                    sigma_theta = np.deg2rad(5)
                else:
                    sigma_theta = np.deg2rad(45)
        else: 
            #if timestamp is not annotated uncertainty is 100 
            sigma_theta = np.deg2rad(100) 
        
        print("sigma_theta: ",sigma_theta) 

        if heading is None or sigma_theta is None:
            print("WARNING ... returning prematurely heading: {}, sigma_theta: {}".format(heading,sigma_theta))
            return
        
        # Compute sigma_SNR as the inverse of the average SNR
        avg_snr = np.mean(self.gsv_snrs)
        print("avg_snr: ",avg_snr)
        sigma_snr = 1.0 / avg_snr if avg_snr > 0 else 1.0
        sigma_pos = np.sqrt(sigma_snr**2 + 0.02)

        # Compute Jacobian of the heading uncertainty
        J_theta = np.array([
            [-self.distance * np.sin(heading), self.distance * np.cos(heading)]
        ])
        
        # Heading uncertainty contribution
        heading_covariance = J_theta.T @ np.array([[sigma_theta**2]]) @ J_theta

        # GPS position uncertainty (HDOP)
        gps_covariance = sigma_pos**2 * np.array([
            [self.hdop**2, 0],
            [0, self.hdop**2]
        ])

        # Combined covariance matrix
        covariance_matrix = gps_covariance + heading_covariance
        
        rospy.loginfo(f"Covariance Matrix:\n{covariance_matrix}")

        if not os.path.exists("May_results/2control/covariance_matrices"):
            os.mkdir("May_results/2control/covariance_matrices") 
        
        print("writing {} ...".format(os.path.join("May_results/2control/covariance_matrices",str(timestamp.to_nsec()) + ".csv")))
        np.savetxt(os.path.join("May_results/2control/covariance_matrices",str(timestamp.to_nsec()) + ".csv"),covariance_matrix) 



if __name__ == '__main__':
    try:
        csv_file = "May_results/2control/processed_compass_heading.csv"
        GpsCovarianceCalculator(csv_file)
    except rospy.ROSInterruptException:
        pass
