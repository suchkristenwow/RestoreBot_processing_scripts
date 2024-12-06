#!/usr/bin/env python

import rospy
import re
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix, NavSatStatus

# Regular expressions for NMEA sentences
GGA_PATTERN = re.compile(r"\$GNGGA,\d+.\d+,(?P<lat>\d+\.\d+),(?P<lat_dir>[NS]),(?P<long>\d+\.\d+),(?P<long_dir>[EW]),\d+,\d+,\d+\.\d+,(?P<alt>\d+\.\d+),M")
RMC_PATTERN = re.compile(r"\$GNRMC,\d+.\d+,[AV],(?P<lat>\d+\.\d+),(?P<lat_dir>[NS]),(?P<long>\d+\.\d+),(?P<long_dir>[EW]),.*,(?P<date>\d+)")

def parse_lat_long(lat, lat_dir, long, long_dir):
    """ Convert latitude and longitude from NMEA format to decimal degrees. """
    # Latitude conversion
    lat_degrees = float(lat[:2])
    lat_minutes = float(lat[2:])
    latitude = lat_degrees + (lat_minutes / 60.0)
    if lat_dir == 'S':
        latitude = -latitude

    # Longitude conversion
    long_degrees = float(long[:3])
    long_minutes = float(long[3:])
    longitude = long_degrees + (long_minutes / 60.0)
    if long_dir == 'W':
        longitude = -longitude

    return latitude, longitude

def parse_gga(sentence):
    """ Parse GGA sentence for latitude, longitude, and altitude. """
    match = GGA_PATTERN.match(sentence)
    if not match:
        return None

    lat = match.group('lat')
    lat_dir = match.group('lat_dir')
    long = match.group('long')
    long_dir = match.group('long_dir')
    altitude = float(match.group('alt'))

    latitude, longitude = parse_lat_long(lat, lat_dir, long, long_dir)
    
    return {'latitude': latitude, 'longitude': longitude, 'altitude': altitude}

def parse_rmc(sentence):
    """ Parse RMC sentence for latitude and longitude. """
    match = RMC_PATTERN.match(sentence)
    if not match:
        return None

    lat = match.group('lat')
    lat_dir = match.group('lat_dir')
    long = match.group('long')
    long_dir = match.group('long_dir')

    latitude, longitude = parse_lat_long(lat, lat_dir, long, long_dir)
    
    return {'latitude': latitude, 'longitude': longitude}

def nmea_callback(msg):
    """ Callback function for NMEA sentence messages. """
    nmea_sentence = msg.sentence  # Access the actual NMEA sentence from nmea_msgs/Sentence

    # Attempt to parse GGA and RMC sentences
    parsed_data = None
    if nmea_sentence.startswith('$GNGGA'):
        parsed_data = parse_gga(nmea_sentence)
    elif nmea_sentence.startswith('$GNRMC'):
        parsed_data = parse_rmc(nmea_sentence)

    if not parsed_data:
        return  # Skip if sentence is not parsed or not relevant

    # Create and populate NavSatFix message
    navsatfix_msg = NavSatFix()
    navsatfix_msg.header.stamp = msg.header.stamp 
    navsatfix_msg.header.frame_id = "navsat_link"
    navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
    navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS

    # Set the latitude, longitude, and altitude if available
    navsatfix_msg.latitude = parsed_data.get('latitude', 0.0)
    navsatfix_msg.longitude = parsed_data.get('longitude', 0.0)
    navsatfix_msg.altitude = parsed_data.get('altitude', 0.0)

    # Optional: Set the covariance if required
    navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    # Publish the NavSatFix message
    navsatfix_pub.publish(navsatfix_msg)

if __name__ == '__main__':
    rospy.init_node('nmea_to_navsatfix')

    # Subscribe to NMEA sentences (from nmea_msgs/Sentence)
    rospy.Subscriber('/nmea_sentence', Sentence, nmea_callback)

    # Publisher for NavSatFix messages
    navsatfix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

    rospy.spin()
