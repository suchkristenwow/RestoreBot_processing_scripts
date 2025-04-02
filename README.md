# Restorebot Processing & Analysis Tools 
These tools are designed to work with the Restorebot Dataset. 

## Dataset Structure & Download Instructions 
To make our data more easily sharible, we provide a script that takes in the extracted data and turns it back into ROS bags. 

### Processing Tools 
For repeatibility, we provide the tools to used extract the data products. To get the GPS coordinates of the frustrum corners of 
our down-facing cameras, we use the following procedure.

1. Record timestamps, yaw in the robot frame (LIOSAM), and measured heading from NMEA sentences. 
  - Run LIOSAM from the corresponding season
  - Play the point clouds, IMU data, front-facing camera images, and nmea sentences back from the bag
  - Run get_frame_gps_coords.py, publishDownCamFrames(_Nov).py, publish_gps_fix.py and publish_IMU_correct.py
      - Call these scripts BEFORE playing back the bag 
  This step generates yawHeading_data.csv. While this is running, we annotate the approximate compass heading.
2. Run automate_annotations.py
3. Run interpolate_compassheading.py
4. Run getGPSCoords.py
  - The resulting csv contains the gps coordinates for the robot, sensors, and down-facing cameras
  - The result can be visualized using plot_results_animation.py which shows a top-down view (in GPS coordinate space)
    of the down-facing camera FOV, and the robot chassis alongside the front-facing camera image. 

To estimate uncertainty;

1. Run gps_uncertainty_estimation.py and publish_gps_fix.py
2. Play back the NMEA sentences from the desired bag

This creates a directory containing csv files for each nmea sentence received. The csv contains the 2x2 covariance matrix as per 
the method in our paper. 

### Cross-Seasonal Matching Tools 
These tools are to help find overlapping GPS frames between seasons. 

Use get_overlapping_frames.py to find the corresponding frames between seasons which overlap. 

You can stitch these together using stitch_imgs.py to have a side-by-side comparison 

# Reference 
