import numpy as np
import os 
import glob 
import shutil 
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon, Ellipse

LAT_METERS_PER_DEGREE = 111320  # meters per degree latitude
LON_METERS_PER_DEGREE = 85390   # meters per degree longitude at this latitude

from geopy.distance import geodesic

def load_covariance_matrix(covariance_dir,available_timestamps,timestamp):
    """Load the covariance matrix for the given timestamp."""
    # Find the closest available timestamp
    closest_timestamp = min(available_timestamps, key=lambda t: abs(t - timestamp)) 
    #tmp = closest_timestamp*10**(-9)
    filename = os.path.join(covariance_dir, f"{closest_timestamp}.csv") 
    #print("filename: ",filename)
    if not os.path.exists(filename):
        raise OSError 
    #print("delta_t: ",abs(timestamp - tmp) )
    # Check if the closest timestamp is within 0.1 seconds
    if abs(timestamp - closest_timestamp) <= 0.1:
        #return pd.read_csv(filename, header=None).values
        #print(np.genfromtxt(filename))
        return np.genfromtxt(filename)
    else:
        # If no timestamp is close enough, return None
        return None

def create_uncertainty_ellipse(cov_matrix, center, ax, **kwargs):
    """Create an uncertainty ellipse scaled for lat/lon plotting."""
    if cov_matrix is None:
        print("Covariance matrix is None; skipping ellipse creation.")
        return None

    try:
        # Convert covariance matrix from meters² to degrees²
        lat_scale = 1 / LAT_METERS_PER_DEGREE
        lon_scale = 1 / LON_METERS_PER_DEGREE
        scale_matrix = np.diag([lat_scale, lon_scale])  # Scale factors for lat/lon
        scaled_cov_matrix = scale_matrix @ cov_matrix @ scale_matrix.T

        # Eigenvalues and eigenvectors for ellipse orientation and scaling
        eigenvalues, eigenvectors = np.linalg.eig(scaled_cov_matrix)
        major_axis = 2 * np.sqrt(eigenvalues[0])  # 2-sigma width
        minor_axis = 2 * np.sqrt(eigenvalues[1])  # 2-sigma height
        angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))  # Orientation

        # Create and return the ellipse
        ellipse = Ellipse(
            xy=center, width=major_axis, height=minor_axis, angle=angle, **kwargs
        )
        return ellipse
    except Exception as e:
        print(f"Error creating ellipse: {e}")
        return None
    
def are_coordinates_within_threshold(coord1, coord2, threshold_meters=0.1):
    """
    Checks if two GPS coordinates are within a certain distance threshold.

    Parameters:
        coord1 (tuple): First GPS coordinate as (latitude, longitude).
        coord2 (tuple): Second GPS coordinate as (latitude, longitude).
        threshold_meters (float): The distance threshold in meters.

    Returns:
        bool: True if the coordinates are within the threshold, False otherwise.
    """
    # Calculate the distance in meters using geodesic (Vincenty formula fallback)
    distance = geodesic(coord1, coord2).meters
    #print("distance: ",distance) 
    return distance <= threshold_meters

def find_closest_frame(frame_dir,timestamp):
    """Find the closest frame to the given timestamp."""
    frame_files = [f for f in os.listdir(frame_dir) if f.endswith(".png") or f.endswith(".jpg")]
    frame_timestamps = [10 ** (-9) * int(f.split(".")[0]) for f in frame_files] 
    closest_timestamp = min(frame_timestamps, key=lambda t: abs(t - timestamp))
    tstep_str = str(int(closest_timestamp * 1e9))
    matching_files = [
        os.path.join(frame_dir, f)
        for f in os.listdir(frame_dir)
        if tstep_str[:-5] in f
    ]

    if len(matching_files) > 0:
        return matching_files[0]
    else: 
        matching_files = [
            os.path.join(frame_dir, f)
            for f in os.listdir(frame_dir)
            if tstep_str[:-6] in f
        ] 
        return matching_files[0] 
    
def precompute_timestamps(covariance_dir):
    """Precompute all available timestamps from the covariance matrix files."""
    print("precomputing timestamps ...")
    filenames = glob.glob(os.path.join(covariance_dir, "*.csv"))
    timestamps = [int(os.path.basename(f).split(".")[0]) for f in filenames]  # Convert nanoseconds to seconds
    return sorted(timestamps)

"""
def load_covariance_matrix(timestamp,available_timestamps):
    print("timestamp: ",timestamp) 
    print("available_timestamps: ",available_timestamps[0]) 

    # Find the closest available timestamp
    closest_timestamp = min(available_timestamps, key=lambda t: abs(t - timestamp)) 
    print("closest_timestamp: ",closest_timestamp)
    '''
    tmp = closest_timestamp*10**(-9) 
    print("tmp:",tmp) 
    '''
    tmp = closest_timestamp
    filename = os.path.join(covariance_dir, f"{closest_timestamp}.csv") 
    print("filename: ",filename)

    if not os.path.exists(filename):
        raise OSError 
    print("delta_t: ",abs(timestamp - tmp) )
    # Check if the closest timestamp is within 0.1 seconds
    if abs(timestamp - tmp) <= 0.1:
        #return pd.read_csv(filename, header=None).values
        #print(np.genfromtxt(filename))
        return np.genfromtxt(filename)
    else:
        # If no timestamp is close enough, return None
        return None
"""

def is_inside_ellipse(cov_matrix, center, point):
    """
    Check if a point is inside the covariance ellipse.
    :param cov_matrix: Covariance matrix (2x2)
    :param center: Center of the ellipse (longitude, latitude)
    :param point: Point to check (longitude, latitude)
    :return: True if inside, False otherwise
    """
    diff = np.array(point) - np.array(center)
    inv_cov_matrix = np.linalg.inv(cov_matrix)
    mahalanobis_dist = np.sqrt(diff.T @ inv_cov_matrix @ diff)
    return mahalanobis_dist <= 2  # Check for 2-sigma ellipse

def find_frames_inside_ellipse(cov_matrix, center, longitudes, latitudes, timestamps):
    """
    Find all frames whose points fall inside the covariance ellipse.
    :param cov_matrix: Covariance matrix (2x2)
    :param center: Center of the ellipse (longitude, latitude)
    :param longitudes: Array of longitude values
    :param latitudes: Array of latitude values
    :param timestamps: Array of timestamps
    :return: List of (timestamp, longitude, latitude) for frames inside the ellipse
    """
    frames_inside = []
    for i in range(len(longitudes)):
        point = (longitudes[i], latitudes[i])
        if is_inside_ellipse(cov_matrix, center, point):
            frames_inside.append((timestamps[i], longitudes[i], latitudes[i]))
    return frames_inside


def find_closest_index(array, target):
    """
    Finds the index of the element in the array closest to the target value.

    Parameters:
        array (list or np.ndarray): The input array (can be unsorted).
        target (int or float): The target value.

    Returns:
        int: The index of the element closest to the target value.
    """
    array = np.array(array)  # Convert to NumPy array for efficient computation
    differences = np.abs(array - target)  # Compute the absolute differences
    closest_index = np.argmin(differences)  # Find the index of the smallest difference
    return closest_index

nov_covariance_dir = "Nov_results/covariance_matrices" 

may_covariance_dir = "May_results/covariance_matrices"

available_timestamps = precompute_timestamps(may_covariance_dir)  # Precompute once at the start 

nov_available_timestamps = precompute_timestamps(nov_covariance_dir)
# Read the GPS data
may_gps_results = np.genfromtxt("May_results/results.csv", delimiter=",", skip_header=1)

# Extract timestamps, latitudes, and longitudes
may_timestamps = may_gps_results[:, 0]
may_latitudes = may_gps_results[:, 1]
may_longitudes = may_gps_results[:, 2]

# Use camera coordinates directly instead of frame center
cam_lat, cam_lon = may_gps_results[:, 15], may_gps_results[:, 16]

# Extract frame corner coordinates
top_right_lat, top_right_lon = may_gps_results[:, 17], may_gps_results[:, 18]
top_left_lat, top_left_lon = may_gps_results[:, 19], may_gps_results[:, 20]
bottom_right_lat, bottom_right_lon = may_gps_results[:, 21], may_gps_results[:, 22]
bottom_left_lat, bottom_left_lon = may_gps_results[:, 23], may_gps_results[:, 24]

# Read the GPS data
nov_gps_results = np.genfromtxt("Nov_results/results.csv", delimiter=",", skip_header=1)
print(nov_gps_results.shape)
 
# Extract timestamps, latitudes, and longitudes
nov_timestamps = nov_gps_results[:, 0]
nov_latitudes = nov_gps_results[:, 1]
nov_longitudes = nov_gps_results[:, 2]

ouster_lat, ouster_lon = nov_gps_results[:, 5], nov_gps_results[:, 6]
# Extract wheel coordinates
front_left_lat, front_left_lon = nov_gps_results[:, 7], nov_gps_results[:, 8]
front_right_lat, front_right_lon = nov_gps_results[:, 9], nov_gps_results[:, 10]
rear_left_lat, rear_left_lon = nov_gps_results[:, 11], nov_gps_results[:, 12]
rear_right_lat, rear_right_lon = nov_gps_results[:, 13], nov_gps_results[:, 14]

leftCam_top_right_lat, leftCam_top_right_lon = nov_gps_results[:,15] , nov_gps_results[:,16] 
leftCam_top_left_lat, leftCam_top_left_lon = nov_gps_results[:,17], nov_gps_results[:,18]
leftCam_bottom_right_lat, leftCam_bottom_right_lon = nov_gps_results[:,19], nov_gps_results[:,20]
leftCam_bottom_left_lat, leftCam_bottom_left_lon = nov_gps_results[:,21], nov_gps_results[:,22]

left_cam_lat = nov_gps_results[:,23]; left_cam_lon = nov_gps_results[:,24]

rightCam_top_right_lat, rightCam_top_right_lon = nov_gps_results[:,25] , nov_gps_results[:,26] 
rightCam_top_left_lat, rightCam_top_left_lon = nov_gps_results[:,27], nov_gps_results[:,28]
rightCam_bottom_right_lat, rightCam_bottom_right_lon = nov_gps_results[:,29], nov_gps_results[:,30]
rightCam_bottom_left_lat, rightCam_bottom_left_lon = nov_gps_results[:,31], nov_gps_results[:,32]

right_cam_lat = nov_gps_results[:,33]; right_cam_lon = nov_gps_results[:,34]

may_imgs = "/media/kristen/easystore3/RestorebotData/May2022_backups/1conmod/down_imgs"

match_dir = "/media/kristen/easystore3/matches"

n_tsteps = len(available_timestamps)

for x,timestamp in enumerate(available_timestamps): 
    print("processing {} out of {}".format(x,n_tsteps))
    #may timestamp 
    may_tstep_idx = find_closest_index(may_timestamps,timestamp)
    may_cam_lat = cam_lat[may_tstep_idx]; may_cam_lon = cam_lon[may_tstep_idx] 

    # Example usage with November data
    cov_matrix = load_covariance_matrix(may_covariance_dir,available_timestamps,timestamp)  # Load current covariance matrix 
    if cov_matrix is None:
        print("cov_matrix: ",cov_matrix)
        input("Press Enter to Continue")
        continue 

    scaled_cov_matrix = np.diag([1 / LAT_METERS_PER_DEGREE, 1 / LON_METERS_PER_DEGREE]) @ cov_matrix @ np.diag([1 / LAT_METERS_PER_DEGREE, 1 / LON_METERS_PER_DEGREE])

    closest_idx = np.argmin(np.abs(may_timestamps - timestamp))
    print("closest_idx: ",closest_idx) 

    #find the timestamp and get the May coord 
    center = (cam_lon[closest_idx], cam_lat[closest_idx]) 
    
    may_frames_inside = find_frames_inside_ellipse(
        scaled_cov_matrix,
        center,
        cam_lon, 
        cam_lat, 
        may_timestamps
    )

    right_frames_inside = find_frames_inside_ellipse(
        scaled_cov_matrix,
        center,
        right_cam_lon, 
        right_cam_lat, 
        nov_timestamps
    )

    # Print results
    if len(right_frames_inside) > 0:
        print(f"Frames inside covariance ellipse: {len(right_frames_inside)}")

    frame_dir = "/media/kristen/easystore3/RestorebotData/Nov2022/Nov6/1conmod/down_imgs1"
    closest_right_frames = []

    closest_nov_frame_tstamp = None 
    closest_nov_frame_d = np.inf 

    close_right_frames = 0
    for frame in right_frames_inside:
        #print(f"Timestamp: {frame[0]}, Longitude: {frame[1]}, Latitude: {frame[2]}") 
        idx = find_closest_index(nov_timestamps,frame[0]) 
        rcam_lat = right_cam_lat[idx]; rcam_lon = right_cam_lon[idx]
        if are_coordinates_within_threshold((rcam_lat,rcam_lon),(may_cam_lat,may_cam_lon)): 
            close_right_frames += 1 
            if geodesic((rcam_lat,rcam_lon),(may_cam_lat,may_cam_lon)).meters < closest_nov_frame_d:
                closest_nov_frame_tstamp = nov_timestamps[idx] 
                closest_nov_frame_d = geodesic((rcam_lat,rcam_lon),(may_cam_lat,may_cam_lon)).meters 

        closest_frame = find_closest_frame(frame_dir,frame[0])
        #print("closest_frame: ",closest_frame) 
        closest_right_frames.append(closest_frame) 

    left_frames_inside = find_frames_inside_ellipse(
        scaled_cov_matrix,
        center,
        left_cam_lon, 
        left_cam_lat, 
        nov_timestamps
    )

    # Print results
    if len(left_frames_inside):
        print(f"Frames inside covariance ellipse: {len(left_frames_inside)}")

    frame_dir ="/media/kristen/easystore3/RestorebotData/Nov2022/Nov6/1conmod/down_imgs"
    closest_left_frames = []

    close_left_frames = 0
    for frame in left_frames_inside:
        #print(f"Timestamp: {frame[0]}, Longitude: {frame[1]}, Latitude: {frame[2]}")
        closest_frame = find_closest_frame(frame_dir,frame[0]) 
        #print("closest_frame: ",closest_frame) 
        closest_left_frames.append(closest_frame) 
        idx = find_closest_index(nov_timestamps,frame[0]) 
        lcam_lat = right_cam_lat[idx]; lcam_lon = right_cam_lon[idx]
        if are_coordinates_within_threshold((lcam_lat,lcam_lon),(may_cam_lat,may_cam_lon)): 
            close_left_frames += 1 
            if geodesic((lcam_lat,lcam_lon),(may_cam_lat,may_cam_lon)).meters < closest_nov_frame_d:
                closest_nov_frame_tstamp = nov_timestamps[idx] 
                closest_nov_frame_d = geodesic((lcam_lat,lcam_lon),(may_cam_lat,may_cam_lon)).meters 
             
    if close_left_frames == 0 and close_right_frames == 0:
        print("camera too far! skipping ...")
        continue 
    
    if len(right_frames_inside) > 0 or len(left_frames_inside) > 0: 
        may_frame_t = find_closest_frame(may_imgs,timestamp) 

        if not os.path.exists(os.path.join(match_dir,str(timestamp))):
            os.mkdir(os.path.join(match_dir,str(timestamp)))
        match_t_dir = os.path.join(match_dir,str(timestamp)) 
        shutil.copy(may_frame_t,os.path.join(match_t_dir,"may_frame.jpg")) 

        if not os.path.exists(os.path.join(match_t_dir,"may_frames")):
            os.mkdir(os.path.join(match_t_dir,"may_frames")) 

        for frame in may_frames_inside:
            closest_frame = find_closest_frame(may_imgs,frame[0])  
            #print("closest_frame: ",closest_frame)
            shutil.copy(closest_frame,os.path.join(match_t_dir,"may_frames")) 

        if len(closest_right_frames) > 0:
            if not os.path.exists(os.path.join(match_t_dir,"right_frames")): 
                os.mkdir(os.path.join(match_t_dir,"right_frames"))

        for img_path in closest_right_frames: 
            shutil.copy(img_path,os.path.join(match_t_dir,"right_frames"))
        
        if len(closest_left_frames) > 0:
            if not os.path.exists(os.path.join(match_t_dir,"left_frames")): 
                os.mkdir(os.path.join(match_t_dir,"left_frames"))

        for img_path in closest_left_frames: 
            shutil.copy(img_path,os.path.join(match_t_dir,"left_frames")) 
        
        # Initialize the plot
        plt.figure(figsize=(10, 8))

        # Plot overlapping May frames (blue)
        for frame in closest_left_frames:  # Closest left frames that overlap
            frame_coordinates = [
                (top_right_lon[closest_idx], top_right_lat[closest_idx]),
                (top_left_lon[closest_idx], top_left_lat[closest_idx]),
                (bottom_left_lon[closest_idx], bottom_left_lat[closest_idx]),
                (bottom_right_lon[closest_idx], bottom_right_lat[closest_idx])
            ]
            polygon = Polygon(frame_coordinates, closed=True, edgecolor='blue', facecolor='blue', alpha=0.5)
            plt.gca().add_patch(polygon)

        # Plot overlapping left camera frames (orange)
        for frame in left_frames_inside:  # Left camera frames that overlap
            frame_coordinates = [
                (leftCam_top_right_lon[closest_idx], leftCam_top_right_lat[closest_idx]),
                (leftCam_top_left_lon[closest_idx], leftCam_top_left_lat[closest_idx]),
                (leftCam_bottom_left_lon[closest_idx], leftCam_bottom_left_lat[closest_idx]),
                (leftCam_bottom_right_lon[closest_idx], leftCam_bottom_right_lat[closest_idx])
            ]
            polygon = Polygon(frame_coordinates, closed=True, edgecolor='orange', facecolor='orange', alpha=0.5)
            plt.gca().add_patch(polygon)

        # Plot overlapping right camera frames (red)
        for frame in right_frames_inside:  # Right camera frames that overlap
            frame_coordinates = [
                (rightCam_top_right_lon[closest_idx], rightCam_top_right_lat[closest_idx]),
                (rightCam_top_left_lon[closest_idx], rightCam_top_left_lat[closest_idx]),
                (rightCam_bottom_left_lon[closest_idx], rightCam_bottom_left_lat[closest_idx]),
                (rightCam_bottom_right_lon[closest_idx], rightCam_bottom_right_lat[closest_idx])
            ]
            polygon = Polygon(frame_coordinates, closed=True, edgecolor='red', facecolor='red', alpha=0.5)
            plt.gca().add_patch(polygon)


        # Create dummy patches for the legend
        blue_patch = mpatches.Patch(color='blue', alpha=0.5, label='Overlapping May Frames')
        orange_patch = mpatches.Patch(color='orange', alpha=0.5, label='Overlapping Nov Left Frames')
        red_patch = mpatches.Patch(color='red', alpha=0.5, label='Overlapping Nov Right Frames')


        for i in range(len(cam_lat)):
            frame_coordinates = [
                (top_right_lon[i],top_right_lat[i]),
                (top_left_lon[i],top_left_lat[i]),
                (bottom_left_lon[i],bottom_left_lat[i]),
                (bottom_right_lon[i],bottom_right_lat[i])
            ]
            polygon = Polygon(frame_coordinates, closed=True, edgecolor='blue', facecolor='none', alpha=0.25)
            plt.gca().add_patch(polygon)

        # Plot camera frames for November results (orange frames for left camera)
        for i in range(len(leftCam_top_right_lat)):
            frame_coordinates = [
                (leftCam_top_right_lon[i], leftCam_top_right_lat[i]),
                (leftCam_top_left_lon[i], leftCam_top_left_lat[i]),
                (leftCam_bottom_left_lon[i], leftCam_bottom_left_lat[i]),
                (leftCam_bottom_right_lon[i], leftCam_bottom_right_lat[i])
            ]
            polygon = Polygon(frame_coordinates, closed=True, edgecolor='orange', facecolor='none', alpha=0.25)
            plt.gca().add_patch(polygon)

        # Plot camera frames for November results (red frames for right camera)
        for i in range(len(rightCam_top_right_lat)):
            frame_coordinates = [
                (rightCam_top_right_lon[i], rightCam_top_right_lat[i]),
                (rightCam_top_left_lon[i], rightCam_top_left_lat[i]),
                (rightCam_bottom_left_lon[i], rightCam_bottom_left_lat[i]),
                (rightCam_bottom_right_lon[i], rightCam_bottom_right_lat[i])
            ]
            polygon = Polygon(frame_coordinates, closed=True, edgecolor='red', facecolor='none', alpha=0.25)
            plt.gca().add_patch(polygon)
        
        # Load covariance matrix and create ellipse
        #cov_matrix = load_covariance_matrix(may_covariance_dir, available_timestamps, may_timestamps[i])
        if cov_matrix is not None:
            ellipse = create_uncertainty_ellipse(
                cov_matrix,
                center=(may_cam_lon,may_cam_lat),
                ax=plt.gca(),
                edgecolor='black',
                facecolor='none', 
                linewidth=1,
                label="May Frame Covariance"
            )
        print("ellipse: ",ellipse) 
        if ellipse:  # Check if ellipse was successfully created
            plt.gca().add_patch(ellipse)
        else:
            print(f"Skipping ellipse for timestamp {may_timestamps[may_tstep_idx]} due to invalid covariance matrix.") 

        '''
        nov_cov_matrix = load_covariance_matrix(nov_covariance_dir,nov_available_timestamps,closest_nov_frame_tstamp) 

        if nov_cov_matrix is not None: 
            nov_tstep_idx = find_closest_index(nov_timestamps,closest_nov_frame_tstamp)
            nov_cam_left_lat = left_cam_lat[nov_tstep_idx] 
            nov_cam_left_lon = left_cam_lon[nov_tstep_idx] 
            nov_cam_right_lat = right_cam_lat[nov_tstep_idx] 
            nov_cam_right_lon = right_cam_lon[nov_tstep_idx]  
            nov_cam_lon_t = np.mean([nov_cam_left_lon,nov_cam_right_lon])
            nov_cam_lat_t = np.mean([nov_cam_left_lat,nov_cam_right_lat])
            ellipse = create_uncertainty_ellipse(
                cov_matrix,
                center=(nov_cam_lon_t,nov_cam_lat_t),
                ax=plt.gca(),
                edgecolor='red',
                facecolor='none', 
                linewidth=1
            )
        else: 
            input("Press enter to acknowledge")
        '''
       
        # Turn on the grid
        plt.grid(True)

        # Set plot labels and limits
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title("Overlapping Camera Frames")
        plt.axis("equal")  # Ensure axis reflects lat/lon scale

        # Save the plot to a file
        # Add legend to the plot
        plt.legend(handles=[blue_patch, orange_patch, red_patch, ellipse], loc='upper right') 

        output_filename = os.path.join(match_t_dir,"overlapping_frames_plot.png")
        plt.savefig(output_filename, dpi=300)  # Save with high resolution (300 DPI)
        print("writing {}".format(output_filename)) 
        # Display the plot
        plt.show()

        input("Press Enter to Continue ...")

        plt.close() 