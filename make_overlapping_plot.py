import os
import glob 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.patches as mpatches 
from matplotlib.patches import Ellipse 

def load_covariance_matrix(covariance_dir,available_timestamps,timestamp):
    """Load the covariance matrix for the given timestamp."""
    # Find the closest available timestamp
    closest_timestamp = min(available_timestamps, key=lambda t: abs(t*10**(-9) - timestamp)) 
    tmp = closest_timestamp*10**(-9)
    filename = os.path.join(covariance_dir, f"{closest_timestamp}.csv") 
    #print("filename: ",filename)
    if not os.path.exists(filename):
        raise OSError 
    #print("delta_t: ",abs(timestamp - tmp) )
    # Check if the closest timestamp is within 0.1 seconds
    if abs(timestamp - tmp) <= 0.1:
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


def precompute_timestamps(covariance_dir):
    """Precompute all available timestamps from the covariance matrix files."""
    print("precomputing timestamps ...")
    filenames = glob.glob(os.path.join(covariance_dir, "*.csv"))
    timestamps = [int(os.path.basename(f).split(".")[0]) for f in filenames]  # Convert nanoseconds to seconds
    return sorted(timestamps)
    
LAT_METERS_PER_DEGREE = 111320  # meters per degree latitude
LON_METERS_PER_DEGREE = 85390   # meters per degree longitude at this latitude

# Annotated compass data
automated_annotations = np.genfromtxt("May_results/data.csv", delimiter=",")  # [row_tstep, true_course, measured_yaw, angular_vel]
may_annotated_tsteps = automated_annotations[:, 0]

# Read the GPS data
may_gps_results = np.genfromtxt("May_results/results.csv", delimiter=",", skip_header=1)

# Extract timestamps, latitudes, and longitudes
may_timestamps = may_gps_results[:, 0]
filtered_latitudes = may_gps_results[:, 1]
filtered_longitudes = may_gps_results[:, 2]

ouster_lat, ouster_lon = may_gps_results[:, 5], may_gps_results[:, 6]
# Extract wheel coordinates
front_left_lat, front_left_lon = may_gps_results[:, 7], may_gps_results[:, 8]
front_right_lat, front_right_lon = may_gps_results[:, 9], may_gps_results[:, 10]
rear_left_lat, rear_left_lon = may_gps_results[:, 11], may_gps_results[:, 12]
rear_right_lat, rear_right_lon = may_gps_results[:, 13], may_gps_results[:, 14]

# Use camera coordinates directly instead of frame center
cam_lat, cam_lon = may_gps_results[:, 15], may_gps_results[:, 16]

# Extract frame corner coordinates
top_right_lat, top_right_lon = may_gps_results[:, 17], may_gps_results[:, 18]
top_left_lat, top_left_lon = may_gps_results[:, 19], may_gps_results[:, 20]
bottom_right_lat, bottom_right_lon = may_gps_results[:, 21], may_gps_results[:, 22]
bottom_left_lat, bottom_left_lon = may_gps_results[:, 23], may_gps_results[:, 24]

# Directory containing covariance matrices
may_covariance_dir = "May_results/fake_covariance_matrices"

may_available_timestamps = precompute_timestamps(may_covariance_dir)  # Precompute once at the start 
print("got the available timestamps: ",len(may_available_timestamps)) 

############################################################################################################
# Annotated compass data
automated_annotations = np.genfromtxt("Nov_results/data.csv", delimiter=",")  # [row_tstep, true_course, measured_yaw, angular_vel]
nov_annotated_tsteps = automated_annotations[:, 0]

# Read the GPS data
nov_gps_results = np.genfromtxt("Nov_results/results.csv", delimiter=",", skip_header=1)
print(nov_gps_results.shape)
 
# Extract timestamps, latitudes, and longitudes
nov_timestamps = nov_gps_results[:, 0]
filtered_latitudes = nov_gps_results[:, 1]
filtered_longitudes = nov_gps_results[:, 2]

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

# Directory containing covariance matrices
nov_covariance_dir = "Nov_results/covariance_matrices"

nov_available_timestamps = precompute_timestamps(nov_covariance_dir)  # Precompute once at the start

# Initialize the plot
plt.figure(figsize=(10, 8))

'''

# Plot camera frames for November results (orange frames for left camera)
for i in range(len(leftCam_top_right_lat)):
    frame_coordinates = [
        (leftCam_top_right_lon[i], leftCam_top_right_lat[i]),
        (leftCam_top_left_lon[i], leftCam_top_left_lat[i]),
        (leftCam_bottom_left_lon[i], leftCam_bottom_left_lat[i]),
        (leftCam_bottom_right_lon[i], leftCam_bottom_right_lat[i])
    ]
    polygon = Polygon(frame_coordinates, closed=True, edgecolor='orange', facecolor='orange', alpha=0.1)
    plt.gca().add_patch(polygon)

# Plot camera frames for November results (red frames for right camera)
for i in range(len(rightCam_top_right_lat)):
    frame_coordinates = [
        (rightCam_top_right_lon[i], rightCam_top_right_lat[i]),
        (rightCam_top_left_lon[i], rightCam_top_left_lat[i]),
        (rightCam_bottom_left_lon[i], rightCam_bottom_left_lat[i]),
        (rightCam_bottom_right_lon[i], rightCam_bottom_right_lat[i])
    ]
    polygon = Polygon(frame_coordinates, closed=True, edgecolor='red', facecolor='red', alpha=0.1)
    plt.gca().add_patch(polygon)


all_match_tsteps = [int(x) for x in os.listdir("/media/kristen/easystore3/matches")]

for match_t in all_match_tsteps: 
    match_t_dir = os.path.join("/media/kristen/easystore3/matches",str(match_t))
    # In your loop, add a check before calling `plt.gca().add_patch`
    for i in range(len(top_right_lat)):
        frame_coordinates = [
            (top_right_lon[i], top_right_lat[i]),
            (top_left_lon[i], top_left_lat[i]),
            (bottom_left_lon[i], bottom_left_lat[i]),
            (bottom_right_lon[i], bottom_right_lat[i])
        ]
        polygon = Polygon(frame_coordinates, closed=True, edgecolor='blue', facecolor='blue', alpha=0.1)
        plt.gca().add_patch(polygon)
        print("may_timestamps[i]:", may_timestamps[i])


        if np.abs(may_timestamps[i] - match_t * 10**(-9)) < 0.1:
            polygon = Polygon(frame_coordinates, closed=True, edgecolor='black', alpha=1)       
            plt.gca().add_patch(polygon)

            # Load covariance matrix and create ellipse
            cov_matrix = load_covariance_matrix(may_covariance_dir, may_available_timestamps, may_timestamps[i])
            if cov_matrix is not None:
                ellipse = create_uncertainty_ellipse(
                    cov_matrix,
                    center=(cam_lon[i], cam_lat[i]),
                    ax=plt.gca(),
                    edgecolor='black',
                    facecolor='none', 
                    linewidth=1
                )
                print("ellipse: ",ellipse) 
                if ellipse:  # Check if ellipse was successfully created
                    plt.gca().add_patch(ellipse)
                else:
                    print(f"Skipping ellipse for timestamp {may_timestamps[i]} due to invalid covariance matrix.") 
            
'''

for i in range(len(top_right_lat)):
    frame_coordinates = [
        (top_right_lon[i], top_right_lat[i]),
        (top_left_lon[i], top_left_lat[i]),
        (bottom_left_lon[i], bottom_left_lat[i]),
        (bottom_right_lon[i], bottom_right_lat[i])
    ]
    polygon = Polygon(frame_coordinates, closed=True, edgecolor='blue', facecolor='blue', alpha=0.1)
    plt.gca().add_patch(polygon)
    
# Create dummy patches for the legend
blue_patch = mpatches.Patch(color='blue', alpha=0.1, label='May Frames')
#orange_patch = mpatches.Patch(color='orange', alpha=0.1, label='Nov Left Frames')
#red_patch = mpatches.Patch(color='red', alpha=0.1, label='Nov Right Frames')

# Add legend to the plot
plt.legend(handles=[blue_patch], loc='upper right')

# Turn on the grid
plt.grid(True)

# Set plot labels and limits
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Camera Frames: Conmod1")
plt.axis("equal")  # Ensure axis reflects lat/lon scale

# Save the plot to a file
output_filename = os.path.join("camera_frames_plot.png")
plt.savefig(output_filename, dpi=300)  # Save with high resolution (300 DPI)

# Display the plot
#plt.show()

print(f"Plot saved as {output_filename}")
