import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
import os
from PIL import Image

LAT_METERS_PER_DEGREE = 111320  # meters per degree latitude
LON_METERS_PER_DEGREE = 85390   # meters per degree longitude at this latitude
CURRENT_TIMESTAMP = None

trail_duration = 1.0  # Duration of the trail in seconds
fade_duration = 15.0  # Duration for fading points

frame_dir = "/media/kristen/easystore3/front_imgs"  # Directory containing frame images
current_frame_image = None  # Global variable to hold the current image figure
image_ax = None  # Global variable for the image axis

# Pause flag
is_paused = True 
preserved_xlim = None  # To store x-axis limits
preserved_ylim = None  # To store y-axis limits

import pandas as pd

# Directory containing covariance matrices
covariance_dir = "May_results/covariance_matrices"

# Precompute all available covariance matrix timestamps
import glob

def precompute_timestamps():
    """Precompute all available timestamps from the covariance matrix files."""
    print("precomputing timestamps ...")
    filenames = glob.glob(os.path.join(covariance_dir, "*.csv"))
    timestamps = [int(os.path.basename(f).split(".")[0]) for f in filenames]  # Convert nanoseconds to seconds
    return sorted(timestamps)

available_timestamps = precompute_timestamps()  # Precompute once at the start
print("available timestamps: ",available_timestamps)
print("Done!")

def load_covariance_matrix(timestamp):
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

from matplotlib.patches import Ellipse

def create_uncertainty_ellipse(cov_matrix, center, ax, **kwargs):
    """Create an uncertainty ellipse scaled for lat/lon plotting."""
    if cov_matrix is None:
        return None

    # Convert covariance matrix from meters² to degrees²
    lat_scale = 1 / LAT_METERS_PER_DEGREE
    lon_scale = 1 / LON_METERS_PER_DEGREE
    scale_matrix = np.diag([lat_scale, lon_scale])  # Scale factors for lat/lon
    scaled_cov_matrix = scale_matrix @ cov_matrix @ scale_matrix.T

    # Eigenvalues and eigenvectors for ellipse orientation and scaling
    eigenvalues, eigenvectors = np.linalg.eig(scaled_cov_matrix)
    major_axis = 2 * np.sqrt(eigenvalues[0])  # 2-sigma width
    minor_axis = 2 * np.sqrt(eigenvalues[1])  # 2-sigma height
    angle = np.degrees(np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0]))  # Orientation

    # Create and return the ellipse
    ellipse = Ellipse(
        xy=center, width=major_axis, height=minor_axis, angle=angle, **kwargs
    )
    #ax.add_patch(ellipse)
    return ellipse

def update_closest_frame(timestamp):
    """Update the displayed image to match the closest frame for the current timestamp."""
    print("updating closest frame...")
    global current_frame_image, image_ax
    frame_path = find_closest_frame(timestamp)
    if frame_path is not None and os.path.exists(frame_path):
        img = Image.open(frame_path)

        if current_frame_image is None:  # Create the image figure only once
            current_frame_image = plt.figure("Closest Frame")
            image_ax = current_frame_image.add_subplot(111)
            image_ax.axis("off")  # Turn off axes
        image_ax.clear()
        image_ax.imshow(img)
        image_ax.axis("off")
        image_ax.set_title(f"Frame: {os.path.basename(frame_path)}", fontsize=10, loc='center')
        current_frame_image.canvas.draw_idle()  # Refresh the figure
        plt.pause(0.01)  # Allow the secondary figure to update
    else:
        print("Invalid frame path.")
        return 

def find_closest_frame(timestamp):
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

# Annotated compass data
automated_annotations = np.genfromtxt("May_results/data.csv", delimiter=",")  # [row_tstep, true_course, measured_yaw, angular_vel]
annotated_tsteps = automated_annotations[:, 0]

# Read the GPS data
gps_results = np.genfromtxt("May_results/results.csv", delimiter=",", skip_header=1)

# Extract timestamps, latitudes, and longitudes
timestamps = gps_results[:, 0]
filtered_latitudes = gps_results[:, 1]
filtered_longitudes = gps_results[:, 2]

ouster_lat, ouster_lon = gps_results[:, 5], gps_results[:, 6]
# Extract wheel coordinates
front_left_lat, front_left_lon = gps_results[:, 7], gps_results[:, 8]
front_right_lat, front_right_lon = gps_results[:, 9], gps_results[:, 10]
rear_left_lat, rear_left_lon = gps_results[:, 11], gps_results[:, 12]
rear_right_lat, rear_right_lon = gps_results[:, 13], gps_results[:, 14]

# Use camera coordinates directly instead of frame center
cam_lat, cam_lon = gps_results[:, 15], gps_results[:, 16]

# Extract frame corner coordinates
top_right_lat, top_right_lon = gps_results[:, 17], gps_results[:, 18]
top_left_lat, top_left_lon = gps_results[:, 19], gps_results[:, 20]
bottom_right_lat, bottom_right_lon = gps_results[:, 21], gps_results[:, 22]
bottom_left_lat, bottom_left_lon = gps_results[:, 23], gps_results[:, 24]

# Define fixed plot boundaries
initial_lat_min, initial_lat_max = 38.10985, 38.110
initial_lon_min, initial_lon_max = -109.60225, -109.60192

# Create the plot
fig, ax = plt.subplots()

# Initialize scatter plots
recent_plot, = ax.plot([], [], 'o', color='red', markersize=8, alpha=1.0, zorder=5)
older_plot, = ax.plot([], [], 'o', color='red', markersize=6, alpha=0.2, zorder=4)
very_old_plot, = ax.plot([], [], 'o', color='red', markersize=4, alpha=0.05, zorder=3)

# Initialize the rectangle for the robot
robot_rectangle = patches.Polygon([[0, 0], [0, 0], [0, 0], [0, 0]], closed=True, edgecolor='blue', facecolor='none', linewidth=2, zorder=10)
ax.add_patch(robot_rectangle)

camera_rectangle = patches.Polygon([[0, 0], [0, 0], [0, 0], [0, 0]], closed=True, edgecolor='green', facecolor='none', linewidth=2, zorder=10)
ax.add_patch(camera_rectangle) 

# Initialize the ouster scatter plot
ouster_plot, = ax.plot([], [], 'o', color='purple', markersize=10, alpha=0.8, zorder=5)

# Set plot title, labels, and fixed limits
ax.set_title("GPS Coordinates Animation")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.set_xlim([initial_lon_min, initial_lon_max])
ax.set_ylim([initial_lat_min, initial_lat_max])
plt.grid(True)
ax.set_aspect(LAT_METERS_PER_DEGREE / LON_METERS_PER_DEGREE)
 
# Pause flag: Start paused
is_paused = True
paused_frame = 0  # Frame to start from when resuming

def on_key_press(event):
    """Toggle pause/play on space key press."""
    global is_paused, preserved_xlim, preserved_ylim, paused_frame
    if event.key == ' ':
        is_paused = not is_paused
        if is_paused:
            # Preserve the current zoom level
            preserved_xlim = ax.get_xlim()
            preserved_ylim = ax.get_ylim()
            print("Animation paused!")
        else:
            # Restore the preserved zoom level if paused earlier
            print("Animation resumed!")

fig.canvas.mpl_connect('key_press_event', on_key_press)

def update(frame):
    """Update the plot and the displayed image."""
    global CURRENT_TIMESTAMP, is_paused, preserved_xlim, preserved_ylim, paused_frame
    print(f'update called .... this is frame: {frame}')

    # If paused, do not process the frame
    if is_paused:
        print("is_paused is true, returning old plot...")
        return (recent_plot, older_plot, very_old_plot,
                robot_rectangle, camera_rectangle,
                ouster_plot)

    # Save the current frame index as paused_frame
    paused_frame = frame

    # Update current time
    current_time = timestamps[frame]
    CURRENT_TIMESTAMP = current_time

    # Update the closest frame (image)
    update_closest_frame(current_time)
    print("current_time: ", current_time)

    # Update scatter plots
    time_window_start = current_time - trail_duration
    fade_window_start = current_time - fade_duration

    recent_idx = (timestamps >= time_window_start) & (timestamps <= current_time)
    older_idx = (timestamps >= fade_window_start) & (timestamps < time_window_start)
    very_old_idx = (timestamps < fade_window_start)

    recent_plot.set_data(filtered_longitudes[recent_idx], filtered_latitudes[recent_idx])
    older_plot.set_data(filtered_longitudes[older_idx], filtered_latitudes[older_idx])
    very_old_plot.set_data(filtered_longitudes[very_old_idx], filtered_latitudes[very_old_idx])

    # Update robot rectangle
    robot_coords = [
        [front_left_lon[frame], front_left_lat[frame]],
        [front_right_lon[frame], front_right_lat[frame]],
        [rear_right_lon[frame], rear_right_lat[frame]],
        [rear_left_lon[frame], rear_left_lat[frame]]
    ]
    robot_rectangle.set_xy(robot_coords)

    # Update camera rectangle
    camera_coords = [
        [top_left_lon[frame], top_left_lat[frame]],
        [top_right_lon[frame], top_right_lat[frame]],
        [bottom_right_lon[frame], bottom_right_lat[frame]],
        [bottom_left_lon[frame], bottom_left_lat[frame]]
    ]
    camera_rectangle.set_xy(camera_coords)

    # Update the ouster point
    ouster_plot.set_data(ouster_lon[frame], ouster_lat[frame])

    # Only dynamically adjust axis limits if not paused and zoom is not manually set
    if not is_paused and not (preserved_xlim and preserved_ylim):
        lat_min, lat_max = np.min(filtered_latitudes) - 0.0001, np.max(filtered_latitudes) + 0.0001
        lon_min, lon_max = np.min(filtered_longitudes) - 0.0001, np.max(filtered_longitudes) + 0.0001
        ax.set_xlim([lon_min, lon_max])
        ax.set_ylim([lat_min, lat_max])
    
    # Add to the top of your update function
    ellipse = None  # Initialize a variable to hold the uncertainty ellipse

    # Inside the update function, before returning plots
    cov_matrix = load_covariance_matrix(current_time)
    print("cov_matrix: ",cov_matrix) 

    if ellipse:
        ellipse.remove()  # Remove the previous ellipse if it exists

    ellipse = create_uncertainty_ellipse(
        cov_matrix,
        center=(filtered_longitudes[frame], filtered_latitudes[frame]),
        ax=ax,
        edgecolor=None,
        facecolor='red',
        linewidth=1.5,
        alpha=0.1
    )

    fig.canvas.draw_idle()
    return (recent_plot, older_plot, very_old_plot,
            robot_rectangle, camera_rectangle,
            ouster_plot,ellipse)

def frame_generator():
    """Generator function to yield frames only when not paused."""
    global paused_frame
    while paused_frame < len(timestamps):
        if not is_paused:
            yield paused_frame  # Yield the current frame index
            paused_frame += 1  # Increment to the next frame
        else:
            plt.pause(0.1)  # Small delay to avoid busy waiting while paused

# Create the animation
ani = animation.FuncAnimation(
    fig,
    update,
    frames=frame_generator,  # Start from paused frame
    interval=250,
    blit=False,
    repeat=False
)

plt.show()