import matplotlib
matplotlib.use("TkAgg")  # Or "Qt5Agg" depending on your environment
import time 

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from matplotlib.animation import FFMpegWriter
import numpy as np
import os
from PIL import Image
import math 

# Define file paths for output videos
plot_video_file = "gps_plot.mp4"
frame_video_file = "closest_frame.mp4"

# Set up FFMpegWriter
fps = 5  # Frames per second
plot_writer = FFMpegWriter(fps=fps, metadata=dict(artist='Me'), bitrate=1800)
frame_writer = FFMpegWriter(fps=fps, metadata=dict(artist='Me'), bitrate=1800)

LAT_METERS_PER_DEGREE = 111320  # meters per degree latitude
LON_METERS_PER_DEGREE = 85390   # meters per degree longitude at this latitude
CURRENT_TIMESTAMP = None

trail_duration = 1.0  # Duration of the trail in seconds
fade_duration = 15.0  # Duration for fading points

frame_dir = "/media/kristen/easystore2/RestorebotData/May2022_final/1conmod/Front_Facing_Images" 

current_frame_image = None  # Global variable to hold the current image figure
image_ax = None  # Global variable for the image axis

# Pause flag
is_paused = True 
preserved_xlim = None  # To store x-axis limits
preserved_ylim = None  # To store y-axis limits

import pandas as pd

# Directory containing covariance matrices
covariance_dir = "May_results/finalized_May_results/1conmod/covariance_matrices"
#1545615532109757900.csv

# Precompute all available covariance matrix timestamps
import glob

def precompute_timestamps():
    """Precompute all available timestamps from the covariance matrix files."""
    print("precomputing timestamps ...")
    filenames = glob.glob(os.path.join(covariance_dir, "*.csv"))
    timestamps = [int(os.path.basename(f).split(".")[0]) for f in filenames]  # Convert nanoseconds to seconds
    return sorted(timestamps)

available_timestamps = precompute_timestamps()  # Precompute once at the start
print("min(available_timestamps):{}, max(available_timetamps):{}".format(min(available_timestamps),max(available_timestamps))) 

def load_covariance_matrix(timestamp):
    """Load the covariance matrix for the given timestamp."""
    # Find the closest available timestamp
    print("timestamp: {}, available_timestamps[0]: {}".format(timestamp,available_timestamps[0]))
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

def update_closest_frame(timestamp,compass_headings):
    """Update the displayed image to match the closest frame for the current timestamp."""
    #print("updating closest frame...")
    global current_frame_image, image_ax
    frame_path = find_closest_frame(timestamp,compass_headings)
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
        time.sleep(0.01)  # Allow the secondary figure to update
    else:
        print("Invalid frame path.")
        return 

def find_closest_frame(timestamp,compass_headings):
    """Find the closest frame to the given timestamp."""
    frame_files = [f for f in os.listdir(frame_dir) if f.endswith(".png") or f.endswith(".jpg")]
    frame_timestamps = [10 ** (-9) * int(f.split(".")[0]) for f in frame_files]
    closest_timestamp = min(frame_timestamps, key=lambda t: abs(t - timestamp))
    #get the corresponding heading 
    idx = np.abs(compass_headings[:,0] - closest_timestamp).argmin() 
    print("Corresponding compass heading: ",compass_headings[idx,1]) 
    tstep_str = str(int(closest_timestamp * 1e9))
    #print("looking for matching files to this timestamp:")
    matching_files = [
        os.path.join(frame_dir, f)
        for f in os.listdir(frame_dir)
        if tstep_str[:-5] in f
    ]

    if len(matching_files) > 0:
        print("closest_timestamp: ",closest_timestamp)
        print("timestamp: ",timestamp)
        delta_t = abs(closest_timestamp - timestamp) 
        print("delta_t: ",delta_t)
        if delta_t > 1:
            raise OSError 
        return matching_files[0]
    else: 
        #print("there are no matches...")
        matching_files = [
            os.path.join(frame_dir, f)
            for f in os.listdir(frame_dir)
            if tstep_str[:11] in f
        ] 
        matching_file_timestamp = int(os.path.basename(matching_files[0][:-4]))
        if math.floor(math.log10(abs(matching_file_timestamp))) == 18: 
            matching_file_timestamp = matching_file_timestamp * 10 **(-9)
        delta_t = abs(matching_file_timestamp - timestamp) 

        if delta_t > 1: 
            print("matching_file_timestamp: ",matching_file_timestamp)
            print("timestamp: ",timestamp) 
            print("delta_t: ",delta_t)
            raise OSError 
        return matching_files[0] 

# Annotated compass data
automated_annotations = np.genfromtxt("May_results/finalized_May_results/1conmod/data.csv", delimiter=",")  # [row_tstep, true_course, measured_yaw, angular_vel]
annotated_tsteps = automated_annotations[:, 0]

# Read the GPS data
gps_results = np.genfromtxt("May_results/finalized_May_results/1conmod/results.csv", delimiter=",", skip_header=1)
print("min(gps_results[:,0]): {}, max(gps_results[:,0]): {}".format(min(gps_results[:,0]),max(gps_results[:,0]))) 

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
    """Toggle pause/play on space key press and handle saving."""
    global is_paused, preserved_xlim, preserved_ylim, paused_frame
    if event.key == ' ':
        is_paused = not is_paused
        if is_paused:
            # Preserve the current zoom level
            preserved_xlim = ax.get_xlim()
            preserved_ylim = ax.get_ylim()
            print("Animation paused! Zoom in and adjust view.")
        else:
            # Restore the preserved zoom level when resuming
            if preserved_xlim and preserved_ylim:
                ax.set_xlim(preserved_xlim)
                ax.set_ylim(preserved_ylim)
            print("Animation resumed!")
    elif event.key == 's' and is_paused:
        # Save the current plot as an image with the preserved field of view
        if preserved_xlim and preserved_ylim:
            ax.set_xlim(preserved_xlim)
            ax.set_ylim(preserved_ylim)
        print(f"Saving frame {paused_frame} with preserved field of view...")
        fig.savefig(f"paused_frame_{paused_frame}.png", dpi=300)
        print(f"Paused frame {paused_frame} saved as 'paused_frame_{paused_frame}.png'")

def on_mouse_click(event):
    """Handle mouse click to select a frame to save."""
    global paused_frame, is_paused, preserved_xlim, preserved_ylim
    if is_paused and event.inaxes == ax:  # Only if animation is paused
        # Preserve the current zoom level
        preserved_xlim = ax.get_xlim()
        preserved_ylim = ax.get_ylim()

        print(f"Mouse clicked at ({event.xdata}, {event.ydata})")
        print(f"Saving frame {paused_frame} with preserved field of view...")
        
        # Save the current plot as an image with the preserved field of view
        if preserved_xlim and preserved_ylim:
            ax.set_xlim(preserved_xlim)
            ax.set_ylim(preserved_ylim)
        fig.savefig(f"selected_frame_{paused_frame}.png", dpi=300)
        print(f"Frame {paused_frame} saved as 'selected_frame_{paused_frame}.png'")
 
fig.canvas.mpl_connect('key_press_event', on_key_press)
fig.canvas.mpl_connect('button_press_event', on_mouse_click) 

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
    compass_data = np.genfromtxt("May_results/finalized_May_results/1conmod/processed_compass_heading.csv", delimiter=",", skip_header=1)  # t, compassHeading

    update_closest_frame(current_time,compass_data)
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

    # Dynamically adjust plot limits to frame the robot
    all_latitudes = [coord[1] for coord in robot_coords]
    all_longitudes = [coord[0] for coord in robot_coords]
    margin = 0.0001  # Adjust this for tighter or looser framing

    lat_min, lat_max = min(all_latitudes) - margin, max(all_latitudes) + margin
    lon_min, lon_max = min(all_longitudes) - margin, max(all_longitudes) + margin

    ax.set_xlim([lon_min, lon_max])
    ax.set_ylim([lat_min, lat_max])

    # Add to the top of your update function
    ellipse = None  # Initialize a variable to hold the uncertainty ellipse

    # Inside the update function, before returning plots
    cov_matrix = load_covariance_matrix(current_time)
    #print("cov_matrix: ", cov_matrix) 

    if ellipse:
        ellipse.remove()  # Remove the previous ellipse if it exists
    '''
    ellipse = create_uncertainty_ellipse(
        cov_matrix,
        center=(filtered_longitudes[frame], filtered_latitudes[frame]),
        ax=ax,
        edgecolor=None,
        facecolor='red',
        linewidth=1.5,
        alpha=0.1
    )
    '''

    fig.canvas.draw_idle()
    return (recent_plot, older_plot, very_old_plot,
            robot_rectangle, camera_rectangle,
            ouster_plot, ellipse)


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
    repeat=False, 
    cache_frame_data=False 
)

plt.show()

# Save the plot animation
with plot_writer.saving(fig, plot_video_file, dpi=100):
    for frame in frame_generator():
        update(frame)  # Update the plot
        plot_writer.grab_frame()  # Capture the current frame

# Save the closest frame animation (if displayed in a separate figure)
if current_frame_image:
    with frame_writer.saving(current_frame_image, frame_video_file, dpi=100):
        for frame in frame_generator():
            update_closest_frame(timestamps[frame])  # Update the closest frame
            frame_writer.grab_frame()  # Capture the current frame