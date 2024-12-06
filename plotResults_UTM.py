import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
from pyproj import Proj, Transformer

# Read the GPS data
gps_results = np.genfromtxt("results.csv", delimiter=",", skip_header=1)

# Extract timestamps, latitudes, and longitudes
timestamps = gps_results[:, 0]
latitudes = gps_results[:, 1]
longitudes = gps_results[:, 2]

# Automatically determine the UTM zone based on the longitude
utm_zone = int((longitudes.mean() + 180) // 6) + 1
proj_str = f"+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +north"
transformer = Transformer.from_proj("epsg:4326", proj_str, always_xy=True)

# Project latitude and longitude to UTM (x, y)
x_coords, y_coords = transformer.transform(longitudes, latitudes)

# Debug: Print a sample of transformed coordinates
print("Sample Transformed Coordinates:")
print(f"Latitude: {latitudes[0]}, Longitude: {longitudes[0]}")
print(f"UTM X: {x_coords[0]}, UTM Y: {y_coords[0]}")

# Define plot limits based on the transformed coordinates
x_min, x_max = x_coords.min() - 10, x_coords.max() + 10
y_min, y_max = y_coords.min() - 10, y_coords.max() + 10

# Create the plot
fig, ax = plt.subplots()
ax.set_xlim([x_min, x_max])
ax.set_ylim([y_min, y_max])
ax.set_aspect('equal')
plt.grid(True)

# Initialize scatter plot
scatter_plot, = ax.plot([], [], 'o', color='red', markersize=8)

# Function to update the animation frame
def update(frame):
    print(f"Frame: {frame}, X: {x_coords[frame]}, Y: {y_coords[frame]}")
    # Update the scatter plot with all points up to the current frame
    scatter_plot.set_data(x_coords[:frame + 1], y_coords[:frame + 1])
    return scatter_plot,

# Define the frames explicitly using `np.arange`
frame_indices = np.arange(len(timestamps))

# Create the animation (without blitting)
ani = animation.FuncAnimation(fig, update, frames=frame_indices, interval=50, repeat=False)

# Show the plot
plt.show()
