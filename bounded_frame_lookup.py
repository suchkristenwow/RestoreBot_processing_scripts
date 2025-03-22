import numpy as np 
import os 
import glob 
from matplotlib.patches import Ellipse 

LAT_METERS_PER_DEGREE = 111320  # meters per degree latitude
LON_METERS_PER_DEGREE = 85390   # meters per degree longitude at this latitude

def precompute_timestamps(covariance_dir):
    """Precompute all available timestamps from the covariance matrix files."""
    print("precomputing timestamps ...")
    filenames = glob.glob(os.path.join(covariance_dir, "*.csv"))
    timestamps = [int(os.path.basename(f).split(".")[0]) for f in filenames]  # Convert nanoseconds to seconds
    return sorted(timestamps)

def load_covariance_matrix(timestamp,covariance_dir):
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

def create_uncertainty_ellipse(cov_matrix, center, **kwargs):
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

    return ellipse


#MAY RESULTS 
# Read the GPS data
gps_results = np.genfromtxt("May_results/results.csv", delimiter=",", skip_header=1)

# Extract timestamps, latitudes, and longitudes
timestamps = gps_results[:, 0]
filtered_latitudes = gps_results[:, 1]
filtered_longitudes = gps_results[:, 2]

# Use camera coordinates directly instead of frame center
cam_lat, cam_lon = gps_results[:, 15], gps_results[:, 16]

# Extract frame corner coordinates
top_right_lat, top_right_lon = gps_results[:, 17], gps_results[:, 18]
top_left_lat, top_left_lon = gps_results[:, 19], gps_results[:, 20]
bottom_right_lat, bottom_right_lon = gps_results[:, 21], gps_results[:, 22]
bottom_left_lat, bottom_left_lon = gps_results[:, 23], gps_results[:, 24]

available_timestamps = precompute_timestamps()  # Precompute once at the start

# Directory containing covariance matrices
may_covariance_dir = "May_results/covariance_matrices/" 

cov_matrix_t = load_covariance_matrix(t) 
###### 

#NOV RESULTS 

#### 
