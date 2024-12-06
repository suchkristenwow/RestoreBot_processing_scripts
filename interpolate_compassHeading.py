import csv
import numpy as np
from collections import deque

def normalize_angle(angle):
    """Normalize angle to the range [-180, 180]."""
    return (angle + 180) % 360 - 180

class AngularVelocityProcessor:
    def __init__(self, automated_annotations, max_angular_velocity=50.0):
        self.window = deque()
        self.angular_velocity = 0.0
        self.max_angular_velocity = max_angular_velocity
        self.error_flag = False
        self.last_annotated_time = None
        self.last_annotated_yaw = None
        self.last_annotated_heading = None 
        self.automated_annotations = automated_annotations

    def get_annotated_heading(self,t):
        idx = np.abs(self.automated_annotations[:, 0] - t).argmin()
        t_annot = self.automated_annotations[idx, 0]
        annotated_heading = self.automated_annotations[idx, 1] if abs(t_annot - t) <= 0.1 else None 
        return annotated_heading, t_annot 
    
    def update_deque(self,timestamp,heading):
        # Add the new (timestamp, yaw_deg) pair to the deque
        self.window.append((timestamp, heading))

        # Remove old entries outside the 1-second window
        while self.window and self.window[0][0] < timestamp - 1:
            self.window.popleft()

        # Calculate angular velocity if there are at least two points
        if len(self.window) > 1:
            old_time, old_yaw = self.window[0]
            new_time, new_yaw = self.window[-1]

            # Compute angular velocity
            time_delta = new_time - old_time
            if time_delta > 0:
                #print("new_yaw: {}, old_yaw: {}".format(new_yaw,old_yaw))
                yaw_diff = normalize_angle(new_yaw - old_yaw)
                #print("yaw_diff:{}, time_delta: {}".format(yaw_diff, time_delta)) 
                '''
                if new_yaw < 0:
                    new_yaw += 360
                    print("new_yaw: ",new_yaw)
                    tmp_yaw_diff = normalize_angle(new_yaw - old_yaw)
                    print("tmp_yaw_diff: ",tmp_yaw_diff)
                    if abs(tmp_yaw_diff) < abs(yaw_diff):
                        yaw_diff = tmp_yaw_diff 
                        print("changing yaw diff to: ",yaw_diff) 
                '''
            
                self.angular_velocity = yaw_diff / time_delta

                # Check if the angular velocity exceeds the maximum allowed value
                if abs(self.angular_velocity) > self.max_angular_velocity:
                    print(f"ERROR: Angular velocity exceeds maximum allowed value: {self.angular_velocity:.2f}")
                    return True 
                else:
                    return False 
                
        return False 

    def update(self, row):
        timestamp = row[0] 
        # timestamp, yaw_deg, last_annotated_yaw, last_annotated_time, last_annotated_heading 
        """Update angular velocity and process compass heading."""
        yaw_deg = normalize_angle(np.degrees(row[1]))  
        #print("yaw_deg: {}".format(yaw_deg)) 

        annotated_heading, annot_t = self.get_annotated_heading(timestamp) 

        if annotated_heading is not None:
            #print("annotated heading:",annotated_heading)
            self.last_annotated_time = annot_t 
            self.last_annotated_yaw = yaw_deg 
            self.last_annotated_heading = annotated_heading
            updated_heading = annotated_heading 
        else:
            # Process compass heading
            if self.last_annotated_time is not None and self.last_annotated_heading is not None:
                #print("last_annotated_yaw: {}, yaw_deg: {}".format(self.last_annotated_yaw,yaw_deg)) 
                yaw_diff = normalize_angle(self.last_annotated_yaw - yaw_deg)
                #print("last_annotated_heading: {}, yaw_diff: {}".format(self.last_annotated_heading,yaw_diff))
                updated_heading = normalize_angle(self.last_annotated_heading + yaw_diff)
                self.error_flag = self.update_deque(timestamp,updated_heading)
            else:
                print("in the else! this is yaw_deg: ",yaw_deg)
                updated_heading = normalize_angle(yaw_deg)  # Fallback to current yaw
                self.error_flag = True 

        return updated_heading

    def get_angular_velocity(self):
        """Get the current angular velocity."""
        return self.angular_velocity

    def has_error(self):
        """Check if an error flag is set."""
        return self.error_flag

def shortest_angular_difference(start, end):
    """
    Compute the shortest angular difference accounting for wrap-around.
    
    :param start: Starting angle (degrees).
    :param end: Ending angle (degrees).
    :return: Shortest difference to interpolate.
    """
    diff = (end - start + 180) % 360 - 180
    return diff


def smooth_error_ranges(data, error_indices):
    """
    Smooth the heading values for consecutive error ranges.
    
    :param data: List of dictionaries with timestamp and measured heading values.
    :param error_indices: List of indices where errors occurred.
    :return: Smoothed data.
    """
    smoothed_data = data.copy()
    ranges = []

    # Group consecutive error indices into ranges
    current_range = [error_indices[0]]
    for idx in error_indices[1:]:
        if idx == current_range[-1] + 1:
            current_range.append(idx)
        else:
            ranges.append(current_range)
            current_range = [idx]
    ranges.append(current_range)  # Add the last range

    print("error_ranges: ",ranges) 
    # Smooth each range
    for range_idx,error_range in enumerate(ranges):
        start_idx = error_range[0] - 1  # Index before the range
        end_idx = error_range[-1] + 1  # Index after the range

        if range_idx != len(ranges) - 1:
            next_range = ranges[range_idx + 1] 
            next_range_start = next_range[0] - 1 
            if next_range_start - end_idx < 100:
                end_idx = next_range[0]
                error_range = np.arange(start_idx,end_idx)

        print("start_idx: {}, end_idx: {}".format(start_idx,end_idx))
        if start_idx < 0 or end_idx >= len(data):
            print(f"Skipping range {error_range} due to boundary conditions.")
            continue

        start_heading = smoothed_data[start_idx]["updated_heading"]
        end_heading = smoothed_data[end_idx]["updated_heading"]
        print("start_heading: {}, end_heading: {}".format(start_heading,end_heading)) 

        angular_diff = shortest_angular_difference(start_heading,end_heading) 

        # Interpolate for the error range
        for i, idx in enumerate(error_range):
            fraction = (i + 1) / (len(error_range) + 1)
            new_heading = start_heading + fraction * angular_diff
            new_heading = (new_heading + 360) % 360  # Ensure it's in [0, 360)
            old_heading = smoothed_data[idx]['updated_heading']
            print(f"Updating heading at index {idx} from: {np.round(old_heading,2)} to {np.round(new_heading,2)}")
            smoothed_data[idx]["updated_heading"] = new_heading 

        print("Done processing this range!")
        print() 

    return smoothed_data

# Example usage
if __name__ == "__main__":
    # Example data: timestamps, yaw_deg, annotated values
    automated_annotations = np.genfromtxt("data.csv", delimiter=",")
    processor = AngularVelocityProcessor(automated_annotations)
    processed_data = []

    automated_annotations = np.genfromtxt("data.csv", delimiter=",")  # [timestamp, true_course, measured_yaw, angular_vel]
    raw_measurements = np.genfromtxt("yawHeading_data.csv", delimiter=",", skip_header=1) 

    #1650825771.194719 

    err_idx = []
    for i,row in enumerate(raw_measurements):
        row_t = row[0]; row_yaw_deg = normalize_angle(np.degrees(row[1])) 
        updated_heading = processor.update(row) 
        angular_velocity = processor.get_angular_velocity()
        error_flag = processor.has_error()

        if error_flag:
            err_idx.append(i) 
  
        processed_data.append({
            "timestamp": row_t,
            "yaw_deg": row_yaw_deg,
            "updated_heading": updated_heading,
            "angular_velocity": angular_velocity,
            "error_flag": error_flag
        }) 

    if processed_data[0]["error_flag"]:
        init_heading = 3.201870000000000118e+02 
        processed_data[0]["updated_heading"] = init_heading 
        processed_data[0]["error_flag"] = 0 
        
    processed_data = smooth_error_ranges(processed_data,err_idx) 

    # Write processed data to CSV
    with open("processed_compass_heading.csv", "w", newline="") as csvfile:
        # Define only the fields you want to write
        fieldnames = ["timestamp", "updated_heading"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        # Filter data to include only the desired fields
        writer.writerows({"timestamp": row["timestamp"], "updated_heading": row["updated_heading"]} for row in processed_data)
