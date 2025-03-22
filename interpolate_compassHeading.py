import csv
import numpy as np
from collections import deque

def normalize_angle(angle):
    """Normalize angle to the range [-180, 180]."""
    return (angle + 180) % 360 - 180

class AngularVelocityProcessor:
    def __init__(self, automated_annotations, manual_annotations, max_angular_velocity=50.0):
        self.window = deque()
        self.angular_velocity = 0.0
        self.max_angular_velocity = max_angular_velocity
        self.error_flag = False
        self.last_annotated_time = None
        self.last_annotated_yaw = None
        self.last_annotated_heading = None 
        self.automated_annotations = automated_annotations
        self.manual_annotations = manual_annotations

    def get_annotated_heading(self,t):
        idx = np.abs(self.automated_annotations[:, 0] - t).argmin()
        t_annot = self.automated_annotations[idx, 0]
        print("t: {}, t_annot: {}, delta_t: {}".format(t,t_annot,abs(t_annot - t)))
        annotated_heading = self.automated_annotations[idx, 1] if abs(t_annot - t) <= 0.3 else None 

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

                yaw_diff = normalize_angle(new_yaw - old_yaw) 

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
        print("yaw_deg: {}".format(yaw_deg)) 

        annotated_heading, annot_t = self.get_annotated_heading(timestamp) 
        print("annotated_heading: {}, annot_t: {}".format(annotated_heading,annot_t)) 

        if annotated_heading is not None:
            print("annotated heading:",annotated_heading)
            self.last_annotated_time = annot_t 
            self.last_annotated_yaw = yaw_deg 
            self.last_annotated_heading = annotated_heading
            self.error_flag = self.update_deque(timestamp,annotated_heading) 
            updated_heading = annotated_heading 
            uncertainty_radians = 1.571
        else:
            # Process compass heading
            if self.last_annotated_time is not None and self.last_annotated_heading is not None:
                print("last_annotated_yaw: {}, yaw_deg: {}".format(self.last_annotated_yaw,yaw_deg)) 
                yaw_diff = normalize_angle(self.last_annotated_yaw - yaw_deg)
                print("last_annotated_heading: {}, yaw_diff: {}".format(self.last_annotated_heading,yaw_diff))
                updated_heading = normalize_angle(self.last_annotated_heading + yaw_diff)
                self.error_flag = self.update_deque(timestamp,updated_heading) 
                uncertainty_radians = 2.356
            else:
                print("in the else! this is yaw_deg: ",yaw_deg)
                updated_heading = normalize_angle(yaw_deg)  # Fallback to current yaw
                self.error_flag = True 
                uncertainty_radians = np.pi 

        if not 0 < updated_heading < 360:
            print("normalizing angle ... ", updated_heading)
            updated_heading = updated_heading % 360 
            print("this is normalized angle: ",updated_heading) 

        print("before checking manual annotations .... updated_heading: ",updated_heading)

        for annotation_row in self.manual_annotations: 
            #print("timestamp: ",timestamp) 
            #print("range start: {}, range_end: {}".format(annotation_row[0],annotation_row[1])) 
            if annotation_row[0] <= timestamp <= annotation_row[1]: 
                if annotation_row[2] > annotation_row[3]: 
                    if annotation_row[2] < updated_heading or annotation_row[3] > updated_heading:
                        if not (0 <= updated_heading <= annotation_row[3] or annotation_row[2] < updated_heading) < 360:
                            updated_heading = circular_mean([annotation_row[2],annotation_row[3]])
                            uncertainty_radians = np.deg2rad(100)
                            input("WARNING")
                else: 
                    if annotation_row[2] > updated_heading or annotation_row[3] < updated_heading:
                        updated_heading = circular_mean([annotation_row[2],annotation_row[3]])
                        uncertainty_radians = np.deg2rad(100)
                        input("WARNING")
        print() 
        return updated_heading, uncertainty_radians 

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
            print("next_range: ",next_range)
            next_range_start = next_range[0] - 1
            print("next_range_start: {}, end_idx: {}".format(next_range_start,end_idx)) 
            if next_range_start - end_idx < 100:
                if next_range[-1] - next_range[0] < 10:
                    print("difference cleared, appending the new range")
                    end_idx = next_range[-1] 
                else: 
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
            new_heading = new_heading % 360 
            old_heading = smoothed_data[idx]['updated_heading']
            print(f"Updating heading at index {idx} from: {np.round(old_heading,2)} to {np.round(new_heading,2)}")
            smoothed_data[idx]["updated_heading"] = new_heading 

        print("Done processing this range!") 
        print() 

    return smoothed_data

def get_corresponding_idx(data,t): 
    for i,d in enumerate(data): 
        if t - 0.3 <= d["timestamp"] <= t + 0.3:
            return i 

def get_cardinal_direction(heading): 
    if 0 <= heading < 22:
        return 'N'
    if 22 <= heading < 67:
        return 'NE' 
    if 67 <= heading < 112:
        return 'E' 
    if 112 <= heading < 157:
        return 'SE' 
    if 157 <= heading < 202:
        return 'S'
    if 202 <= heading < 247:
        return'SW' 
    if 247 <= heading < 292:
        return 'W' 
    if 292 <= heading < 337:
        return 'NW' 
    if 337 <= heading < 360:
        return 'N' 
    
def get_turn_directions(range1,range2): 
    clockwise_order = ['N','NE','E','SE','S','SW','W','NW'] 
    counter_clockwise_order = list(reversed(clockwise_order))
    turn_directions = [] 
    range1_theta0 = range1[2]; range1_theta1 = range1[3]
    init_cardinal_direction = get_cardinal_direction(range1_theta0)
    cardinal_idx0 = clockwise_order.index(init_cardinal_direction)
    range2_theta0 = range2[2]; range2_theta1 = range2[3] 
    final_cardinal_direction = get_cardinal_direction(range2_theta1) 
    cardinal_idx1 = clockwise_order.index(final_cardinal_direction) 
    headings = [range1_theta0,range1_theta1,range2_theta0,range2_theta1]
    if any(v < 0 or 360 <= v for v in headings):
        raise OSError 
    mean_range1_heading = np.mean([range1_theta0,range1_theta1]) 
    mean_range2_heading = np.mean([range2_theta0,range2_theta1]) 
    print("mean_range1_heading: {}, mean_range2_heading: {}".format(mean_range1_heading,mean_range2_heading))
    if mean_range1_heading < mean_range2_heading: 
        print("turning clockwise...")
        #turning clockwise 
        if cardinal_idx1 >= cardinal_idx0 and cardinal_idx1 + 1 < len(clockwise_order):
            turn_directions = clockwise_order[cardinal_idx0:cardinal_idx1+1]
        else:
            turn_directions = clockwise_order[cardinal_idx0:] + clockwise_order[:cardinal_idx1+1]
    else: 
        print("turning counter-clockwise...")
        #turning counter-clockwise
        cardinal_idx0 = counter_clockwise_order.index(init_cardinal_direction) 
        cardinal_idx1 = counter_clockwise_order.index(final_cardinal_direction) 
        if cardinal_idx1 >= cardinal_idx0 and cardinal_idx1 + 1 < len(counter_clockwise_order):
            turn_directions = counter_clockwise_order[cardinal_idx0:cardinal_idx1+1] 
        else: 
            turn_directions = counter_clockwise_order[cardinal_idx0:] + clockwise_order[:cardinal_idx1+1] 
    print("turn_directions:",turn_directions)
    return turn_directions 

def get_adjacent_directions(turn_directions):
    cardinal_directions = ["N","NE","E","SE","S","SW","W","NW"]  
    direction0 = turn_directions[0]; direction1 = turn_directions[-1] 
    #print("direction0:{}, direction1: {}".format(direction0,direction1))
    idx0 = cardinal_directions.index(direction0) 
    idx1 = cardinal_directions.index(direction1)
    #print("idx0: {}, idx1: {}".format(idx0,idx1))
    adjacent_directions = []
    if idx0 < idx1: 
        adjacent_directions.append(cardinal_directions[idx1-1])
        if idx0 + 1 < len(cardinal_directions): 
            adjacent_directions.append(cardinal_directions[idx0+1]) 
        else: 
            adjacent_directions.append(cardinal_directions[0])
    else: 
        #print("appending {} to adjacent directions".format(cardinal_directions[idx0-1]))
        adjacent_directions.append(cardinal_directions[idx0-1])
        if idx1 + 1 < len(cardinal_directions): 
            #print("in the if ... appending {} to adjacent directions".format(cardinal_directions[idx1+1]))
            adjacent_directions.append(cardinal_directions[idx1+1]) 
        else: 
            #print("in the else ... appending {} to adjacent directions".format(cardinal_directions[0]))
            adjacent_directions.append(cardinal_directions[0])
    return adjacent_directions

def generate_evenly_spaced_values(val1, val2, n):
    start, end = sorted([val1, val2])  # Ensure ascending order
    values = np.linspace(start, end, n+1)[1:]  # Exclude the first value
    return values[::-1] if val1 > val2 else values  # Reverse if val1 > val2

def get_inst_angular_vel(measurement1,measurement2): 
    theta_current = measurement2['updated_heading']
    theta_prev = measurement1['updated_heading']
    delta_t = measurement2['timestamp'] - measurement1['timestamp']
    delta_theta = (theta_current - theta_prev + 180)%360 - 180 
    omega = delta_theta/delta_t 
    return omega 

def hand_fix_ranges(processed_data,manual_annotations,still_ranges=None): 
    ranges2fix = [(0,1650824667),(1650824667,1650824695)]
    for j,range_ in enumerate(ranges2fix):
        keys = [i for i,x in enumerate(processed_data) if range_[0] < x['timestamp'] < range_[1]]
        sorted_keys = sorted(keys, key=lambda x: processed_data[x]['timestamp']) 
        prev_heading_estimate = processed_data[sorted_keys[0] - 1]['updated_heading'] 
        prev_measurement_t = processed_data[sorted_keys[0] - 1]['timestamp'] 
        for idx in range(sorted_keys[0],sorted_keys[-1] + 1):
            original_heading_estimate = processed_data[idx]['updated_heading'] 
            measurement_t_idx = processed_data[idx]['timestamp'] 
            is_still = False 
            if still_ranges is not None:
                for range_ in still_ranges: 
                    if range_[0] <= measurement_t_idx <= range_[1]:
                        is_still = True 
                        break 
            if is_still:
                continue 
            ang_vel_idx = processed_data[idx]['angular_velocity']  
            actual_delta_theta = get_inst_angular_vel(processed_data[idx-1],processed_data[idx]) 
            print("actual_delta_theta (orig):",actual_delta_theta)
            print("measured angular velocity:",ang_vel_idx)
            delta_t = measurement_t_idx - prev_measurement_t 
            new_heading_estimate = (prev_heading_estimate + ang_vel_idx*delta_t) % 360
            if check_annotations(manual_annotations,measurement_t_idx,new_heading_estimate):
                print("original_heading: {}, new_heading: {}".format(original_heading_estimate,new_heading_estimate)) 
                processed_data[idx]['updated_heading'] = new_heading_estimate
            #update prev heading estimate and t 
            prev_heading_estimate = processed_data[idx]['updated_heading'] 
            prev_measurement_t = processed_data[idx]['timestamp']
            print()
    return processed_data

import numpy as np

def generate_evenly_spaced_headings(theta0, theta1, num_points, clockwise=None):
    """
    Generates evenly spaced compass headings between theta0 and theta1,
    considering wraparound at 360 degrees and allowing for direction selection.

    :param theta0: Initial heading (0-359 degrees).
    :param theta1: Final heading (0-359 degrees).
    :param num_points: Number of evenly spaced headings to generate.
    :param clockwise: Boolean specifying whether to go clockwise (True) or counterclockwise (False).
                      If None, chooses the shortest path.
    :return: List of evenly spaced compass headings (in degrees).
    """
    theta0 = theta0 % 360
    theta1 = theta1 % 360

    if theta0 == theta1:
        return [theta0] * num_points  # Edge case: start and end are the same

    # Compute clockwise and counterclockwise distances
    clockwise_dist = (theta1 - theta0) % 360
    counterclockwise_dist = (theta0 - theta1) % 360

    # Choose shortest path if `clockwise` is None
    if clockwise is None:
        clockwise = clockwise_dist <= counterclockwise_dist

    # Generate points in the selected direction
    if clockwise:
        headings = np.linspace(theta0, theta0 + clockwise_dist, num_points) % 360
    else:
        headings = np.linspace(theta0, theta0 - counterclockwise_dist, num_points) % 360

    return list(headings)


def enforce_stillness(t_ranges,data,manual_annotations):
    for annotation_row in manual_annotations:
        t0_annot = annotation_row[0]; tf_annot = annotation_row[1]
        for range_ in t_ranges: 
            if t0_annot <= range_[0] and range_[1] <= tf_annot: 
                #want to get the corresponding keys
                keys = [i for i,x in enumerate(data) if range_[0] < x['timestamp'] < range_[1]]
                sorted_keys = sorted(keys, key=lambda x: data[x]['timestamp']) 
                updated_heading = data[sorted_keys[0]]['updated_heading']
                if annotation_row[2] > annotation_row[3]: 
                    if annotation_row[2] < updated_heading or annotation_row[3] > updated_heading:
                        if not (0 <= updated_heading <= annotation_row[3] or annotation_row[2] < updated_heading) < 360:
                            raise OSError 
                else: 
                    if annotation_row[2] > updated_heading or annotation_row[3] < updated_heading:
                        raise OSError 
                for key in sorted_keys: 
                    data[key]['updated_heading'] = updated_heading 
    return data

def is_heading_in_range(x, min_theta, max_theta):
    """
    Checks if the given heading x falls between min_theta and max_theta,
    accounting for angle wraparound at 360 degrees.

    :param x: The compass heading to check (0-359 degrees).
    :param min_theta: The lower bound of the range (0-359 degrees).
    :param max_theta: The upper bound of the range (0-359 degrees).
    :return: True if x is within the range, False otherwise.
    """
    # Normalize angles to be within [0, 360)
    x = x % 360
    min_theta = min_theta % 360
    max_theta = max_theta % 360

    if min_theta <= max_theta:
        # Normal case: range does not wrap around 360
        return min_theta <= x <= max_theta
    else:
        # Wrapped case: range spans across 0 degrees
        return x >= min_theta or x <= max_theta

def get_closest_idx(timestamp,processed_data): 
    closest_key = None
    min_delta = float('inf')

    for k,_ in enumerate(processed_data):
        if abs(timestamp - processed_data[k]['timestamp']) < min_delta:
            min_delta = abs(timestamp - processed_data[k]['timestamp'])
            closest_key = k 
    
    return closest_key

def circular_mean(headings):
    """
    Computes the circular mean of a list of compass headings (0-360 degrees),
    taking into account wraparound at 360 degrees.

    :param headings: List of headings (angles in degrees).
    :return: Circular mean heading (in degrees).
    """
    if not headings:
        raise ValueError("List of headings is empty")

    # Convert angles to unit vectors
    sum_sin = sum(np.sin(np.radians(h)) for h in headings)
    sum_cos = sum(np.cos(np.radians(h)) for h in headings)

    # Compute mean angle from vector sum
    mean_angle_rad = np.arctan2(sum_sin, sum_cos)
    
    # Convert back to degrees and normalize to [0, 360)
    mean_angle_deg = np.degrees(mean_angle_rad) % 360

    return mean_angle_deg

def enforce_straight_paths(straight_ranges,processed_data,hand_annotations,known_heading=None):
        for row_heading in straight_ranges: 
            range_ = straight_ranges[row_heading]
            print("range_:",range_)
            t0_range = range_[0]; tf_range = range_[1] 
            straight_idx = [i for i,_ in enumerate(processed_data) if t0_range <= processed_data[i]['timestamp'] <= tf_range] 
            idx_range_len = len(straight_idx) 

            int_idx = int(idx_range_len*.75)
   
            for i in straight_idx[:int_idx]: 
                processed_data[i]['updated_heading'] = row_heading 
            
            #interpolate smooth range to next measurement 
            idx = get_closest_idx(tf_range + 0.2,processed_data) 
            next_measurement = processed_data[idx]['updated_heading']
            evenly_spaced_headings = generate_evenly_spaced_headings(row_heading,next_measurement,len(straight_idx[int_idx:])) 
            print("int_idx:",int_idx) 
            print("len(evenly_spaced_headings):",len(evenly_spaced_headings))
            print(len(straight_idx[int_idx:]))

            for i,j in enumerate(straight_idx[int_idx:]):
                processed_data[j]['updated_heading'] = evenly_spaced_headings[i]

        return processed_data 

def check_annotations(manual_annotations,timestamp,updated_heading):
    for annotation_row in manual_annotations: 
        if annotation_row[0] <= timestamp <= annotation_row[1]: 
            if annotation_row[2] > annotation_row[3]: 
                if annotation_row[2] < updated_heading or annotation_row[3] > updated_heading:
                    if not (0 <= updated_heading <= annotation_row[3] or annotation_row[2] < updated_heading) < 360:
                        return False 
            else: 
                if annotation_row[2] > updated_heading or annotation_row[3] < updated_heading:
                    return False  
    return True 
                    
if __name__ == "__main__":
    # Example data: timestamps, yaw_deg, annotated values
    automated_annotations = np.genfromtxt("May_results/2drill/data.csv", delimiter=",")
    hand_annotations = np.genfromtxt("/home/kristen/Documents/2drill_compass_annotations.csv",delimiter=",",skip_header=True) 

    processor = AngularVelocityProcessor(automated_annotations,hand_annotations)
    processed_data = []

    automated_annotations = np.genfromtxt("May_results/2drill/data.csv", delimiter=",")  # [timestamp, true_course, measured_yaw, angular_vel]
    raw_measurements = np.genfromtxt("May_results/2drill/yawHeading_data.csv", delimiter=",", skip_header=1) 

    err_idx = []; err_tsteps = []
    for i,row in enumerate(raw_measurements):
        row_t = row[0]; row_yaw_deg = normalize_angle(np.degrees(row[1])) 
        print("row_t: ",row_t)

        updated_heading,uncertainty_radians = processor.update(row) 
        angular_velocity = processor.get_angular_velocity()
        error_flag = processor.has_error()
        print("updated_heading: ",updated_heading) 

        if error_flag:
            err_tsteps.append(row_t)
            err_idx.append(i) 
  
        processed_data.append({
            "timestamp": row_t,
            "yaw_deg": row_yaw_deg,
            "updated_heading": updated_heading,
            "angular_velocity": angular_velocity,
            "uncertainty_radians":uncertainty_radians,
            "error_flag": error_flag
        }) 

    if len(err_idx) > 0: 
        print("error_tsteps: ",err_tsteps)
        processed_data = smooth_error_ranges(processed_data,err_idx) 
    
    for measurement in processed_data: 
        measurement["updated_heading"] = measurement["updated_heading"] % 360

    turn0 = 1650825186.5; turn1 = 1650825187
    turn_idx = [i for i,x in enumerate(processed_data) if turn0 < processed_data[i]['timestamp'] <= turn1] 
    sorted_idx = sorted(turn_idx, key=lambda x: processed_data[x]['timestamp']) 
    first_measurement_idx = get_closest_idx(turn0,processed_data) 
    first_measurement_heading = processed_data[0]['updated_heading'] 
    last_measurement_idx = get_closest_idx(turn1,processed_data) 
    last_measurement_heading = processed_data[last_measurement_idx]['updated_heading'] 
    print("first_measurement heading:",first_measurement_heading) 
    print("last_measurement_heading: ",last_measurement_heading) 
    print("num points:",last_measurement_idx - first_measurement_idx + 1)
    input() 
    turn_headings = generate_evenly_spaced_headings(first_measurement_heading,last_measurement_heading,last_measurement_idx - first_measurement_idx + 1,clockwise=False)  
    for i,j in enumerate(range(first_measurement_idx,last_measurement_idx+1)):
        processed_data[j]['updated_heading'] = turn_headings[i]

    init_idx = [i for i,x in enumerate(processed_data) if 0 <= processed_data[i]['timestamp'] <= 1650825187.8] 
    sorted_idx = sorted(init_idx, key=lambda x: processed_data[x]['timestamp']) 

    for idx in sorted_idx:
        if not is_heading_in_range(processed_data[idx]['updated_heading'],292,22):
            processed_data[idx]['updated_heading'] = first_measurement_heading

    #make whatever fixes 
    still_ranges = [(0,1650825183)]
    processed_data = enforce_stillness(still_ranges,processed_data,hand_annotations) 

    # Write processed data to CSV
    with open("May_results/2drill/processed_compass_heading.csv", "w", newline="") as csvfile:
        # Define only the fields you want to write
        fieldnames = ["timestamp", "updated_heading","uncertainty"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        # Filter data to include only the desired fields
        writer.writerows({"timestamp": row["timestamp"], "updated_heading": row["updated_heading"], "uncertainty":row["uncertainty_radians"]} for row in processed_data)
