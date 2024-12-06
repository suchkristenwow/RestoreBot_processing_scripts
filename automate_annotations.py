import numpy as np 

def get_range_index(ranges, value):
    for i, (start, end) in enumerate(ranges):
        if start <= value <= end:
            return i
    return None


hand_annotations = np.genfromtxt("/home/kristen/Documents/compass_annotations.csv") 
hand_annotations = hand_annotations[1:,:]

init_tf = hand_annotations[0,1] 

annotated_time_ranges = [(row[0],row[1]) for row in hand_annotations]

raw_data = np.genfromtxt("yawHeading_data.csv",delimiter=",",skip_header=1)

'''
[msg_tstamp, self.yaw, self.angularVel, heading, 
    lat, lon, 
    self.filtered_lat, self.filtered_lon, 
    self.position_covar[0], self.position_covar[1], self.position_covar[2],
    self.position_covar[3], self.position_covar[4], self.position_covar[5],
    self.position_covar[6], self.position_covar[7], self.position_covar[8]]
''' 

annotations = []
for row in raw_data:
    valid_measurement = False
    # [msg_tstamp, self.yaw, self.angularVel , heading] 
    row_tstep = row[0]
    measured_yaw = row[1]
    angular_vel = row[2]
    true_course = row[3]

    idx = get_range_index(annotated_time_ranges, row_tstep)
    if idx is None:
        continue

    annotation = hand_annotations[idx, :]
    theta_0 = annotation[2]
    theta_1 = annotation[3]

    # Check if true_course is within the range [theta_0, theta_1], considering wrap-around
    if theta_0 <= theta_1:
        # Normal range
        if theta_0 <= true_course <= theta_1:
            valid_measurement = True
    else:
        # Wrap-around range
        if true_course >= theta_0 or true_course <= theta_1:
            valid_measurement = True

    if valid_measurement:
        print("Appending measurement:", [row_tstep,true_course, measured_yaw, angular_vel])  
        annotations.append([row_tstep,true_course, measured_yaw, angular_vel])

# Save the annotations
np.savetxt("data.csv", np.array(annotations), delimiter=",")