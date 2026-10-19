# Current camera evidence for this exact swept curve; never an imagined rollout.
roomwm_msgs/FrameRef frame
string<=64 calibration_fingerprint
float64 clearance_m
float64 path_linear_mps
float64 path_angular_rps
float64 observed_half_fov_rad
bool metric_valid
bool covers_swept_footprint
uint16 person_count
bool person_visible
bool person_distance_valid
float64 person_distance_m
float64 person_bearing_rad
string<=256 path_reason
