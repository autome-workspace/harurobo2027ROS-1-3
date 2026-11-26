# Base frame. Header stamp is the scan reference time.
std_msgs/Header header
builtin_interfaces/Time image_stamp
bool valid
# True when the estimate is based on an existing LiDAR lock.
bool tracking
geometry_msgs/Point center
float64 surface_distance
float64 scan_range_min
float64 ball_radius
geometry_msgs/Point32[] obstacles
