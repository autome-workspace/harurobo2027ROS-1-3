# Optical frame. Ray is (right, down, forward); stamp is the image stamp.
std_msgs/Header header
# Image detection is independent of calibration. An unavailable ray has z=0.
bool detected
float64 u
float64 v
float64 area
float64 diameter
float64 angular_radius
geometry_msgs/Vector3 ray
