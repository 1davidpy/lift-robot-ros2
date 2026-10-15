# One SDK frameset after depth-to-color alignment. Not hardware synchronization.
std_msgs/Header header
string camera_id
# Unique within this camera process session, opaque to consumers.
string pair_id
uint64 sequence
uint64 depth_device_sequence
uint64 color_device_sequence
CameraFrame color
CameraFrame depth
sensor_msgs/CameraInfo camera_info
# Original SDK timestamps remain in each CameraFrame's named clock domain.
