# SAM2 per-object segmentation masks
std_msgs/Header header
int32[] object_ids
sensor_msgs/Image[] masks          # binary mono8, one per object
float32[] confidences
string[] class_names
float32[] mask_areas               # pixel area of each mask
