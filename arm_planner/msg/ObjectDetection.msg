# Single object detection with 3D pose and affordances
int32 tracking_id
string class_name
float32 confidence
geometry_msgs/PoseStamped pose_3d
string lifecycle_state         # Detected/Tracked/Lost/Recovered/Moved/Removed
string[] affordance_regions    # e.g. ["handle", "rim", "base"]
float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h
