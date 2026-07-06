# Unified perception output combining all active modules
std_msgs/Header header

# Per-object data (parallel arrays)
int32[] object_ids
string[] class_names
float32[] detection_confidences

# 3D position (always available)
geometry_msgs/Point[] positions

# 6D orientation (from FoundationPose, identity if unavailable)
geometry_msgs/Quaternion[] orientations
bool[] pose_6d_available

# Material (from MaterialRecognitionNode)
string[] materials
float32[] material_confidences

# Depth quality per object
float32[] depth_confidences

# Scene-level metadata
string scene_type               # "simple" | "cluttered" | "transparent_present"
string depth_source             # "depth_anything" | "splat" | "hybrid"
string[] active_modules         # e.g. ["yolo","sam2","material","foundation_pose"]
float32 total_vram_used_gb
