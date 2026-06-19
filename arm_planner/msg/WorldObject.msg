# Persistent world object representation
int32 id
string name
string class_name
geometry_msgs/PoseStamped last_known_pose
builtin_interfaces/Time last_seen
string color
string material
string lifecycle_state
string[] spatial_relations     # "is_left_of:box", "is_inside:shelf"
float32[] affordance_scores    # per affordance region
