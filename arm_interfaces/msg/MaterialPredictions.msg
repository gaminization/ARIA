# Per-object material predictions
std_msgs/Header header
int32[] object_ids
string[] material_classes          # e.g. "glass", "metal", "plastic"
float32[] confidences
float32[] grip_force_scales        # multiplier for default grip force
float32[] approach_speed_scales    # multiplier for default approach speed
string[] special_notes             # per-object notes e.g. "fragile"
