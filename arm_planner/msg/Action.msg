# Single action in the task queue
string action_type       # pick, place, push, pull, stack, etc.
string target_object
string destination
geometry_msgs/PoseStamped target_pose
float32 confidence
string status            # PENDING, EXECUTING, COMPLETE, FAILED
string reasoning         # why this action was chosen
string params_json       # full LLM "parameters" object (JSON) for skills that need more than object/destination
