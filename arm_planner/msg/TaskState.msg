# Current task execution state
string current_command
string current_goal
string[] subgoals
arm_planner/Action[] action_queue
string[] chain_of_thought
arm_planner/FailureEvent[] failure_log
float32 confidence
bool awaiting_user_approval
string task_status             # IDLE/PLANNING/EXECUTING/PAUSED/COMPLETE/FAILED
string task_id
