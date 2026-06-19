# Failure event for logging and recovery
string failure_type            # MISSED_OBJECT, IK_FAILURE, etc.
string cause
string recovery_attempted
bool recovery_succeeded
builtin_interfaces/Time timestamp
string action_at_failure
