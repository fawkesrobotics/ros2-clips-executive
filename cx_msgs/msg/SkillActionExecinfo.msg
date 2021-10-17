# named after the clips deftemplate skill-action-execinfo
int8 S_IDLE=0
int8 S_RUNNING=1
int8 S_FINAL=2
int8 S_FAILED=3

int8 status

string action
string[] action_parameters
string mapped_action
string agent_id

string skill_id
string string_status
string error_msg

builtin_interfaces/Time start_stamp
builtin_interfaces/Time status_stamp

