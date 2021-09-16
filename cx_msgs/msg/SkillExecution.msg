int16 REQUEST=1
int16 RESPONSE=2
int16 CONFIRM=3
int16 REJECT=4
int16 FEEDBACK=5
int16 FINISH=6
int16 CANCEL=7

int16 type
string node_id

string action
string[] action_parameters
# needed for fawkes skiller or config mapped action
string mapped_action

string status
bool success
float32 progress