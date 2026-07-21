uint8 INTENT_NONE=0
uint8 INTENT_ROTATE_VERIFY=1
uint8 INTENT_STOP=2

std_msgs/Header header
uint64 query_id
uint8 intent
float32 target_bearing
float32 maximum_rotation_angle
float32 maximum_angular_speed
builtin_interfaces/Time deadline
bool rotation_permitted
bool forward_permitted
string reason
