uint8 MODE_SHADOW=0
uint8 MODE_ACTIVE=1

uint8 STATE_STARTING=0
uint8 STATE_WAITING_FOR_GESTURE=1
uint8 STATE_VALIDATING_TARGET=2
uint8 STATE_ARMING=3
uint8 STATE_FOLLOWING=4
uint8 STATE_BLOCKED=5
uint8 STATE_RC_OVERRIDE=6
uint8 STATE_FAULT=7
uint8 STATE_DISARMED=8

std_msgs/Header header
uint8 runtime_mode
uint8 state
int32 logical_target_id
bool motion_session_enabled
bool target_authorized
bool arm_request_pending
bool safety_armed
bool rc_override_active
float32 target_confidence
string reason
