uint8 MEMORY_OBSERVATION_ONLY=0
uint8 MEMORY_LOCAL_SESSION=1
uint8 MEMORY_WORLD=2

std_msgs/Header header
uint8 memory_mode
uint64 localization_epoch_id
bool mode_changed
bool epoch_changed
string<=128 canonical_frame_id
string<=128 local_frame_id
string<=128 world_frame_id
string<=128 base_frame_id
bool local_healthy
bool world_healthy
string<=256 reason
