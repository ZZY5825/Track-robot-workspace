uint8 IDENTITY_NONE=0
uint8 IDENTITY_CONFIRMED=1
uint8 IDENTITY_AMBIGUOUS=2
uint8 IDENTITY_OCCLUDED=3

uint8 LOCK_NO_TARGET=0
uint8 LOCK_CANDIDATE_VISIBLE=1
uint8 LOCK_TARGET_LOCKED=2
uint8 LOCK_TARGET_LOST=3

std_msgs/Header header
int32 logical_target_id
int32 visual_track_id
uint8 lock_state
uint8 identity_state
float32[4] bbox
float32[] keypoints
float32 detector_confidence
float32 identity_confidence
bool camera_visible
