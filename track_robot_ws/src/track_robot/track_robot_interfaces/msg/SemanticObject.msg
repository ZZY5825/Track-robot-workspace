uint8 MEMORY_OBSERVATION_ONLY=0
uint8 MEMORY_LOCAL_SESSION=1
uint8 MEMORY_WORLD=2

uint8 LIFECYCLE_TENTATIVE=0
uint8 LIFECYCLE_CONFIRMED=1
uint8 LIFECYCLE_STALE=2
uint8 LIFECYCLE_LOST=3
uint8 LIFECYCLE_ARCHIVED=4

uint8 SUPPORT_NONE=0
uint8 SUPPORT_CAMERA_LIDAR=1
uint8 SUPPORT_CAMERA_ONLY=2
uint8 SUPPORT_LIDAR_ONLY=3
uint8 SUPPORT_PREDICTION_ONLY=4

uint8 VISIBILITY_VISIBLE=0
uint8 VISIBILITY_OCCLUDED=1
uint8 VISIBILITY_OUT_OF_FOV=2
uint8 VISIBILITY_UNKNOWN=3

uint8 MOTION_STATIC=0
uint8 MOTION_DYNAMIC=1
uint8 MOTION_UNCERTAIN=2
uint8 MOTION_TEMPORARILY_MOVING=3

uint8 REID_NOT_REQUIRED=0
uint8 REID_PENDING=1
uint8 REID_CONFIRMED=2
uint8 REID_REJECTED=3

uint8 INSPECTION_NOT_INSPECTED=0
uint8 INSPECTION_REQUESTED=1
uint8 INSPECTION_COMPLETE=2

std_msgs/Header header
uint64 memory_epoch_id
uint64 global_object_id

bool camera_track_id_valid
uint64 camera_source_epoch_id
int64 camera_track_id
bool visual_candidate_id_valid
uint64 visual_producer_epoch_id
uint64 visual_candidate_id
bool lidar_tracklet_id_valid
uint64 lidar_source_epoch_id
int64 lidar_tracklet_id

uint8 memory_mode
uint64 localization_epoch_id
string<=128 position_frame_id
bool position_valid
geometry_msgs/Point position
bool velocity_valid
geometry_msgs/Vector3 velocity
bool extent_valid
geometry_msgs/Vector3 extent
float32[9] position_covariance

uint8 lifecycle_state
uint8 support_state
uint8 visibility_state
uint8 motion_class
uint8 reidentification_state
uint8 inspection_state

track_robot_interfaces/SemanticLabelEvidence[<=16] semantic_labels
string<=512 semantic_description
string<=128 appearance_summary_id
uint8 appearance_prototype_count
string<=128 appearance_encoder_id
string<=128 appearance_checkpoint_id
uint32 appearance_descriptor_version

uint64 active_query_id
uint64 active_query_version
float32 task_relevance
float32 inspection_priority

builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
builtin_interfaces/Time last_camera_seen
builtin_interfaces/Time last_lidar_seen
uint32 observation_count
uint32 camera_observation_count
uint32 lidar_observation_count
track_robot_interfaces/SemanticObjectHistorySample[<=16] short_history

float32 association_confidence
float32 fusion_confidence
float32 uncertainty
float32 duplicate_score
bool anomaly_score_valid
float32 anomaly_score
