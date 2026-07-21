uint8 PROPOSAL_CAMERA_DETECTOR=0
uint8 PROPOSAL_OPEN_VOCABULARY=1
uint8 PROPOSAL_LANGUAGE_PATCH=2
uint8 PROPOSAL_PATCH_GROUP=3
uint8 PROPOSAL_MANUAL_ROI=4
uint8 PROPOSAL_LIDAR_PROJECTED=5
uint8 PROPOSAL_EXTERNAL_DETECTOR=6

uint8 MASK_NONE=0
uint8 MASK_PNG=1
uint8 MASK_COCO_RLE_JSON=2

std_msgs/Header header
uint64 producer_epoch_id
uint64 proposal_id
uint32 image_width
uint32 image_height
sensor_msgs/RegionOfInterest roi
uint8 mask_encoding
uint8[<=65536] compressed_mask
uint8 proposal_source
float32 detector_confidence
track_robot_interfaces/SemanticLabelEvidence[<=16] semantic_labels
