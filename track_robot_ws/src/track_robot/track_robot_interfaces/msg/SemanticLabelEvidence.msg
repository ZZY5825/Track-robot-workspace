uint8 EVIDENCE_PERMANENT=0
uint8 EVIDENCE_TASK_CONDITIONED=1
uint8 EVIDENCE_DETECTOR=2
uint8 EVIDENCE_OPERATOR=3

string<=128 label
float32 confidence
string<=128 provenance
uint8 evidence_kind
uint64 source_observation_id
