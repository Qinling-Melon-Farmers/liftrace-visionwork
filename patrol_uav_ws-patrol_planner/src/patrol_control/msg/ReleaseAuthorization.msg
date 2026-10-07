# Controller copy of the exact permission revision; no actuator authority here.
# 任务/安全层给出的短时投放许可。视觉 evidence_valid 只是输入之一。
Header header
bool permitted
uint8 payload_slot
string align_mode
uint32 target_id
string target_class
time evidence_stamp
time valid_until
string reason

# Fixed action fence. Empty mission_id retains the legacy isolated interface.
string mission_id
uint32 decision_seq
uint16 attempt
time target_first_seen

# Explicit arbiter-instance identity; Header.seq is transport-owned.
string permission_epoch
uint64 permission_revision
