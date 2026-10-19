# Controller service invocation event, not a physical actuator ACK.
# In no-ACK recovery policy, this consumes one slot and starts recovery.
std_msgs/Header header
uint64 request_id
string mission_id
uint32 decision_seq
uint16 attempt
uint8 payload_slot
uint32 target_id
time target_first_seen
string target_class
string align_mode
