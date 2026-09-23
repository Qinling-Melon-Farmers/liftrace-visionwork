uint8 SCHEMA_VERSION=1

uint8 SEARCH=0
uint8 APPROACH=1
uint8 ALIGN=2
uint8 RESUME=3
uint8 RETURN_HOME=4
uint8 LAND=5
uint8 HOLD=6
uint8 ABORT=7

uint8 ACCEPTED=0
uint8 STARTED=1
uint8 PROGRESS=2
uint8 SUCCEEDED=3
uint8 FAILED=4
uint8 REJECTED=5
uint8 CANCELLED=6
uint8 TIMED_OUT=7

uint8 DISPATCH=0
uint8 PLANNER=1
uint8 CAPTURE=2
uint8 ALIGNMENT=3
uint8 RELEASE=4
uint8 RECOVERY=5
uint8 LANDING=6

std_msgs/Header header
uint8 schema_version
string mission_id
string executor_id
uint32 event_seq
uint32 decision_seq
uint8 command
uint8 status
uint8 stage
bool terminal
bool retryable
bool payload_committed
bool has_target
uint32 target_id
time target_first_seen
string target_class
uint16 attempt
uint8 payload_slot
string reason
string evidence_source
