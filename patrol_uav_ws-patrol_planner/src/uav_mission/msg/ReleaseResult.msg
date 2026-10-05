# 对旧 Servo 服务调用的审计结果。
Header header
uint32 execution_id
uint8 payload_slot
string align_mode
uint32 target_id
string target_class
bool success
string reason

# RAW_CALL_STARTED is conservative: bool raw cannot prove physical PWM began.
uint8 EXECUTION_UNKNOWN=0
uint8 NOT_STARTED=1
uint8 RAW_CALL_STARTED=2
uint8 COMPLETED=3
uint8 execution_state
bool terminal
string mission_id
uint32 decision_seq
uint16 attempt
time target_first_seen
time evidence_stamp
