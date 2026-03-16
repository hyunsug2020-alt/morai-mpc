# 드라이브 모드 메시지
# 주행 모드 전환 상태 정보

std_msgs/Header header

# 현재 모드 (문자열)
# "FORWARD"  : 전진 주행 → LTV-MPC
# "REVERSE"  : 후진 주행 → RTI-NMPC
# "PARKING"  : 주차 모드 → Hybrid A* + RTI-NMPC
# "IDLE"     : 정지 대기
string mode

# 모드 전환 이유
string reason

# 타겟 주차구역 ID (-1 = 없음)
int32 target_spot_id

# 현재 주차 진행률 [0.0, 1.0]
float64 parking_progress

# 주차 경로가 계획되었는지
bool path_planned
