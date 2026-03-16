# 주차구역 감지 메시지
# 카메라로 감지한 흰색 사각형 주차선 정보

std_msgs/Header header

# 주차구역 중심 자세 (map 프레임)
geometry_msgs/Pose spot_pose

# 주차구역 크기 [m]
float64 width   # 너비 (차량 진입 방향 기준)
float64 length  # 길이 (차량 길이 방향)

# 감지 신뢰도 [0.0, 1.0]
float64 confidence

# 주차구역 ID (여러 구역 감지시)
int32 spot_id

# 감지 상태
bool is_occupied   # 이미 다른 차량이 점유 중
bool is_valid      # 유효한 주차구역인지
