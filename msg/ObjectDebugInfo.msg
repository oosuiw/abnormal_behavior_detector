# 객체별 상세 디버그 정보 토픽
# 로그 대신 토픽으로 확인

std_msgs/Header header

# 객체 기본 정보
string stable_object_id           # 위치 기반 stable ID (예: grid_3872_30304)
string uuid                       # 원본 UUID (앞 8자리)
geometry_msgs/Point position      # 객체 위치
float64 yaw_rad                   # Yaw (라디안)
float64 yaw_deg                   # Yaw (도)

# Lanelet 매칭 정보
bool lanelet_matched              # Lanelet 매칭 성공 여부
bool is_on_lanelet                # 객체가 Lanelet 위에 있는지 (occupancy)
int64 matched_lanelet_id          # 매칭된 Lanelet ID
float64 lanelet_distance          # Lanelet까지 거리 (m)
uint32 nearby_lanelets_count      # 주변 Lanelet 개수

# 방향 벡터 정보
geometry_msgs/Vector3 object_heading_vector      # 객체 heading 벡터 (x, y, 0)
geometry_msgs/Vector3 lanelet_direction_vector   # Lanelet 방향 벡터 (x, y, 0)

# 각도 정보
float64 dot_product               # 내적 값
float64 angle_diff_deg            # 각도 차이 (도)
float64 wrong_way_threshold_deg   # 역주행 임계값 (도)

# 검출 결과
string behavior_type              # NORMAL / WRONG_WAY / OVER_SPEED / UNDER_SPEED / ABNORMAL_STOP
float64 confidence                # 신뢰도 (0.0 ~ 1.0)
string description                # 설명
int32 consecutive_count           # 연속 검출 횟수
bool is_confirmed                 # 확정 여부 (연속 임계값 도달)

# 속도 정보
float64 speed_ms                  # 속도 (m/s)
float64 speed_kmh                 # 속도 (km/h)
float64 speed_limit_ms            # 제한 속도 (m/s)
float64 speed_limit_kmh           # 제한 속도 (km/h)
