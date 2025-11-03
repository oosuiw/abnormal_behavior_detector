# 이상 거동 전체 상태 토픽
# 현재 이상 거동 차량이 있는지 간단히 확인

std_msgs/Header header

# 전체 상태
bool has_abnormal_behavior        # 이상 거동 차량이 있는지 여부
uint32 total_objects              # 전체 객체 수
uint32 abnormal_objects_count     # 이상 거동 객체 수

# 이상 거동 타입별 카운트
uint32 wrong_way_count            # 역주행 차량 수
uint32 over_speed_count           # 과속 차량 수
uint32 under_speed_count          # 저속 차량 수
uint32 abnormal_stop_count        # 비정상 정차 차량 수

# 처리 시간 (ms)
float64 processing_time_ms
