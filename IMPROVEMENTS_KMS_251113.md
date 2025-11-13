# 성능 최적화 및 보행자 오검출 방지 개선 (KMS_251113)

## 📊 개선 개요

**목적**: 성능 향상 + 보행자 오검출 문제 해결

**개선 일자**: 2025-11-13

**주요 문제점**:
1. 🔴 전체 Lanelet 맵 순회로 인한 성능 병목 (O(N×M))
2. 🔴 보행자가 역주행으로 오검출되는 문제
3. 🟡 중복 계산 (클래스/속도 체크가 여러 곳에서 반복)
4. 🟡 Debug info 과다 발행으로 인한 네트워크 부하
5. 🟡 로그 스팸

---

## ✅ 개선 내용

### 1. 성능 최적화 - 전체 맵 순회 제거

**위치**: `src/abnormal_behavior_detector_node.cpp:249-273`

**Before** (문제):
```cpp
// 전체 Lanelet 맵을 매번 순회 (O(N))
bool is_on_lanelet = false;
for (const auto & ll : lanelet_map_ptr_->laneletLayer) {
  if (lanelet::geometry::inside(ll, search_point)) {
    is_on_lanelet = true;
    break;
  }
}
```

**문제점**:
- 50개 객체 × 100개 Lanelet = **5000번** inside() 체크
- `findClosestLanelet()`에서 이미 nearby 검색을 함 (중복 계산)

**After** (개선):
```cpp
// nearby Lanelet만 체크 (O(k), k=5~10)
const auto nearby_lanelets = lanelet::geometry::findNearest(
  lanelet_map_ptr_->laneletLayer, search_point, num_nearby_lanelets_);

for (const auto & [dist, ll] : nearby_lanelets) {
  if (lanelet::geometry::inside(ll, search_point)) {
    is_on_lanelet = true;
    break;
  }
}
```

**성능 개선**:
- 100개 Lanelet → 5~10개만 체크
- **90% 이상 연산 감소**
- 처리 시간: 5ms → 0.5ms 예상 (10배 향상)

---

### 2. 보행자 오검출 방지 - Early Return

**위치**: `src/abnormal_behavior_detector_node.cpp:249-324`

**Before** (문제):
- 보행자도 Lanelet 매칭 → heading 계산 → 역주행 판단까지 모두 수행
- `isWrongWayDriving()` 함수에서야 비로소 필터링

**After** (개선):
```cpp
// detectAbnormalBehavior() 초반에 즉시 필터링
if (!object.classification.empty()) {
  const uint8_t label = object.classification[0].label;

  if (label == PEDESTRIAN && !detect_wrong_way_for_pedestrian_) {
    // 즉시 NORMAL 반환 - Lanelet 매칭 불필요
    return info;
  }
}

// 저속 객체 빠른 필터링
if (speed < min_speed_for_wrong_way_) {
  return info;  // 3.0 m/s (10.8 km/h) 미만 제외
}
```

**효과**:
- ✅ 보행자는 Lanelet 매칭 전에 **즉시 제외** (CPU 낭비 방지)
- ✅ 저속 객체(< 10.8 km/h) 자동 제외
- ✅ `isWrongWayDriving()`의 중복 체크 제거

---

### 3. 속도 임계값 상향 조정

**위치**: `config/abnormal_behavior_detector.param.yaml:81`

**Before**:
```yaml
min_speed_for_wrong_way: 2.0  # 7.2 km/h
```

**문제점**:
- 뛰는 보행자(10 km/h)도 검출 대상에 포함
- 횡단보도를 빠르게 건너는 보행자 오검출

**After**:
```yaml
min_speed_for_wrong_way: 3.0  # 10.8 km/h (권장)
```

**기준**:
| 속도 | km/h | 대상 |
|------|------|------|
| 1.0 m/s | 3.6 km/h | 걷는 보행자 |
| 2.0 m/s | 7.2 km/h | 뛰는 보행자 |
| **3.0 m/s** | **10.8 km/h** | **자전거/차량** ← 권장 |
| 5.0 m/s | 18.0 km/h | 빠른 차량만 |

**효과**:
- ✅ 대부분의 보행자 자동 제외
- ✅ 자전거 및 차량만 검출

---

### 4. Debug Info 발행 최적화

**위치**: `src/abnormal_behavior_detector_node.cpp:141-145`

**Before**:
```cpp
// 모든 객체의 debug info 발행
pub_debug_info_->publish(debug_info);
```

**문제점**:
- 50개 객체 × 10Hz = **초당 500개 메시지**
- 대역폭 낭비 (이상 거동이 없어도 발행)

**After**:
```cpp
// 이상 거동 객체만 발행
if (behavior_info.type != AbnormalBehaviorType::NORMAL) {
  pub_debug_info_->publish(debug_info);
}
```

**효과**:
- ✅ 이상 거동이 없으면 0개 메시지
- ✅ 역주행 1대 발견 시 → 초당 10개 메시지만
- ✅ **90% 이상 네트워크 부하 감소**

---

### 5. 로그 스팸 방지

**위치**: `src/abnormal_behavior_detector_node.cpp:155-160`

**Before**:
```cpp
RCLCPP_WARN(get_logger(), "[WRONG-WAY] ...");
```

**문제점**:
- 역주행 검출될 때마다 10Hz로 로그 출력
- 터미널 스팸 (가독성 저하)

**After**:
```cpp
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[WRONG-WAY] ...");
```

**효과**:
- ✅ 5초에 한 번만 출력
- ✅ 토픽으로 이미 정보 제공 중 (로그 중복 제거)

---

## 📈 성능 개선 효과 (추정)

| 항목 | Before | After | 개선율 |
|------|--------|-------|--------|
| Lanelet 체크 횟수 | 5000회 | 50~100회 | **98%↓** |
| 보행자 처리 시간 | 5ms | 0.01ms | **99%↓** |
| Debug info 발행 | 500/초 | 0~10/초 | **98%↓** |
| 로그 출력 | 10/초 | 0.2/초 | **98%↓** |
| **전체 처리 시간** | **5~10ms** | **0.5~1ms** | **90%↓** |

**시나리오**: 50개 객체 (차량 10대, 보행자 40명)
- Before: 50개 모두 Lanelet 매칭 수행 → 10ms
- After: 차량 10대만 매칭 수행 → 1ms

---

## 🎯 보행자 오검출 방지 메커니즘

### 3중 필터링

1. **클래스 필터링** (249-313줄)
   ```cpp
   if (label == PEDESTRIAN && !detect_wrong_way_for_pedestrian_) {
     return info;  // 즉시 종료
   }
   ```

2. **속도 필터링** (315-324줄)
   ```cpp
   if (speed < 3.0 m/s) {  // 10.8 km/h
     return info;
   }
   ```

3. **Occupancy 필터링** (기존 v1.6 기능)
   ```cpp
   if (!is_on_lanelet) {
     return info;  // Lanelet 위에 없으면 제외
   }
   ```

### 결과

| 보행자 타입 | 속도 | 클래스 필터링 | 속도 필터링 | Occupancy | 결과 |
|------------|------|--------------|------------|-----------|------|
| 걷는 보행자 | 5 km/h | ✅ 제외 | ✅ 제외 | - | ✅ 검출 안 됨 |
| 뛰는 보행자 | 10 km/h | ✅ 제외 | ✅ 제외 | - | ✅ 검출 안 됨 |
| 빠른 보행자 | 15 km/h | ✅ 제외 | - | - | ✅ 검출 안 됨 |
| 차량 (저속) | 5 km/h | - | ✅ 제외 | - | ✅ 검출 안 됨 |
| 차량 (역주행) | 20 km/h | - | - | ✅ 체크 | ⚠️ 역주행 검출 |

---

## 🔧 사용 방법

### 빌드
```bash
cd ~/autoware
colcon build --packages-select abnormal_behavior_detector
source install/setup.bash
```

### 실행
```bash
ros2 launch abnormal_behavior_detector abnormal_behavior_detector.launch.xml
```

### 파라미터 튜닝 (필요 시)

보행자 오검출이 계속되면:
```yaml
# config/abnormal_behavior_detector.param.yaml

# 1. 속도 임계값 더 높이기
min_speed_for_wrong_way: 5.0  # 18.0 km/h (자전거도 일부 제외)

# 2. 보행자 검출 완전 비활성화 (기본값)
detect_wrong_way_for_pedestrian: false

# 3. 자전거도 제외
detect_wrong_way_for_bicycle: false
```

역주행 미검출이 발생하면:
```yaml
# 속도 임계값 낮추기
min_speed_for_wrong_way: 2.0  # 7.2 km/h

# 연속 프레임 수 줄이기
consecutive_count_threshold: 2  # 2프레임 = 0.2초

# 각도 임계값 낮추기 (곡선 도로)
wrong_way_angle_threshold: 2.094  # 120도
```

---

## 📝 변경 파일 목록

1. **src/abnormal_behavior_detector_node.cpp**
   - 249-324줄: Early return 추가 (클래스/속도 필터링)
   - 257-273줄: 전체 맵 순회 → nearby 체크로 변경
   - 141-145줄: Debug info 발행 필터링
   - 155-160줄: 로그 throttle 적용
   - 451-452줄: isWrongWayDriving() 중복 체크 제거

2. **config/abnormal_behavior_detector.param.yaml**
   - 81줄: min_speed_for_wrong_way: 2.0 → 3.0

---

## ⚠️ 주의사항

1. **속도 임계값 조정**
   - `min_speed_for_wrong_way: 3.0`은 권장값
   - 환경에 따라 2.0~5.0 범위에서 튜닝 필요

2. **Debug Info 발행**
   - 정상 객체는 더 이상 발행되지 않음
   - 전체 객체 모니터링이 필요하면 코드 수정 필요

3. **성능 측정**
   ```bash
   # 처리 시간 확인
   ros2 topic echo /abnormal_behavior/debug/processing_time_ms
   ```
   - 기대값: 0.5~2ms (객체 수에 따라 다름)
   - 10ms 이상이면 추가 최적화 필요

---

## 🚀 향후 개선 방향

1. **Lanelet 매칭 캐싱**
   - 객체별로 마지막 매칭 결과 캐싱
   - 위치가 크게 변하지 않으면 재사용

2. **멀티스레드 처리**
   - 객체별 검출을 병렬 처리
   - std::async 또는 OpenMP 활용

3. **신호등/정지선 고려**
   - 비정상 정차 검출 로직 개선 (TODO 487줄)
   - Lanelet의 stopline 정보 활용

4. **Unit Test 추가**
   - 보행자 필터링 테스트
   - 역주행 검출 정확도 테스트
   - 성능 벤치마크 테스트

---

## 📞 문의

문제 발생 시:
1. Processing time 확인: `ros2 topic echo /abnormal_behavior/debug/processing_time_ms`
2. Debug info 확인: `ros2 topic echo /abnormal_behavior/output/debug_info`
3. 로그 확인: `ros2 launch` 터미널 출력

**버전**: v1.7 (KMS_251113 개선 적용)
