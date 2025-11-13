// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ABNORMAL_BEHAVIOR_DETECTOR__ABNORMAL_BEHAVIOR_DETECTOR_NODE_HPP_
#define ABNORMAL_BEHAVIOR_DETECTOR__ABNORMAL_BEHAVIOR_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <abnormal_behavior_detector/msg/abnormal_behavior_status.hpp>
#include <abnormal_behavior_detector/msg/object_debug_info.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace abnormal_behavior_detector
{

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using abnormal_behavior_detector::msg::AbnormalBehaviorStatus;
using abnormal_behavior_detector::msg::ObjectDebugInfo;
using tier4_debug_msgs::msg::Float64Stamped;

/**
 * @brief 이상 거동 타입
 */
enum class AbnormalBehaviorType {
  NORMAL = 0,           // 정상
  WRONG_WAY = 1,        // 역주행
  OVER_SPEED = 2,       // 과속
  UNDER_SPEED = 3,      // 저속
  ABNORMAL_STOP = 4,    // 비정상 정차
  LANE_VIOLATION = 5    // 차선 위반
};

/**
 * @brief 이상 거동 정보 구조체
 */
struct AbnormalBehaviorInfo
{
  AbnormalBehaviorType type;
  std::string object_id;
  double confidence;  // 신뢰도 (0.0 ~ 1.0)
  std::string description;
};

/**
 * @brief 객체별 이상 거동 이력
 */
struct ObjectAbnormalHistory
{
  std::string object_id;
  std::vector<AbnormalBehaviorType> history;  // 최근 N프레임의 이상 거동 이력
  rclcpp::Time last_update_time;
  int consecutive_wrong_way_count;  // 연속 역주행 카운트
  // KMS_251113: Heading 안정화를 위한 이력 (보행자 오검출 방지)
  std::vector<double> heading_history;  // 최근 N프레임의 yaw 각도 (라디안)
};

/**
 * @brief 이상 거동 검출 노드
 */
class AbnormalBehaviorDetectorNode : public rclcpp::Node
{
public:
  explicit AbnormalBehaviorDetectorNode(const rclcpp::NodeOptions & options);

private:
  // Subscribers
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  // Publishers
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_abnormal_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  rclcpp::Publisher<AbnormalBehaviorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<ObjectDebugInfo>::SharedPtr pub_debug_info_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_;

  // Transform Listener
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  // Lanelet2 Map
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;

  // Object History
  std::unordered_map<std::string, ObjectAbnormalHistory> object_history_;

  // Parameters
  double dist_threshold_for_searching_lanelet_;   // 차선 검색 거리 임계값
  double delta_yaw_threshold_for_searching_lanelet_;  // 차선 검색 각도 임계값
  double wrong_way_angle_threshold_;  // 역주행 판단 각도 임계값 (라디안)
  int consecutive_count_threshold_;   // 이상 거동 확정을 위한 연속 프레임 수
  double min_speed_for_wrong_way_;    // 역주행 검출 최소 속도 임계값 (m/s)
  double speed_threshold_ratio_;      // 과속/저속 판단 비율
  double min_speed_threshold_;        // 정차 판단 속도 임계값
  int history_buffer_size_;           // 이력 버퍼 크기
  double history_timeout_;            // 이력 타임아웃 (초)
  double position_based_id_grid_size_;  // 위치 기반 ID 그리드 크기 (m)
  bool use_position_based_tracking_;    // 위치 기반 추적 사용 여부
  // KMS_251107: 새로 추가된 파라미터 (하드코딩 제거)
  double nearby_lanelet_threshold_;     // Lanelet 경계 근처 매칭 거리 임계값 (m)
  int num_nearby_lanelets_;             // 검색할 인근 Lanelet 개수

  // Behavior detection flags
  bool detect_over_speed_;
  bool detect_under_speed_;
  bool detect_abnormal_stop_;

  // Per-class wrong-way detection flags
  bool detect_wrong_way_for_car_;
  bool detect_wrong_way_for_truck_;
  bool detect_wrong_way_for_bus_;
  bool detect_wrong_way_for_trailer_;
  bool detect_wrong_way_for_motorcycle_;
  bool detect_wrong_way_for_bicycle_;
  bool detect_wrong_way_for_pedestrian_;
  bool detect_wrong_way_for_unknown_;

  // Callbacks
  void onObjects(const PredictedObjects::ConstSharedPtr msg);
  void onMap(const HADMapBin::ConstSharedPtr msg);

  // Core functions
  /**
   * @brief 객체의 이상 거동 검출
   */
  AbnormalBehaviorInfo detectAbnormalBehavior(
    const PredictedObject & object, ObjectDebugInfo & debug_info,
    const rclcpp::Time & current_time);

  /**
   * @brief 역주행 검출
   */
  bool isWrongWayDriving(
    const PredictedObject & object, const lanelet::ConstLanelet & matched_lanelet,
    ObjectDebugInfo & debug_info);

  /**
   * @brief 과속 검출
   */
  bool isOverSpeeding(const PredictedObject & object, const lanelet::ConstLanelet & lanelet);

  /**
   * @brief 저속 검출
   */
  bool isUnderSpeeding(const PredictedObject & object, const lanelet::ConstLanelet & lanelet);

  /**
   * @brief 비정상 정차 검출
   */
  bool isAbnormalStop(const PredictedObject & object, const lanelet::ConstLanelet & lanelet);

  // Utility functions
  /**
   * @brief 객체 위치에서 가장 가까운 차선 찾기
   */
  boost::optional<lanelet::ConstLanelet> findClosestLanelet(const PredictedObject & object);

  /**
   * @brief 객체의 주행 방향 벡터 계산
   */
  Eigen::Vector2d getObjectHeadingVector(const PredictedObject & object);

  /**
   * @brief 차선의 방향 벡터 계산 (객체 위치 기준)
   */
  Eigen::Vector2d getLaneletDirectionVector(
    const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & position);

  /**
   * @brief 객체 속도 계산
   */
  double getObjectSpeed(const PredictedObject & object);

  /**
   * @brief 차선의 제한 속도 가져오기
   */
  boost::optional<double> getLaneletSpeedLimit(const lanelet::ConstLanelet & lanelet);

  /**
   * @brief 객체 이력 업데이트
   */
  void updateObjectHistory(
    const std::string & object_id, AbnormalBehaviorType behavior_type,
    const rclcpp::Time & current_time);

  /**
   * @brief 오래된 이력 삭제
   */
  void cleanupOldHistory(const rclcpp::Time & current_time);

  /**
   * @brief 이상 거동 확정 여부 판단 (연속 프레임 확인)
   */
  bool isAbnormalBehaviorConfirmed(const std::string & object_id, AbnormalBehaviorType type);

  /**
   * @brief UUID를 string으로 변환
   */
  std::string uuidToString(const unique_identifier_msgs::msg::UUID & uuid);

  /**
   * @brief 위치 기반 stable object ID 생성
   */
  std::string getStableObjectId(const PredictedObject & object);

  /**
   * @brief KMS_251113: Heading 안정화 (이동 평균 필터)
   * @param object_id 객체 ID
   * @param current_yaw 현재 프레임의 yaw 각도 (라디안)
   * @return 안정화된 yaw 각도 (라디안)
   */
  double getSmoothedHeading(const std::string & object_id, double current_yaw);

  // Visualization
  visualization_msgs::msg::MarkerArray createDebugMarkers(
    const std::vector<std::pair<PredictedObject, AbnormalBehaviorInfo>> & abnormal_objects,
    const rclcpp::Time & current_time);
};

}  // namespace abnormal_behavior_detector

#endif  // ABNORMAL_BEHAVIOR_DETECTOR__ABNORMAL_BEHAVIOR_DETECTOR_NODE_HPP_
