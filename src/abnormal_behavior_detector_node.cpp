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

#include "abnormal_behavior_detector/abnormal_behavior_detector_node.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace abnormal_behavior_detector
{

AbnormalBehaviorDetectorNode::AbnormalBehaviorDetectorNode(const rclcpp::NodeOptions & options)
: Node("abnormal_behavior_detector", options)
{
  using std::placeholders::_1;

  // Parameters
  dist_threshold_for_searching_lanelet_ =
    declare_parameter<double>("dist_threshold_for_searching_lanelet", 5.0);
  delta_yaw_threshold_for_searching_lanelet_ =
    declare_parameter<double>("delta_yaw_threshold_for_searching_lanelet", 0.785);  // 45도
  wrong_way_angle_threshold_ =
    declare_parameter<double>("wrong_way_angle_threshold", 2.356);  // 135도 (3π/4)
  consecutive_count_threshold_ = declare_parameter<int>("consecutive_count_threshold", 3);
  min_speed_for_wrong_way_ = declare_parameter<double>("min_speed_for_wrong_way", 2.0);  // m/s
  speed_threshold_ratio_ = declare_parameter<double>("speed_threshold_ratio", 1.2);
  min_speed_threshold_ = declare_parameter<double>("min_speed_threshold", 0.5);  // m/s
  history_buffer_size_ = declare_parameter<int>("history_buffer_size", 10);
  history_timeout_ = declare_parameter<double>("history_timeout", 3.0);  // seconds
  position_based_id_grid_size_ = declare_parameter<double>("position_based_id_grid_size", 3.0);  // m
  use_position_based_tracking_ = declare_parameter<bool>("use_position_based_tracking", true);
  // KMS_251107: 하드코딩 제거 - 새로운 파라미터 추가
  nearby_lanelet_threshold_ = declare_parameter<double>("nearby_lanelet_threshold", 5.0);  // m
  num_nearby_lanelets_ = declare_parameter<int>("num_nearby_lanelets", 10);  // 개수

  // Behavior detection flags
  detect_over_speed_ = declare_parameter<bool>("detect_over_speed", false);
  detect_under_speed_ = declare_parameter<bool>("detect_under_speed", false);
  detect_abnormal_stop_ = declare_parameter<bool>("detect_abnormal_stop", false);

  // Transform listener
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  // Subscribers
  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&AbnormalBehaviorDetectorNode::onObjects, this, _1));

  sub_map_ = create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&AbnormalBehaviorDetectorNode::onMap, this, _1));

  // Publishers
  pub_abnormal_objects_ = create_publisher<PredictedObjects>("~/output/abnormal_objects", 1);
  pub_debug_markers_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 1);
  pub_status_ = create_publisher<AbnormalBehaviorStatus>("~/output/status", 1);
  pub_debug_info_ = create_publisher<ObjectDebugInfo>("~/output/debug_info", 1);
  pub_processing_time_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);

  RCLCPP_INFO(get_logger(), "AbnormalBehaviorDetectorNode initialized");
  RCLCPP_INFO(get_logger(), "  Outputs:");
  RCLCPP_INFO(get_logger(), "    - ~/output/abnormal_objects (PredictedObjects)");
  RCLCPP_INFO(get_logger(), "    - ~/output/status (AbnormalBehaviorStatus)");
  RCLCPP_INFO(get_logger(), "    - ~/output/debug_info (ObjectDebugInfo)");
  RCLCPP_INFO(get_logger(), "    - ~/debug/processing_time_ms (Float64Stamped)");
}

void AbnormalBehaviorDetectorNode::onMap(const HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Map received");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);

  // Traffic rules 생성
  traffic_rules_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);

  // Routing graph 생성
  routing_graph_ptr_ = lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_ptr_);

  RCLCPP_INFO(
    get_logger(), "Map loaded: %lu lanelets", lanelet_map_ptr_->laneletLayer.size());
}

void AbnormalBehaviorDetectorNode::onObjects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!lanelet_map_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Map not loaded yet");
    return;
  }

  // 처리 시작 시간
  const auto start_time = std::chrono::high_resolution_clock::now();

  // 이상 거동 객체 리스트
  std::vector<std::pair<PredictedObject, AbnormalBehaviorInfo>> abnormal_objects;

  // 타입별 카운트
  int wrong_way_count = 0;
  int over_speed_count = 0;
  int under_speed_count = 0;
  int abnormal_stop_count = 0;

  // KMS_251113: Time source 통일 - 한 번만 생성하고 재사용
  const rclcpp::Time current_time(msg->header.stamp, RCL_ROS_TIME);

  // 각 객체에 대해 이상 거동 검출
  for (const auto & object : msg->objects) {
    ObjectDebugInfo debug_info;
    debug_info.header = msg->header;

    auto behavior_info = detectAbnormalBehavior(object, debug_info, current_time);

    // KMS_251107: Debug info는 이상 거동 객체만 발행 (대역폭 최적화)
    // if (behavior_info.type != AbnormalBehaviorType::NORMAL) {
    pub_debug_info_->publish(debug_info);
    // }

    if (behavior_info.type != AbnormalBehaviorType::NORMAL) {
      abnormal_objects.push_back({object, behavior_info});

      // 타입별 카운트
      switch (behavior_info.type) {
        case AbnormalBehaviorType::WRONG_WAY:
          wrong_way_count++;
          RCLCPP_WARN(
            get_logger(), "[WRONG-WAY] %s at (%.1f, %.1f)",
            behavior_info.object_id.c_str(),
            object.kinematics.initial_pose_with_covariance.pose.position.x,
            object.kinematics.initial_pose_with_covariance.pose.position.y);
          break;
        case AbnormalBehaviorType::OVER_SPEED:
          over_speed_count++;
          break;
        case AbnormalBehaviorType::UNDER_SPEED:
          under_speed_count++;
          break;
        case AbnormalBehaviorType::ABNORMAL_STOP:
          abnormal_stop_count++;
          break;
        default:
          break;
      }
    }
  }

  // 이상 거동 객체 발행
  if (!abnormal_objects.empty()) {
    PredictedObjects abnormal_msg;
    abnormal_msg.header = msg->header;
    for (const auto & [obj, info] : abnormal_objects) {
      abnormal_msg.objects.push_back(obj);
    }
    pub_abnormal_objects_->publish(abnormal_msg);

    // 디버그 마커 발행
    auto markers = createDebugMarkers(abnormal_objects, current_time);
    pub_debug_markers_->publish(markers);
  }

  // 처리 종료 시간 및 시간 계산
  const auto end_time = std::chrono::high_resolution_clock::now();
  const auto processing_time_us =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  const double processing_time_ms = processing_time_us / 1000.0;

  // Status 토픽 발행
  AbnormalBehaviorStatus status;
  status.header = msg->header;
  status.has_abnormal_behavior = !abnormal_objects.empty();
  status.total_objects = msg->objects.size();
  status.abnormal_objects_count = abnormal_objects.size();
  status.wrong_way_count = wrong_way_count;
  status.over_speed_count = over_speed_count;
  status.under_speed_count = under_speed_count;
  status.abnormal_stop_count = abnormal_stop_count;
  status.processing_time_ms = processing_time_ms;
  pub_status_->publish(status);

  // Processing time 토픽 발행
  Float64Stamped processing_time_msg;
  processing_time_msg.stamp = msg->header.stamp;
  processing_time_msg.data = processing_time_ms;
  pub_processing_time_->publish(processing_time_msg);

  // 오래된 이력 삭제
  cleanupOldHistory(current_time);
}

AbnormalBehaviorInfo AbnormalBehaviorDetectorNode::detectAbnormalBehavior(
  const PredictedObject & object, ObjectDebugInfo & debug_info,
  const rclcpp::Time & current_time)
{
  AbnormalBehaviorInfo info;

  // 위치 기반 추적 사용 시 stable ID 생성, 아니면 UUID 사용
  if (use_position_based_tracking_) {
    info.object_id = getStableObjectId(object);
  } else {
    info.object_id = uuidToString(object.object_id);
  }

  info.type = AbnormalBehaviorType::NORMAL;
  info.confidence = 0.0;
  info.description = "Normal";

  const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
  const double raw_yaw = tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation);

  // KMS_251113: Heading 안정화 (보행자 yaw 튐 현상 방지)
  // 최근 5프레임의 yaw를 원형 평균하여 안정화
  const double yaw = getSmoothedHeading(info.object_id, raw_yaw);

  // Debug info 기본 정보
  debug_info.stable_object_id = info.object_id;
  debug_info.uuid = uuidToString(object.object_id).substr(0, 8);
  debug_info.object_class = object.classification.empty() ? 0 : object.classification[0].label;
  debug_info.position = pos;
  debug_info.yaw_rad = yaw;  // 안정화된 yaw 사용
  debug_info.yaw_deg = yaw * 180.0 / M_PI;

  // 속도 정보
  const double speed = getObjectSpeed(object);
  debug_info.speed_ms = speed;
  debug_info.speed_kmh = speed * 3.6;

  // KMS_251113: 성능 최적화 - 전체 맵 순회 제거
  // 기존: 전체 lanelet_map을 순회 (O(N))
  // 개선: findClosestLanelet()에서 nearby만 검색하고, 그 결과로 is_on_lanelet 판단
  lanelet::BasicPoint2d search_point(pos.x, pos.y);

  // 가장 가까운 차선 찾기
  auto closest_lanelet_opt = findClosestLanelet(object);

  // KMS_251113: is_on_lanelet 체크 - nearby lanelet만 확인 (성능 향상)
  bool is_on_lanelet = false;
  if (closest_lanelet_opt) {
    // findClosestLanelet()의 nearby 결과 재사용
    const auto nearby_lanelets = lanelet::geometry::findNearest(
      lanelet_map_ptr_->laneletLayer, search_point, num_nearby_lanelets_);

    for (const auto & [dist, ll] : nearby_lanelets) {
      if (lanelet::geometry::inside(ll, search_point)) {
        is_on_lanelet = true;
        break;
      }
    }
  }
  debug_info.is_on_lanelet = is_on_lanelet;

  // KMS_251111: 역주행 판단은 반드시 lanelet 위에 있을 때만 수행
  if (!closest_lanelet_opt || !is_on_lanelet) {
    debug_info.lanelet_matched = false;
    debug_info.matched_lanelet_id = -1;
    debug_info.behavior_type = "NORMAL";
    debug_info.description = is_on_lanelet ? "On lanelet but no direction match" : "Not on lanelet";

    // 디버그: lanelet 매칭 실패 이유 로그
    RCLCPP_DEBUG(
      get_logger(), "[%s] Class=%d, Speed=%.1fkm/h - %s (lanelet_opt=%s, on_lanelet=%s)",
      info.object_id.c_str(), debug_info.object_class, debug_info.speed_kmh,
      debug_info.description.c_str(),
      closest_lanelet_opt ? "true" : "false",
      is_on_lanelet ? "true" : "false");

    return info;  // 차선을 찾을 수 없으면 정상으로 판단
  }

  const auto & lanelet = closest_lanelet_opt.get();
  debug_info.lanelet_matched = true;
  debug_info.matched_lanelet_id = lanelet.id();

  // 속도 제한 확인
  const auto speed_limit_opt = getLaneletSpeedLimit(lanelet);
  if (speed_limit_opt) {
    debug_info.speed_limit_ms = speed_limit_opt.get();
    debug_info.speed_limit_kmh = speed_limit_opt.get() * 3.6;
  } else {
    debug_info.speed_limit_ms = 0.0;
    debug_info.speed_limit_kmh = 0.0;
  }

  // 1. 역주행 검출 (우선순위 최상위)
  if (isWrongWayDriving(object, lanelet, debug_info)) {
    updateObjectHistory(info.object_id, AbnormalBehaviorType::WRONG_WAY, current_time);

    const auto it = object_history_.find(info.object_id);
    const int count = (it != object_history_.end()) ? it->second.consecutive_wrong_way_count : 0;

    debug_info.consecutive_count = count;
    debug_info.is_confirmed = (count >= consecutive_count_threshold_);

    if (isAbnormalBehaviorConfirmed(info.object_id, AbnormalBehaviorType::WRONG_WAY)) {
      info.type = AbnormalBehaviorType::WRONG_WAY;
      info.confidence = 0.95;
      info.description = "Wrong-way";
      debug_info.behavior_type = "WRONG_WAY";
      debug_info.confidence = 0.95;
      debug_info.description = "Wrong-way driving detected";
      return info;
    }

    // 역주행이 감지되었지만 아직 확정되지 않은 경우, 카운터가 리셋되지 않도록 여기서 반환
    debug_info.behavior_type = "POTENTIAL_WRONG_WAY";
    debug_info.description = "Potential wrong-way, accumulating count.";
    return info;
  } else {
    // 역주행이 아니므로 카운터를 리셋
    updateObjectHistory(info.object_id, AbnormalBehaviorType::NORMAL, current_time);
  }

  // 2. 비정상 정차 검출
  if (detect_abnormal_stop_ && isAbnormalStop(object, lanelet)) {
    info.type = AbnormalBehaviorType::ABNORMAL_STOP;
    info.confidence = 0.8;
    info.description = "Abnormal stop detected";
    debug_info.behavior_type = "ABNORMAL_STOP";
    debug_info.confidence = 0.8;
    debug_info.description = "Abnormal stop detected";
    return info;
  }

  // 3. 과속 검출
  if (detect_over_speed_ && isOverSpeeding(object, lanelet)) {
    info.type = AbnormalBehaviorType::OVER_SPEED;
    info.confidence = 0.7;
    info.description = "Over-speeding detected";
    debug_info.behavior_type = "OVER_SPEED";
    debug_info.confidence = 0.7;
    debug_info.description = "Over-speeding detected";
    return info;
  }

  // 4. 저속 검출
  if (detect_under_speed_ && isUnderSpeeding(object, lanelet)) {
    info.type = AbnormalBehaviorType::UNDER_SPEED;
    info.confidence = 0.6;
    info.description = "Under-speeding detected";
    debug_info.behavior_type = "UNDER_SPEED";
    debug_info.confidence = 0.6;
    debug_info.description = "Under-speeding detected";
    return info;
  }

  // 정상인 경우
  debug_info.behavior_type = "NORMAL";
  debug_info.confidence = 1.0;
  debug_info.description = "Normal";
  debug_info.consecutive_count = 0;
  debug_info.is_confirmed = false;

  return info;
}

bool AbnormalBehaviorDetectorNode::isWrongWayDriving(
  const PredictedObject & object, const lanelet::ConstLanelet & matched_lanelet,
  ObjectDebugInfo & debug_info)
{
  // 속도 체크 - 저속 객체(보행자 등)는 역주행 검출에서 제외
  const double object_speed = getObjectSpeed(object);
  if (object_speed < min_speed_for_wrong_way_) {
    return false;  // 너무 느리면 역주행 검출 제외 (보행자 오검출 방지)
  }

  Eigen::Vector2d object_direction;

  // PredictedPath 사용 (가장 정확)
  if (!object.kinematics.predicted_paths.empty()) {
    const auto & predicted_path = object.kinematics.predicted_paths[0];

    if (predicted_path.path.size() >= 2) {
      // 현재 위치에서 미래 위치로의 방향 벡터 계산
      const auto & current_pos = object.kinematics.initial_pose_with_covariance.pose.position;
      const auto & future_pos = predicted_path.path[std::min(predicted_path.path.size() - 1, size_t(5))].position;

      const double dx = future_pos.x - current_pos.x;
      const double dy = future_pos.y - current_pos.y;
      const double dist = std::sqrt(dx * dx + dy * dy);

      if (dist > 0.5) {
        object_direction = Eigen::Vector2d(dx, dy).normalized();
      } else {
        object_direction = getObjectHeadingVector(object);
      }
    } else {
      object_direction = getObjectHeadingVector(object);
    }
  } else {
    // PredictedPath가 없으면 heading 사용
    object_direction = getObjectHeadingVector(object);
  }

  // 차선의 방향 벡터 (단위 벡터)
  const auto lanelet_direction =
    getLaneletDirectionVector(matched_lanelet, object.kinematics.initial_pose_with_covariance.pose.position);

  // 내적 계산 (두 벡터가 단위 벡터이므로 바로 cos(θ))
  const double dot_product = object_direction.dot(lanelet_direction);

  // 각도 계산
  const double angle_rad = std::acos(std::clamp(dot_product, -1.0, 1.0));
  const double angle_deg = angle_rad * 180.0 / M_PI;
  const double cos_threshold = std::cos(wrong_way_angle_threshold_);

  // Debug info에 정보 채우기
  debug_info.object_heading_vector.x = object_direction.x();
  debug_info.object_heading_vector.y = object_direction.y();
  debug_info.object_heading_vector.z = 0.0;
  debug_info.lanelet_direction_vector.x = lanelet_direction.x();
  debug_info.lanelet_direction_vector.y = lanelet_direction.y();
  debug_info.lanelet_direction_vector.z = 0.0;
  debug_info.dot_product = dot_product;
  debug_info.angle_diff_deg = angle_deg;
  debug_info.wrong_way_threshold_deg = wrong_way_angle_threshold_ * 180.0 / M_PI;

  // 각도가 임계값보다 크면 역주행
  const bool is_wrong_way = dot_product < cos_threshold;

  return is_wrong_way;
}

bool AbnormalBehaviorDetectorNode::isOverSpeeding(
  const PredictedObject & object, const lanelet::ConstLanelet & lanelet)
{
  const double object_speed = getObjectSpeed(object);
  const auto speed_limit_opt = getLaneletSpeedLimit(lanelet);

  if (!speed_limit_opt) {
    return false;  // 제한 속도 정보가 없으면 판단 불가
  }

  const double speed_limit = speed_limit_opt.get();
  return object_speed > speed_limit * speed_threshold_ratio_;
}

bool AbnormalBehaviorDetectorNode::isUnderSpeeding(
  const PredictedObject & object, const lanelet::ConstLanelet & lanelet)
{
  const double object_speed = getObjectSpeed(object);
  const auto speed_limit_opt = getLaneletSpeedLimit(lanelet);

  if (!speed_limit_opt || speed_limit_opt.get() < 5.0) {
    return false;  // 제한 속도가 너무 낮거나 정보가 없으면 판단 불가
  }

  const double speed_limit = speed_limit_opt.get();
  const double min_acceptable_speed = speed_limit * 0.3;  // 제한 속도의 30% 미만

  return object_speed > min_speed_threshold_ && object_speed < min_acceptable_speed;
}

bool AbnormalBehaviorDetectorNode::isAbnormalStop(
  const PredictedObject & object, [[maybe_unused]] const lanelet::ConstLanelet & lanelet)
{
  const double object_speed = getObjectSpeed(object);

  // 정차 판단
  if (object_speed > min_speed_threshold_) {
    return false;  // 움직이고 있으면 정차 아님
  }

  // TODO: 신호등, 정지선 근처인지 확인하여 정상 정차 여부 판단
  // 현재는 단순히 차선 중앙에서 정차하면 이상으로 판단

  return true;
}

// KMS_251107: Lanelet 매칭 알고리즘 개선
// - 하드코딩된 2m 임계값 제거 → 파라미터 사용
// - 검색 lanelet 개수 5개 → 파라미터로 변경 (원형 교차로 대응)
// - 1단계 occupancy 체크도 최적화 (nearby만 체크)
boost::optional<lanelet::ConstLanelet> AbnormalBehaviorDetectorNode::findClosestLanelet(
  const PredictedObject & object)
{
  const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d search_point(pos.x, pos.y);

  // KMS_251107: 1단계 - 먼저 가까운 lanelet들만 가져오기 (성능 최적화)
  // 전체 맵을 순회하는 대신 가까운 것들만 검색
  const auto nearby_lanelets = lanelet::geometry::findNearest(
    lanelet_map_ptr_->laneletLayer, search_point, num_nearby_lanelets_);

  if (nearby_lanelets.empty()) {
    return boost::none;
  }

  // KMS_251107: 2단계 - 가까운 lanelet 중에서 occupancy(내부 포함) 체크
  for (const auto & [dist, lanelet] : nearby_lanelets) {
    if (lanelet::geometry::inside(lanelet, search_point)) {
      // KMS_251107: 객체가 이 lanelet 내부에 있음 → 바로 반환
      return lanelet;
    }
  }

  // KMS_251107: 3단계 - Lanelet 내부가 아니면, 경계 근처에서 각도 매칭 시도
  // (차선 경계에 걸쳐 있는 경우 대비)
  for (const auto & [dist, lanelet] : nearby_lanelets) {
    // KMS_251107: 거리 체크 - 파라미터 사용 (하드코딩 제거)
    if (dist > nearby_lanelet_threshold_) {
      continue;
    }

    // KMS_251107: 차선 방향과 객체 heading의 차이 확인
    const auto lanelet_direction = getLaneletDirectionVector(lanelet, pos);
    const auto object_heading = getObjectHeadingVector(object);

    // KMS_251107: 내적으로 각도 차이 계산
    const double dot_product = object_heading.dot(lanelet_direction);
    const double angle_diff = std::acos(std::clamp(dot_product, -1.0, 1.0));

    // KMS_251107: 각도 정규화 - 정방향(0도 ± 45도) 또는 역방향(180도 ± 45도) 모두 허용
    // 이는 역주행 차량도 올바른 lanelet과 매칭되도록 함
    const double normalized_angle = std::min(angle_diff, M_PI - angle_diff);

    if (normalized_angle < delta_yaw_threshold_for_searching_lanelet_) {
      return lanelet;
    }
  }

  // KMS_251107: 매칭 실패 - lanelet 위에도 없고, 경계 근처에서도 각도가 맞지 않음
  return boost::none;
}

Eigen::Vector2d AbnormalBehaviorDetectorNode::getObjectHeadingVector(const PredictedObject & object)
{
  const auto & pose = object.kinematics.initial_pose_with_covariance.pose;

  // Quaternion에서 Yaw 추출
  const double yaw = tf2::getYaw(pose.orientation);

  // 차량의 heading 방향 벡터 (정규화된 단위 벡터)
  return Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
}

// KMS_251107: 곡선 도로(원형 교차로) 대응 개선
// - 끝점 처리 안전성 강화
// - 중간 점에서 양방향 평균으로 더 부드러운 방향 계산
Eigen::Vector2d AbnormalBehaviorDetectorNode::getLaneletDirectionVector(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & position)
{
  const auto centerline = lanelet.centerline();

  if (centerline.size() < 2) {
    return Eigen::Vector2d(1.0, 0.0);  // KMS_251107: 기본값 (동쪽 방향)
  }

  // KMS_251107: 객체 위치에서 가장 가까운 centerline 점 찾기
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < centerline.size(); ++i) {
    const auto & pt = centerline[i];
    const double dist =
      std::sqrt((pt.x() - position.x) * (pt.x() - position.x) +
                (pt.y() - position.y) * (pt.y() - position.y));
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // KMS_251107: 방향 벡터 계산 (양방향 고려로 끝점 안전 처리)
  Eigen::Vector2d direction;

  if (closest_idx == 0) {
    // KMS_251107: 첫 점 - 다음 점 방향 사용
    const auto & p1 = centerline[0];
    const auto & p2 = centerline[1];
    direction = Eigen::Vector2d(p2.x() - p1.x(), p2.y() - p1.y());

  } else if (closest_idx == centerline.size() - 1) {
    // KMS_251107: 끝 점 - 이전 점에서 현재 점으로의 방향 사용
    const auto & p1 = centerline[centerline.size() - 2];
    const auto & p2 = centerline[centerline.size() - 1];
    direction = Eigen::Vector2d(p2.x() - p1.x(), p2.y() - p1.y());

  } else {
    // KMS_251107: 중간 점 - 이전-다음 점 사이의 평균 방향 (곡선에서 더 부드러움)
    const auto & p_prev = centerline[closest_idx - 1];
    const auto & p_curr = centerline[closest_idx];
    const auto & p_next = centerline[closest_idx + 1];

    // KMS_251107: 두 세그먼트의 평균 방향으로 곡선의 접선 방향 근사
    Eigen::Vector2d dir1(p_curr.x() - p_prev.x(), p_curr.y() - p_prev.y());
    Eigen::Vector2d dir2(p_next.x() - p_curr.x(), p_next.y() - p_curr.y());

    direction = (dir1 + dir2) / 2.0;
  }

  // KMS_251107: 정규화 (단위 벡터로 만들기)
  if (direction.norm() > 0.01) {
    direction.normalize();
  } else {
    // KMS_251107: 비상 대책 - 방향 벡터가 너무 작으면 기본값 반환
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Lanelet direction vector too small (norm < 0.01), using fallback direction");
    return Eigen::Vector2d(1.0, 0.0);
  }

  return direction;
}

double AbnormalBehaviorDetectorNode::getObjectSpeed(const PredictedObject & object)
{
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  return std::sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);
}

boost::optional<double> AbnormalBehaviorDetectorNode::getLaneletSpeedLimit(
  const lanelet::ConstLanelet & lanelet)
{
  // Lanelet의 속성에서 speed_limit 태그 확인
  const auto speed_limit_attr = lanelet.attributeOr("speed_limit", "none");
  const std::string none_str = "none";

  if (speed_limit_attr != none_str) {
    try {
      // km/h를 m/s로 변환
      const double speed_kmh = std::stod(speed_limit_attr);
      return speed_kmh / 3.6;  // km/h -> m/s
    } catch (const std::exception &) {
      // 변환 실패 시 기본값
    }
  }

  // 기본 제한 속도 (60 km/h = 16.67 m/s)
  return 16.67;
}

void AbnormalBehaviorDetectorNode::updateObjectHistory(
  const std::string & object_id, AbnormalBehaviorType behavior_type,
  const rclcpp::Time & current_time)
{
  auto & history = object_history_[object_id];
  history.object_id = object_id;
  history.last_update_time = current_time;

  // 이력 추가
  history.history.push_back(behavior_type);
  if (history.history.size() > static_cast<size_t>(history_buffer_size_)) {
    history.history.erase(history.history.begin());
  }

  // 연속 역주행 카운트 업데이트
  if (behavior_type == AbnormalBehaviorType::WRONG_WAY) {
    history.consecutive_wrong_way_count++;
  } else {
    history.consecutive_wrong_way_count = 0;
  }
}

void AbnormalBehaviorDetectorNode::cleanupOldHistory(const rclcpp::Time & current_time)
{
  for (auto it = object_history_.begin(); it != object_history_.end();) {
    try {
      // KMS_251113: Time source가 다를 수 있으므로 try-catch로 보호
      const double elapsed = (current_time - it->second.last_update_time).seconds();
      if (elapsed > history_timeout_) {
        it = object_history_.erase(it);
      } else {
        ++it;
      }
    } catch (const std::runtime_error & e) {
      // Time source가 달라서 빼기 실패 → 이 객체는 이전 time source 것이므로 삭제
      RCLCPP_DEBUG(get_logger(), "Removing object history due to time source mismatch: %s", e.what());
      it = object_history_.erase(it);
    }
  }
}

bool AbnormalBehaviorDetectorNode::isAbnormalBehaviorConfirmed(
  const std::string & object_id, AbnormalBehaviorType type)
{
  const auto it = object_history_.find(object_id);
  if (it == object_history_.end()) {
    return false;
  }

  if (type == AbnormalBehaviorType::WRONG_WAY) {
    return it->second.consecutive_wrong_way_count >= consecutive_count_threshold_;
  }

  return false;
}

std::string AbnormalBehaviorDetectorNode::uuidToString(
  const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (size_t i = 0; i < uuid.uuid.size(); ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(uuid.uuid[i]);
    if (i == 3 || i == 5 || i == 7 || i == 9) ss << "-";
  }
  return ss.str();
}

std::string AbnormalBehaviorDetectorNode::getStableObjectId(const PredictedObject & object)
{
  const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;

  // 위치를 grid_size 단위로 양자화하여 stable ID 생성
  // 예: grid_size=3.0m → (11615.82, 90913.39) → (3871, 30304)
  const int grid_x = static_cast<int>(std::round(pos.x / position_based_id_grid_size_));
  const int grid_y = static_cast<int>(std::round(pos.y / position_based_id_grid_size_));

  // "grid_X_Y" 형식으로 ID 생성
  std::stringstream ss;
  ss << "grid_" << grid_x << "_" << grid_y;

  return ss.str();
}

visualization_msgs::msg::MarkerArray AbnormalBehaviorDetectorNode::createDebugMarkers(
  const std::vector<std::pair<PredictedObject, AbnormalBehaviorInfo>> & abnormal_objects,
  const rclcpp::Time & current_time)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (const auto & [object, info] : abnormal_objects) {
    // 텍스트 마커
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = current_time;
    text_marker.ns = "abnormal_behavior_text";
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    text_marker.pose.position.z += 3.0;  // 객체 위 3m

    text_marker.scale.z = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;

    text_marker.text = info.description;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_array.markers.push_back(text_marker);

    // 시각화 마커 (빨간색 CUBE)
    visualization_msgs::msg::Marker visual_marker;
    visual_marker.header = text_marker.header;
    visual_marker.ns = "abnormal_behavior_visual";
    visual_marker.id = marker_id++;
    visual_marker.action = visualization_msgs::msg::Marker::ADD;
    visual_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    visual_marker.type = visualization_msgs::msg::Marker::CUBE;
    visual_marker.pose = object.kinematics.initial_pose_with_covariance.pose;

    // 객체의 실제 크기 사용
    if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
      visual_marker.scale.x = object.shape.dimensions.x;
      visual_marker.scale.y = object.shape.dimensions.y;
      visual_marker.scale.z = object.shape.dimensions.z;
    } else {
      visual_marker.scale.x = 2.0;
      visual_marker.scale.y = 2.0;
      visual_marker.scale.z = 2.0;
    }

    // 빨간색 반투명
    visual_marker.color.r = 1.0;
    visual_marker.color.g = 0.0;
    visual_marker.color.b = 0.0;
    visual_marker.color.a = 0.5;

    marker_array.markers.push_back(visual_marker);
  }

  return marker_array;
}

// KMS_251113: Heading 안정화 함수 (보행자 오검출 방지)
double AbnormalBehaviorDetectorNode::getSmoothedHeading(
  const std::string & object_id, double current_yaw)
{
  auto & history = object_history_[object_id];

  // 현재 yaw를 이력에 추가
  history.heading_history.push_back(current_yaw);

  // 최대 5프레임 이력 유지 (0.5초 @ 10Hz)
  const size_t max_heading_history = 5;
  if (history.heading_history.size() > max_heading_history) {
    history.heading_history.erase(history.heading_history.begin());
  }

  // 단일 프레임이면 그대로 반환
  if (history.heading_history.size() == 1) {
    return current_yaw;
  }

  // KMS_251113: 각도 평균 계산 (원형 평균)
  // 0도와 359도를 평균내면 179.5도가 나오는 문제 방지
  double sin_sum = 0.0;
  double cos_sum = 0.0;

  for (const double yaw : history.heading_history) {
    sin_sum += std::sin(yaw);
    cos_sum += std::cos(yaw);
  }

  const double avg_sin = sin_sum / history.heading_history.size();
  const double avg_cos = cos_sum / history.heading_history.size();

  // atan2로 평균 각도 계산
  const double smoothed_yaw = std::atan2(avg_sin, avg_cos);

  return smoothed_yaw;
}

}  // namespace abnormal_behavior_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(abnormal_behavior_detector::AbnormalBehaviorDetectorNode)
