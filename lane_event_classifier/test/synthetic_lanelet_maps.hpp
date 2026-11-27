// Copyright 2026 TIER IV, Inc.
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

// Shared synthetic lanelet maps and input builders for the unit tests.

#ifndef SYNTHETIC_LANELET_MAPS_HPP_
#define SYNTHETIC_LANELET_MAPS_HPP_

#include <lane_event_classifier/detail/geometry_utils.hpp>
#include <lane_event_classifier/types.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <string>
#include <vector>

namespace lane_event_classifier::test_maps
{

inline void mark_road(lanelet::Lanelet & lane)
{
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  lane.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
}

// Single straight lanelet: x=[0,10], y=[-2,2] (width 4 m, centred on Y=0).
inline lanelet::LaneletMapPtr make_single_lane_map(lanelet::Id & lane_id_out)
{
  lanelet::Point3d lp1(lanelet::utils::getId(), 0.0, 2.0, 0.0);
  lanelet::Point3d lp2(lanelet::utils::getId(), 10.0, 2.0, 0.0);
  lanelet::Point3d rp1(lanelet::utils::getId(), 0.0, -2.0, 0.0);
  lanelet::Point3d rp2(lanelet::utils::getId(), 10.0, -2.0, 0.0);
  lanelet::LineString3d left_bound(lanelet::utils::getId(), {lp1, lp2});
  lanelet::LineString3d right_bound(lanelet::utils::getId(), {rp1, rp2});
  lanelet::Lanelet lane(lanelet::utils::getId(), left_bound, right_bound);
  lane_id_out = lane.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  return map;
}

// Two lanelets connected end-to-end, so the routing graph reports lane_b as next of lane_a.
inline lanelet::LaneletMapPtr make_next_lane_map(lanelet::Id & id_a, lanelet::Id & id_b)
{
  lanelet::Point3d l0(lanelet::utils::getId(), 0.0, 2.0, 0.0);
  lanelet::Point3d r0(lanelet::utils::getId(), 0.0, -2.0, 0.0);
  lanelet::Point3d l_mid(lanelet::utils::getId(), 10.0, 2.0, 0.0);
  lanelet::Point3d r_mid(lanelet::utils::getId(), 10.0, -2.0, 0.0);
  lanelet::Point3d l1(lanelet::utils::getId(), 20.0, 2.0, 0.0);
  lanelet::Point3d r1(lanelet::utils::getId(), 20.0, -2.0, 0.0);

  lanelet::LineString3d left_a(lanelet::utils::getId(), {l0, l_mid});
  lanelet::LineString3d right_a(lanelet::utils::getId(), {r0, r_mid});
  lanelet::Lanelet lane_a(lanelet::utils::getId(), left_a, right_a);
  mark_road(lane_a);

  lanelet::LineString3d left_b(lanelet::utils::getId(), {l_mid, l1});
  lanelet::LineString3d right_b(lanelet::utils::getId(), {r_mid, r1});
  lanelet::Lanelet lane_b(lanelet::utils::getId(), left_b, right_b);
  mark_road(lane_b);

  id_a = lane_a.id();
  id_b = lane_b.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane_a);
  map->add(lane_b);
  return map;
}

// Two side-by-side lanelets: lane_a y=[-2,2], lane_b y=[2,6], both x=[0,10].
inline lanelet::LaneletMapPtr make_parallel_map(lanelet::Id & id_a, lanelet::Id & id_b)
{
  auto make_lane = [](double y_right, double y_left) {
    lanelet::Point3d lp1(lanelet::utils::getId(), 0.0, y_left, 0.0);
    lanelet::Point3d lp2(lanelet::utils::getId(), 10.0, y_left, 0.0);
    lanelet::Point3d rp1(lanelet::utils::getId(), 0.0, y_right, 0.0);
    lanelet::Point3d rp2(lanelet::utils::getId(), 10.0, y_right, 0.0);
    lanelet::LineString3d lb(lanelet::utils::getId(), {lp1, lp2});
    lanelet::LineString3d rb(lanelet::utils::getId(), {rp1, rp2});
    return lanelet::Lanelet(lanelet::utils::getId(), lb, rb);
  };
  auto lane_a = make_lane(-2.0, 2.0);
  auto lane_b = make_lane(2.0, 6.0);
  id_a = lane_a.id();
  id_b = lane_b.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane_a);
  map->add(lane_b);
  return map;
}

// Two parallel lanes with a seam at x=10, both x=[0,20]. Lane A (y=[-2,2]) is a1 -> a2, and lane B
// (y=[2,6]) is b1 -> b2. The lanes share no points, so a2 is the only next lane of a1, and b2 of
// b1.
inline lanelet::LaneletMapPtr make_two_lane_seam_map(
  lanelet::Id & id_a1, lanelet::Id & id_a2, lanelet::Id & id_b1, lanelet::Id & id_b2)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  auto add_lane =
    [&map](double y_right, double y_left, lanelet::Id & first_id, lanelet::Id & second_id) {
      lanelet::Point3d l0(lanelet::utils::getId(), 0.0, y_left, 0.0);
      lanelet::Point3d l1(lanelet::utils::getId(), 10.0, y_left, 0.0);
      lanelet::Point3d l2(lanelet::utils::getId(), 20.0, y_left, 0.0);
      lanelet::Point3d r0(lanelet::utils::getId(), 0.0, y_right, 0.0);
      lanelet::Point3d r1(lanelet::utils::getId(), 10.0, y_right, 0.0);
      lanelet::Point3d r2(lanelet::utils::getId(), 20.0, y_right, 0.0);
      lanelet::Lanelet first(
        lanelet::utils::getId(), lanelet::LineString3d(lanelet::utils::getId(), {l0, l1}),
        lanelet::LineString3d(lanelet::utils::getId(), {r0, r1}));
      lanelet::Lanelet second(
        lanelet::utils::getId(), lanelet::LineString3d(lanelet::utils::getId(), {l1, l2}),
        lanelet::LineString3d(lanelet::utils::getId(), {r1, r2}));
      mark_road(first);
      mark_road(second);
      first_id = first.id();
      second_id = second.id();
      map->add(first);
      map->add(second);
    };
  add_lane(-2.0, 2.0, id_a1, id_a2);
  add_lane(2.0, 6.0, id_b1, id_b2);
  return map;
}

// Single straight lanelet (x=[0,10], y=[-2,2]) tagged as a turn-direction lane.
inline lanelet::LaneletMapPtr make_turn_lane_map(lanelet::Id & lane_id_out)
{
  auto map = make_single_lane_map(lane_id_out);
  auto lane = map->laneletLayer.get(lane_id_out);
  lane.attributes()["turn_direction"] = "left";
  return map;
}

// Single straight lanelet whose LEFT bound (y=2) is a virtual linestring, RIGHT bound (y=-2) solid.
inline lanelet::LaneletMapPtr make_virtual_left_bound_map(lanelet::Id & lane_id_out)
{
  lanelet::Point3d lp1(lanelet::utils::getId(), 0.0, 2.0, 0.0);
  lanelet::Point3d lp2(lanelet::utils::getId(), 10.0, 2.0, 0.0);
  lanelet::Point3d rp1(lanelet::utils::getId(), 0.0, -2.0, 0.0);
  lanelet::Point3d rp2(lanelet::utils::getId(), 10.0, -2.0, 0.0);
  lanelet::LineString3d left_bound(lanelet::utils::getId(), {lp1, lp2});
  left_bound.attributes()["type"] = "virtual";
  lanelet::LineString3d right_bound(lanelet::utils::getId(), {rp1, rp2});
  right_bound.attributes()["type"] = "line_thin";
  lanelet::Lanelet lane(lanelet::utils::getId(), left_bound, right_bound);
  lane_id_out = lane.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  return map;
}

// A road lane (y=[-2,2]) with a road-shoulder lanelet (y=[2,4]) beside it, sharing the y=2 bound.
inline lanelet::LaneletMapPtr make_road_and_shoulder_map(
  lanelet::Id & road_id, lanelet::Id & shoulder_id)
{
  lanelet::Point3d road_rp1(lanelet::utils::getId(), 0.0, -2.0, 0.0);
  lanelet::Point3d road_rp2(lanelet::utils::getId(), 10.0, -2.0, 0.0);
  lanelet::Point3d shared1(lanelet::utils::getId(), 0.0, 2.0, 0.0);
  lanelet::Point3d shared2(lanelet::utils::getId(), 10.0, 2.0, 0.0);
  lanelet::Point3d shoulder_lp1(lanelet::utils::getId(), 0.0, 4.0, 0.0);
  lanelet::Point3d shoulder_lp2(lanelet::utils::getId(), 10.0, 4.0, 0.0);

  lanelet::LineString3d road_right(lanelet::utils::getId(), {road_rp1, road_rp2});
  lanelet::LineString3d shared_bound(lanelet::utils::getId(), {shared1, shared2});
  lanelet::LineString3d shoulder_left(lanelet::utils::getId(), {shoulder_lp1, shoulder_lp2});

  lanelet::Lanelet road(lanelet::utils::getId(), shared_bound, road_right);
  mark_road(road);
  lanelet::Lanelet shoulder(lanelet::utils::getId(), shoulder_left, shared_bound);
  shoulder.attributes()[lanelet::AttributeName::Subtype] = "road_shoulder";

  road_id = road.id();
  shoulder_id = shoulder.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(road);
  map->add(shoulder);
  return map;
}

// A line through the points with the given type and subtype attributes.
inline lanelet::LineString3d make_typed_line(
  const lanelet::Points3d & points, const std::string & type, const std::string & subtype)
{
  lanelet::LineString3d line(lanelet::utils::getId(), points);
  line.attributes()[lanelet::AttributeName::Type] = type;
  line.attributes()[lanelet::AttributeName::Subtype] = subtype;
  return line;
}

// Options of make_bypass_map. The defaults give the shape of the Odaiba bypass (PDDP-886).
struct BypassMapOptions
{
  double branch_length_m{40.0};      // B moves 4 m to the right: 40 m is about 5.7 deg, 4 m 45 deg
  double sibling_length_m{40.0};     // S runs from x=20 to x=20+sibling_length_m, S1 on to x=100
  double branch_side_shift_m{4.0};   // how far B (and N) end to the right of R
  double sibling_side_shift_m{0.0};  // how far S (and S1) end to the left of R
  bool is_sibling_shoulder{false};   // S and S1 are road shoulders
  bool is_crossed_line_virtual{false};  // the right bounds of S and S1 are virtual
  bool has_branch_virtual_bound{true};  // the left bound of B is virtual, else a dashed line
  std::string branch_turn_direction;    // turn_direction of B; empty for no tag
  int n_lane_count{1};                  // N (20 m) is split into this many lanelets of equal length
};

// Lanelet ids of make_bypass_map and make_bypass_merge_map.
struct BypassMap
{
  lanelet::LaneletMapPtr map;
  lanelet::Id r{};   // lane before the split: x=[0,20], y=[-2,2]
  lanelet::Id s{};   // straight successor of R (the other successor), y=[-2,2]
  lanelet::Id s1{};  // successor of S, on to x=100
  lanelet::Id b{};   // branch: from the end of R, 4 m to the right, across the right bound of S
  lanelet::Id n{};   // successor of B: y=[-6,-2], 20 m long
  lanelet::Id q{};   // merge map only: the right lane before M, x=[0,20], y=[-6,-2]
  std::vector<lanelet::Id> lanes_after_n;  // the other lanelets of N in order, when N is split
};

// Lane R splits at x=20 into the straight lane S and the branch B. B runs diagonally to the right
// across the right bound of S (line_thin dashed) and ends in lane N.
inline BypassMap make_bypass_map(const BypassMapOptions & options = {})
{
  using lanelet::Point3d;
  using lanelet::utils::getId;
  constexpr double split_x = 20.0;
  constexpr double end_x = 100.0;
  const double s_end_x = split_x + options.sibling_length_m;
  const double b_end_x = split_x + options.branch_length_m;

  const Point3d r_start_l(getId(), 0.0, 2.0, 0.0);
  const Point3d r_start_r(getId(), 0.0, -2.0, 0.0);
  const Point3d split_l(getId(), split_x, 2.0, 0.0);
  const Point3d split_r(getId(), split_x, -2.0, 0.0);
  const double s_shift = options.sibling_side_shift_m;
  const double b_shift = options.branch_side_shift_m;
  const Point3d s_end_l(getId(), s_end_x, 2.0 + s_shift, 0.0);
  const Point3d s_end_r(getId(), s_end_x, -2.0 + s_shift, 0.0);
  const Point3d end_l(getId(), end_x, 2.0 + s_shift, 0.0);
  const Point3d end_r(getId(), end_x, -2.0 + s_shift, 0.0);
  const Point3d b_end_l(getId(), b_end_x, 2.0 - b_shift, 0.0);
  const Point3d b_end_r(getId(), b_end_x, -2.0 - b_shift, 0.0);
  const Point3d n_end_l(getId(), b_end_x + 20.0, 2.0 - b_shift, 0.0);
  const Point3d n_end_r(getId(), b_end_x + 20.0, -2.0 - b_shift, 0.0);

  const std::string crossed_type = options.is_crossed_line_virtual ? "virtual" : "line_thin";
  lanelet::Lanelet r(
    getId(), make_typed_line({r_start_l, split_l}, "line_thin", "solid"),
    make_typed_line({r_start_r, split_r}, "line_thin", "solid"));
  lanelet::Lanelet s(
    getId(), make_typed_line({split_l, s_end_l}, "line_thin", "solid"),
    make_typed_line({split_r, s_end_r}, crossed_type, "dashed"));
  lanelet::Lanelet s1(
    getId(), make_typed_line({s_end_l, end_l}, "line_thin", "solid"),
    make_typed_line({s_end_r, end_r}, crossed_type, "dashed"));
  lanelet::Lanelet b(
    getId(),
    make_typed_line(
      {split_l, b_end_l}, options.has_branch_virtual_bound ? "virtual" : "line_thin", "dashed"),
    make_typed_line({split_r, b_end_r}, "line_thin", "solid"));
  // N is split into n_lane_count lanelets of equal length. The first one is n.
  const int n_lane_count = options.n_lane_count > 1 ? options.n_lane_count : 1;
  std::vector<Point3d> n_points_l{b_end_l};
  std::vector<Point3d> n_points_r{b_end_r};
  for (int i = 1; i < n_lane_count; ++i) {
    const double x = b_end_x + 20.0 * i / n_lane_count;
    n_points_l.emplace_back(getId(), x, 2.0 - b_shift, 0.0);
    n_points_r.emplace_back(getId(), x, -2.0 - b_shift, 0.0);
  }
  n_points_l.push_back(n_end_l);
  n_points_r.push_back(n_end_r);
  std::vector<lanelet::Lanelet> n_lanes;
  for (std::size_t i = 0; i + 1 < n_points_l.size(); ++i) {
    n_lanes.emplace_back(
      getId(), make_typed_line({n_points_l[i], n_points_l[i + 1]}, "line_thin", "dashed"),
      make_typed_line({n_points_r[i], n_points_r[i + 1]}, "line_thin", "solid"));
  }
  for (auto * lane : {&r, &s, &s1, &b}) {
    mark_road(*lane);
  }
  for (auto & lane : n_lanes) {
    mark_road(lane);
  }
  if (options.is_sibling_shoulder) {
    s.attributes()[lanelet::AttributeName::Subtype] = "road_shoulder";
    s1.attributes()[lanelet::AttributeName::Subtype] = "road_shoulder";
  }
  if (!options.branch_turn_direction.empty()) {
    b.attributes()["turn_direction"] = options.branch_turn_direction;
  }

  BypassMap result;
  result.map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & lane : {r, s, s1, b}) {
    result.map->add(lane);
  }
  for (const auto & lane : n_lanes) {
    result.map->add(lane);
  }
  result.r = r.id();
  result.s = s.id();
  result.s1 = s1.id();
  result.b = b.id();
  result.n = n_lanes.front().id();
  for (std::size_t i = 1; i < n_lanes.size(); ++i) {
    result.lanes_after_n.push_back(n_lanes[i].id());
  }
  return result;
}

// A merge, not a branch: lane Q (x=[0,20], y=[-6,-2]) continues into M, which runs diagonally to
// the left across the right bound of S (line_thin dashed) and joins S1 at x=60. M and S have no
// shared predecessor. R -> S -> S1 is the straight lane, as in make_bypass_map.
inline BypassMap make_bypass_merge_map()
{
  using lanelet::Point3d;
  using lanelet::utils::getId;
  const Point3d r_start_l(getId(), 0.0, 2.0, 0.0);
  const Point3d r_start_r(getId(), 0.0, -2.0, 0.0);
  const Point3d split_l(getId(), 20.0, 2.0, 0.0);
  const Point3d split_r(getId(), 20.0, -2.0, 0.0);
  const Point3d s_end_l(getId(), 60.0, 2.0, 0.0);
  const Point3d s_end_r(getId(), 60.0, -2.0, 0.0);
  const Point3d end_l(getId(), 100.0, 2.0, 0.0);
  const Point3d end_r(getId(), 100.0, -2.0, 0.0);
  const Point3d q_start_l(getId(), 0.0, -2.0, 0.0);
  const Point3d q_start_r(getId(), 0.0, -6.0, 0.0);
  const Point3d q_end_l(getId(), 20.0, -2.0, 0.0);
  const Point3d q_end_r(getId(), 20.0, -6.0, 0.0);

  lanelet::Lanelet r(
    getId(), make_typed_line({r_start_l, split_l}, "line_thin", "solid"),
    make_typed_line({r_start_r, split_r}, "line_thin", "dashed"));
  lanelet::Lanelet s(
    getId(), make_typed_line({split_l, s_end_l}, "line_thin", "solid"),
    make_typed_line({split_r, s_end_r}, "line_thin", "dashed"));
  lanelet::Lanelet s1(
    getId(), make_typed_line({s_end_l, end_l}, "line_thin", "solid"),
    make_typed_line({s_end_r, end_r}, "line_thin", "solid"));
  lanelet::Lanelet q(
    getId(), make_typed_line({q_start_l, q_end_l}, "line_thin", "dashed"),
    make_typed_line({q_start_r, q_end_r}, "line_thin", "solid"));
  lanelet::Lanelet m(
    getId(), make_typed_line({q_end_l, s_end_l}, "virtual", "dashed"),
    make_typed_line({q_end_r, s_end_r}, "line_thin", "solid"));
  for (auto * lane : {&r, &s, &s1, &q, &m}) {
    mark_road(*lane);
  }

  BypassMap result;
  result.map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & lane : {r, s, s1, q, m}) {
    result.map->add(lane);
  }
  result.r = r.id();
  result.s = s.id();
  result.s1 = s1.id();
  result.b = m.id();
  result.q = q.id();
  return result;
}

// stamp_sec / stamp_nanosec passed as integers to avoid float-to-nanosec rounding.
inline LaneEventInput make_input(
  const std::vector<lanelet::Id> & route_lane_ids, double x, double y, int32_t stamp_sec,
  uint32_t stamp_nanosec)
{
  LaneEventInput input;

  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->pose.pose.position.x = x;
  odom->pose.pose.position.y = y;
  odom->header.stamp.sec = stamp_sec;
  odom->header.stamp.nanosec = stamp_nanosec;
  input.odometry_ptr = odom;

  auto route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
  route->segments.reserve(route_lane_ids.size());
  for (const auto id : route_lane_ids) {
    autoware_planning_msgs::msg::LaneletSegment seg;
    seg.preferred_primitive.id = id;
    route->segments.push_back(seg);
  }
  input.route_ptr = route;
  input.footprint = compute_footprint(input);  // vehicle_info null → single centre point
  return input;
}

// An axis-aligned bounding-box object at (x, y), static unless a speed is given.
inline autoware_perception_msgs::msg::PredictedObject make_object(
  double x, double y, double size_x = 2.0, double size_y = 2.0, double speed_mps = 0.0,
  uint8_t label = autoware_perception_msgs::msg::ObjectClassification::CAR)
{
  autoware_perception_msgs::msg::PredictedObject object;
  // Classified by default: ignored_object_labels filters UNKNOWN, so an unclassified object would
  // never qualify as a candidate.
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = speed_mps;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = size_x;
  object.shape.dimensions.y = size_y;
  object.shape.dimensions.z = 1.5;
  return object;
}

inline autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr make_objects(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects)
{
  auto message = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  message->objects = objects;
  return message;
}

// Builds an input with an explicit trajectory, footprint, turn indicator, and perceived objects.
inline LaneEventInput make_trajectory_input(
  const std::vector<lanelet::Id> & route_lane_ids, const lanelet::BasicPoint2d & ego,
  int32_t stamp_sec, uint32_t stamp_nanosec,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points,
  const std::vector<lanelet::BasicPoint2d> & footprint,
  uint8_t turn_indicator = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE,
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr = nullptr)
{
  LaneEventInput input;

  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->pose.pose.position.x = ego.x();
  odom->pose.pose.position.y = ego.y();
  odom->header.stamp.sec = stamp_sec;
  odom->header.stamp.nanosec = stamp_nanosec;
  input.odometry_ptr = odom;

  auto route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
  route->segments.reserve(route_lane_ids.size());
  for (const auto id : route_lane_ids) {
    autoware_planning_msgs::msg::LaneletSegment seg;
    seg.preferred_primitive.id = id;
    route->segments.push_back(seg);
  }
  input.route_ptr = route;

  auto trajectory = std::make_shared<autoware_planning_msgs::msg::Trajectory>();
  trajectory->header.stamp.sec = stamp_sec;
  trajectory->header.stamp.nanosec = stamp_nanosec;
  trajectory->points.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
    trajectory_point.pose.position.x = point.x();
    trajectory_point.pose.position.y = point.y();
    trajectory->points.push_back(trajectory_point);
  }
  input.trajectory_ptr = trajectory;

  input.footprint = footprint;
  input.turn_indicator = turn_indicator;
  input.objects_ptr = objects_ptr;
  return input;
}

inline constexpr uint32_t ms(uint32_t milliseconds)
{
  return milliseconds * 1'000'000u;
}

}  // namespace lane_event_classifier::test_maps

#endif  // SYNTHETIC_LANELET_MAPS_HPP_
