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

#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <lane_event_classifier/detail/geometry_utils.hpp>
#include <lane_event_classifier/lane_change/geometry.hpp>

#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lane_event_classifier
{

namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;
using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::Point2d;

// A point range (trajectory samples or a lanelet bound) as a boost linestring for intersection.
template <class PointRange>
LineString2d to_line_string(const PointRange & points)
{
  LineString2d line_string;
  line_string.reserve(points.size());
  for (const auto & point : points) {
    line_string.emplace_back(point.x(), point.y());
  }
  return line_string;
}

// Arc length along the polyline to the point of it nearest to the query point.
double arc_length_at(
  const std::vector<lanelet::BasicPoint2d> & points, const lanelet::BasicPoint2d & query)
{
  double best_distance = std::numeric_limits<double>::max();
  double best_arc_m = 0.0;
  double arc_m = 0.0;
  for (std::size_t index = 0; index + 1 < points.size(); ++index) {
    const lanelet::BasicPoint2d segment = points[index + 1] - points[index];
    const double length = segment.norm();
    const double t =
      length > 0.0 ? std::clamp((query - points[index]).dot(segment) / (length * length), 0.0, 1.0)
                   : 0.0;
    const double distance = (points[index] + segment * t - query).norm();
    if (distance < best_distance) {
      best_distance = distance;
      best_arc_m = arc_m + t * length;
    }
    arc_m += length;
  }
  return best_arc_m;
}

// Direction of the line over window_m of arc before and after the point of it nearest to point.
lanelet::BasicPoint2d direction_near(
  const lanelet::ConstLineString2d & line, const lanelet::BasicPoint2d & point, double window_m)
{
  const double arc_m = lanelet::geometry::toArcCoordinates(line, point).length;
  const double length = lanelet::geometry::length(line);
  const lanelet::BasicPoint2d ahead =
    lanelet::geometry::interpolatedPointAtDistance(line, std::min(length, arc_m + window_m));
  const lanelet::BasicPoint2d behind =
    lanelet::geometry::interpolatedPointAtDistance(line, std::max(0.0, arc_m - window_m));
  return ahead - behind;
}

// Angle between two directions in degrees, folded into [0, 90].
double crossing_angle_deg(const lanelet::BasicPoint2d & a, const lanelet::BasicPoint2d & b)
{
  const double norms = a.norm() * b.norm();
  if (norms <= 0.0) {
    return 90.0;
  }
  return std::acos(std::clamp(std::abs(a.dot(b)) / norms, 0.0, 1.0)) * 180.0 / M_PI;
}

// The lane and its successors that start within reach_m of the start of the lane.
std::vector<lanelet::ConstLanelet> lanes_within(
  const LaneTracker & tracker, lanelet::Id first_lane_id, double reach_m)
{
  std::vector<lanelet::ConstLanelet> lanes;
  std::vector<std::pair<lanelet::Id, double>> open{{first_lane_id, 0.0}};  // lane id, start [m]
  while (!open.empty()) {
    const auto [lane_id, start_m] = open.back();
    open.pop_back();
    const auto lane = tracker.get_lanelet(lane_id);
    if (!lane || start_m >= reach_m) {
      continue;
    }
    lanes.push_back(*lane);
    const double end_m = start_m + lanelet::geometry::length2d(*lane);
    for (const auto next_lane_id : tracker.next_lane_ids(lane_id)) {
      open.emplace_back(next_lane_id, end_m);
    }
  }
  return lanes;
}

// A painted line of the other successor S (or of a successor of S) that the branch B crosses.
struct BypassLine
{
  lanelet::Id branch_lane_id{lanelet::InvalId};
  lanelet::ConstLineString2d line;
  bool is_to_left{false};  // the line is the left bound of its lane, so the crossing is to the left
  std::vector<lanelet::Id> other_lane_ids;  // S and its successors within the length of B
};

// The bypass lines of the reference lane (docs/lane_change.md, "Bypass branch"): a successor B on
// the route with a virtual bound crosses a non-virtual bound of another successor S (not a road
// shoulder), or of a successor of S within the length of B, at less than max_angle_deg.
std::vector<BypassLine> find_bypass_lines(
  const LaneTracker & tracker, lanelet::Id reference_lane_id, double max_angle_deg)
{
  constexpr double angle_window_m = 1.0;
  std::vector<BypassLine> bypass_lines;
  const auto next_lane_ids = tracker.next_lane_ids(reference_lane_id);
  if (next_lane_ids.size() < 2) {
    return bypass_lines;
  }
  for (const auto branch_lane_id : next_lane_ids) {
    const auto branch = tracker.get_lanelet(branch_lane_id);
    if (
      !branch || !tracker.is_route_primitive(branch_lane_id) ||
      (!is_virtual_linestring(branch->leftBound()) &&
       !is_virtual_linestring(branch->rightBound()))) {
      continue;
    }
    const auto branch_centerline = branch->centerline2d();
    const auto branch_line = to_line_string(branch_centerline);
    const double branch_length_m = lanelet::geometry::length(branch_centerline);
    for (const auto other_lane_id : next_lane_ids) {
      if (other_lane_id == branch_lane_id) {
        continue;
      }
      const auto other_lanes = lanes_within(tracker, other_lane_id, branch_length_m);
      std::vector<lanelet::Id> other_lane_ids;
      other_lane_ids.reserve(other_lanes.size());
      for (const auto & other_lane : other_lanes) {
        other_lane_ids.push_back(other_lane.id());
      }
      for (const auto & owner : other_lanes) {
        if (lanelet2_utils::is_shoulder_lane(owner)) {
          continue;
        }
        for (const bool is_left : {true, false}) {
          if (is_virtual_linestring(is_left ? owner.leftBound() : owner.rightBound())) {
            continue;
          }
          const auto bound = is_left ? owner.leftBound2d() : owner.rightBound2d();
          std::vector<Point2d> hits;
          boost::geometry::intersection(branch_line, to_line_string(bound), hits);
          if (hits.empty()) {
            continue;
          }
          const lanelet::BasicPoint2d hit{hits.front().x(), hits.front().y()};
          const double angle_deg = crossing_angle_deg(
            direction_near(branch_centerline, hit, angle_window_m),
            direction_near(bound, hit, angle_window_m));
          if (angle_deg < max_angle_deg) {
            bypass_lines.push_back({branch_lane_id, bound, is_left, other_lane_ids});
          }
        }
      }
    }
  }
  return bypass_lines;
}

// Outcome of scanning the trajectory for the reference lane's lateral boundary crossing.
struct TrajectoryCrossingScan
{
  std::optional<lanelet::Id> target_lane_id;  // first off-sequence route-preferred lane entered
  std::optional<lanelet::BasicPoint2d> crossing_point;  // first sample outside the lane sequence
};

// Scans forward for the first off-sequence sample; see docs/lane_change.md, "Finding a crossing".
TrajectoryCrossingScan scan_trajectory_for_crossing(
  const LaneTracker & tracker, const std::unordered_set<lanelet::Id> & lane_sequence,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points)
{
  TrajectoryCrossingScan scan;
  for (const auto & point : trajectory_points) {
    bool is_in_lane_sequence = false;
    bool has_off_sequence_lane = false;
    for (const auto id : tracker.lanelet_ids_at(point)) {
      if (lane_sequence.count(id) != 0) {
        is_in_lane_sequence = true;
        continue;
      }
      has_off_sequence_lane = true;
      // The target must be route-preferred, otherwise the settle check can never confirm it.
      if (!scan.target_lane_id && tracker.is_route_primitive(id)) {
        scan.target_lane_id = id;
      }
    }

    // First off-sequence sample marks the lateral boundary crossing.
    if (!scan.crossing_point && !is_in_lane_sequence && has_off_sequence_lane) {
      scan.crossing_point = point;
    }
    // Remaining samples cannot change the outcome.
    if (scan.target_lane_id && scan.crossing_point) {
      break;
    }
  }
  return scan;
}
}  // namespace

LaneChangeGeometry::LaneChangeGeometry(
  double crossing_look_ahead_m, double bypass_max_crossing_angle_deg)
: crossing_look_ahead_m_{crossing_look_ahead_m},
  bypass_max_crossing_angle_deg_{bypass_max_crossing_angle_deg}
{
}

LaneChangeObservation LaneChangeGeometry::observe(
  const LaneTracker & tracker, const LaneEventInput & input, const LaneEventContext & context,
  const std::optional<BypassLanes> & active_bypass) const
{
  // Compute the two per-cycle intermediates once and share them across the helpers below.
  const auto trajectory_points =
    forward_trajectory_points_from_input(input, crossing_look_ahead_m_);
  const auto & footprint_ids = context.footprint_lane_ids();

  LaneChangeObservation observation;
  observation.crossing = compute_crossing(tracker, input, context, trajectory_points);
  // A bypass event tests the abort against the stored origin lane (docs/lane_change.md, "Bypass
  // branch"): the held lane can move into the branch when the branch has no turn_direction.
  const auto origin_lane_id =
    active_bypass ? active_bypass->origin_lane_id : tracker.reference_lane().reference_lane_id;
  observation.trajectory_returns_to_reference = compute_trajectory_returns_to_reference(
    tracker, origin_lane_id, observation.crossing.has_value(), trajectory_points);
  observation.is_footprint_off_route_primitives =
    is_footprint_off_route_primitives(tracker, input, footprint_ids);
  observation.is_footprint_inside_reference_lane =
    tracker.is_footprint_fully_inside_lane(origin_lane_id, input.footprint);
  observation.settle_lane_id = compute_settle_lane_id(tracker, input, footprint_ids);
  observation.is_footprint_clear_of_bypass_lanes =
    active_bypass && is_footprint_clear_of_lanes(tracker, input, active_bypass->covered_lane_ids);
  return observation;
}

std::optional<LaneChangeCrossing> LaneChangeGeometry::compute_crossing(
  const LaneTracker & tracker, const LaneEventInput & input, const LaneEventContext & context,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points) const
{
  if (trajectory_points.empty() || !input.route_ptr) {
    return std::nullopt;
  }
  // Reference-lane attributes were resolved by the tracker when it anchored onto the lane.
  const auto & reference = tracker.reference_lane();
  const auto reference_lane_id = reference.reference_lane_id;

  // Bypass branch (docs/lane_change.md, "Bypass branch"): the route branch is a successor of the
  // reference lane, so this check runs before the straight-on-route skip.
  if (auto bypass_crossing = compute_bypass_crossing(tracker, trajectory_points)) {
    return bypass_crossing;
  }

  // Straight-on-route skip; see docs/lane_change.md, "Finding a crossing".
  if (driving_straight_stays_on_route(tracker, reference_lane_id)) {
    return std::nullopt;
  }

  const auto reference_lane_opt = tracker.get_lanelet(reference_lane_id);
  if (!reference_lane_opt) {
    return std::nullopt;
  }

  const auto & reference_lane = *reference_lane_opt;

  if (reference.is_reference_lane_road_shoulder) {
    return std::nullopt;
  }

  const auto & lane_sequence = context.sequence_ids(crossing_look_ahead_m_);

  const auto [target_lane_id, crossing_point] =
    scan_trajectory_for_crossing(tracker, lane_sequence, trajectory_points);
  if (!target_lane_id || !crossing_point) {
    return std::nullopt;
  }

  // Onset exemptions (docs/lane_change.md, "Exemptions"); turn lanes carry turn_direction.
  if (reference.is_reference_lane_intersection) {
    return std::nullopt;
  }

  const auto target_lane_opt = tracker.get_lanelet(*target_lane_id);
  if (target_lane_opt && lanelet2_utils::is_shoulder_lane(*target_lane_opt)) {
    return std::nullopt;
  }

  // Signed offset from the centerline; positive means left.
  const double lateral_offset =
    lanelet::geometry::toArcCoordinates(reference_lane.centerline2d(), *crossing_point).distance;
  const bool is_to_left = lateral_offset > 0.0;

  // Onset exemption; see docs/lane_change.md, "Exemptions".
  const bool is_crossed_bound_virtual = is_to_left
                                          ? reference.is_reference_lane_left_bound_virtual
                                          : reference.is_reference_lane_right_bound_virtual;
  if (is_crossed_bound_virtual) {
    return std::nullopt;
  }

  LaneChangeCrossing crossing;
  crossing.target_lane_id = *target_lane_id;
  crossing.crossing_point = *crossing_point;
  crossing.is_to_left = is_to_left;
  return crossing;
}

std::optional<LaneChangeCrossing> LaneChangeGeometry::compute_bypass_crossing(
  const LaneTracker & tracker, const std::vector<lanelet::BasicPoint2d> & trajectory_points) const
{
  if (trajectory_points.size() < 2) {
    return std::nullopt;
  }
  const auto bypass_lines = find_bypass_lines(
    tracker, tracker.reference_lane().reference_lane_id, bypass_max_crossing_angle_deg_);
  if (bypass_lines.empty()) {
    return std::nullopt;
  }
  // The first crossing along the trajectory sets the onset (several lines can match).
  const auto trajectory_line = to_line_string(trajectory_points);
  std::optional<LaneChangeCrossing> first_crossing;
  double first_arc_m = std::numeric_limits<double>::max();
  for (const auto & bypass_line : bypass_lines) {
    std::vector<Point2d> hits;
    boost::geometry::intersection(trajectory_line, to_line_string(bypass_line.line), hits);
    for (const auto & hit : hits) {
      const lanelet::BasicPoint2d point{hit.x(), hit.y()};
      const double arc_m = arc_length_at(trajectory_points, point);
      if (arc_m < first_arc_m) {
        first_arc_m = arc_m;
        first_crossing = LaneChangeCrossing{
          bypass_line.branch_lane_id, point, bypass_line.is_to_left, std::nullopt};
      }
    }
  }
  if (!first_crossing) {
    return std::nullopt;
  }
  // The end waits for R and every lane whose line the branch crosses (several lines can match).
  BypassLanes bypass_lanes;
  bypass_lanes.origin_lane_id = tracker.reference_lane().reference_lane_id;
  bypass_lanes.branch_lane_id = first_crossing->target_lane_id;
  bypass_lanes.covered_lane_ids.push_back(bypass_lanes.origin_lane_id);
  for (const auto & bypass_line : bypass_lines) {
    if (bypass_line.branch_lane_id != bypass_lanes.branch_lane_id) {
      continue;
    }
    for (const auto lane_id : bypass_line.other_lane_ids) {
      const auto & covered = bypass_lanes.covered_lane_ids;
      if (std::find(covered.cbegin(), covered.cend(), lane_id) == covered.cend()) {
        bypass_lanes.covered_lane_ids.push_back(lane_id);
      }
    }
  }
  first_crossing->bypass = std::move(bypass_lanes);
  return first_crossing;
}

bool LaneChangeGeometry::compute_trajectory_returns_to_reference(
  const LaneTracker & tracker, lanelet::Id origin_lane_id, bool has_forward_crossing,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points)
{
  // A forward crossing means the trajectory is still heading out, not back.
  if (has_forward_crossing || trajectory_points.empty()) {
    return false;
  }
  const auto reference_lane_opt = tracker.get_lanelet(origin_lane_id);
  if (!reference_lane_opt) {
    return false;
  }
  // The far look-ahead point is back inside the reference lane.
  return lanelet::geometry::inside(*reference_lane_opt, trajectory_points.back());
}

bool LaneChangeGeometry::is_footprint_clear_of_lanes(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<lanelet::Id> & lane_ids)
{
  // Bypass end (docs/lane_change.md, "Bypass branch"): the whole body is past the crossed lines.
  return std::none_of(
    input.footprint.cbegin(), input.footprint.cend(), [&](const lanelet::BasicPoint2d & point) {
      const auto ids_at_point = tracker.lanelet_ids_at(point);
      return std::any_of(ids_at_point.cbegin(), ids_at_point.cend(), [&](const lanelet::Id id) {
        return std::find(lane_ids.cbegin(), lane_ids.cend(), id) != lane_ids.cend();
      });
    });
}

bool LaneChangeGeometry::is_footprint_off_route_primitives(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (!input.route_ptr || footprint_ids.empty()) {
    return false;
  }

  // "Off the route primitives" == no footprint corner lies inside any route-preferred primitive.
  return std::none_of(
    footprint_ids.cbegin(), footprint_ids.cend(),
    [&tracker](const lanelet::Id id) { return tracker.is_route_primitive(id); });
}

std::optional<lanelet::Id> LaneChangeGeometry::compute_settle_lane_id(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (!input.route_ptr) {
    return std::nullopt;
  }
  const auto reference_lane_id = tracker.reference_lane().reference_lane_id;

  // A settle candidate is a route-preferred primitive other than the reference lane
  const auto settled =
    std::find_if(footprint_ids.cbegin(), footprint_ids.cend(), [&](const lanelet::Id candidate_id) {
      return candidate_id != reference_lane_id && tracker.is_route_primitive(candidate_id) &&
             tracker.is_footprint_fully_inside_lane(candidate_id, input.footprint);
    });
  if (settled == footprint_ids.cend()) {
    return std::nullopt;
  }
  return *settled;
}

}  // namespace lane_event_classifier
