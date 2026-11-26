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

// Trajectory-driven lane-change tests on the real test map (see docs/lane_change.md).

#include "synthetic_lanelet_maps.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <lane_event_classifier/detail/lane_tracker.hpp>
#include <lane_event_classifier/lane_change/classifier.hpp>
#include <lane_event_classifier/lane_following/checker.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <lane_event_classifier_msgs/msg/driving_state.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace lane_event_classifier
{

namespace
{
using DS = lane_event_classifier_msgs::msg::DrivingState;
using TurnIndicatorsReport = autoware_vehicle_msgs::msg::TurnIndicatorsReport;

constexpr std::array<lanelet::Id, 4> route_ids()
{
  return {47, 1167, 51, 55};  // preferred-primitive sequence
}

lanelet::LaneletMapPtr load_test_map()
{
  const auto map_const =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(TEST_MAP_PATH);
  return map_const ? autoware::experimental::lanelet2_utils::remove_const(map_const) : nullptr;
}

std::vector<lanelet::BasicPoint2d> centerline_points(
  const lanelet::LaneletMapPtr & map, lanelet::Id id)
{
  const auto centerline = map->laneletLayer.get(id).centerline2d();
  std::vector<lanelet::BasicPoint2d> points;
  points.reserve(centerline.size());
  for (const auto & point : centerline) {
    points.emplace_back(point.x(), point.y());
  }
  return points;
}

lanelet::BasicPoint2d point_at_fraction(
  const std::vector<lanelet::BasicPoint2d> & points, double fraction)
{
  const auto index = static_cast<std::size_t>(
    std::clamp(fraction, 0.0, 1.0) * static_cast<double>(points.size() - 1));
  return points[index];
}

lanelet::BasicPoint2d nearest_point_on(
  const std::vector<lanelet::BasicPoint2d> & points, const lanelet::BasicPoint2d & query)
{
  lanelet::BasicPoint2d nearest = points.front();
  double nearest_distance_sq = std::numeric_limits<double>::max();
  for (const auto & point : points) {
    const double distance_sq = (point - query).squaredNorm();
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest = point;
    }
  }
  return nearest;
}

// Builds a trajectory that sweeps laterally through the listed lanes while advancing forward.
std::vector<lanelet::BasicPoint2d> build_lane_trajectory(
  const lanelet::LaneletMapPtr & map, const std::vector<lanelet::Id> & lane_ids, double start_frac,
  double end_frac, std::size_t point_count)
{
  std::vector<std::vector<lanelet::BasicPoint2d>> centerlines;
  centerlines.reserve(lane_ids.size());
  for (const auto id : lane_ids) {
    centerlines.push_back(centerline_points(map, id));
  }
  const auto & base = centerlines.front();
  const double lane_span = static_cast<double>(lane_ids.size() - 1);

  std::vector<lanelet::BasicPoint2d> trajectory;
  trajectory.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    const double progress = static_cast<double>(index) / static_cast<double>(point_count - 1);
    const double longitudinal_fraction = start_frac + (end_frac - start_frac) * progress;
    const lanelet::BasicPoint2d anchor = point_at_fraction(base, longitudinal_fraction);

    const double lane_position = progress * lane_span;
    const auto lane_index = static_cast<std::size_t>(lane_position);
    const auto next_lane_index = std::min(lane_index + 1, lane_ids.size() - 1);
    const double lateral_fraction = lane_position - static_cast<double>(lane_index);

    const lanelet::BasicPoint2d point_a = nearest_point_on(centerlines[lane_index], anchor);
    const lanelet::BasicPoint2d point_b = nearest_point_on(centerlines[next_lane_index], anchor);
    trajectory.push_back(point_a + (point_b - point_a) * lateral_fraction);
  }
  return trajectory;
}

// A small footprint square around a point (well within a road lane's width).
std::vector<lanelet::BasicPoint2d> footprint_box(const lanelet::BasicPoint2d & center)
{
  constexpr double half = 0.25;
  return {
    {center.x() - half, center.y() - half},
    {center.x() + half, center.y() - half},
    {center.x() + half, center.y() + half},
    {center.x() - half, center.y() + half}};
}

// Drives one cycle exactly as the node does: update, classify, hold or release (re-anchor when an
// event ends), then the lane-following check.
class Simulator
{
public:
  Simulator(lanelet::LaneletMapPtr map, LaneChangeConfig config)
  : classifier_{true, config, tracker_, LaneChangeGeometry{config.crossing_look_ahead_m, config.bypass_max_crossing_angle_deg}},
    lane_following_checker_{LaneFollowingConfig{}, LaneFollowingGeometry{LaneFollowingConfig{}}}
  {
    const auto result = tracker_.set_lanelet_map(map);
    EXPECT_TRUE(result.has_value());
  }

  std::optional<uint8_t> step(const LaneEventInput & input)
  {
    [[maybe_unused]] const auto update_result = tracker_.update(input);
    context_.update(tracker_, input);
    classifier_.update(input, context_);
    state_ = classifier_.get_state();
    const bool is_active = state_ == DS::LANE_CHANGING || state_ == DS::ABORTING_LANE_CHANGE;
    const bool was_held = tracker_.is_reference_lane_held();
    tracker_.apply_reference_lane_hold(is_active);
    if (was_held && !tracker_.is_reference_lane_held()) {
      [[maybe_unused]] const auto reanchor_result = tracker_.update(input);
      context_.update(tracker_, input);
    }
    const auto & ego_position = input.odometry_ptr->pose.pose.position;
    lane_following_result_ =
      lane_following_checker_.evaluate(tracker_, context_, {ego_position.x, ego_position.y});
    return state_;
  }

  [[nodiscard]] lanelet::Id reference_lane_id() const
  {
    return tracker_.reference_lane().reference_lane_id;
  }

  [[nodiscard]] bool is_lane_following() const { return lane_following_result_.is_following; }

  // The state the node publishes for the last cycle: the classifier state when it claims an event,
  // else LANE_FOLLOWING or, for a departure, UNKNOWN.
  [[nodiscard]] uint8_t published_state() const
  {
    if (state_) {
      return *state_;
    }
    return lane_following_result_.is_following ? DS::LANE_FOLLOWING : DS::UNKNOWN;
  }

private:
  LaneTracker tracker_;
  LaneEventContext context_;
  LaneChangeClassifier classifier_;
  LaneFollowingChecker lane_following_checker_;
  LaneFollowingResult lane_following_result_{};
  std::optional<uint8_t> state_;
};

LaneChangeConfig make_config()
{
  LaneChangeConfig config;
  config.enable_classifier = true;
  config.crossing_persist_duration_s = 0.3;
  config.settle_confirm_duration_s = 0.5;
  config.crossing_look_ahead_m = 50.0;
  config.crossing_position_tolerance_m = 2.0;
  config.confidence_factor = 0.5;
  return config;
}

// Splits a running millisecond count into a (sec, nsec) stamp.
std::pair<int32_t, uint32_t> stamp_from_ms(int64_t total_ms)
{
  return {
    static_cast<int32_t>(total_ms / 1000), static_cast<uint32_t>((total_ms % 1000) * 1'000'000)};
}

// The centerlines of the lanes, joined in order, sampled every metre, from start_x on.
std::vector<lanelet::BasicPoint2d> trajectory_along(
  const lanelet::LaneletMapPtr & map, const std::vector<lanelet::Id> & lane_ids, double start_x)
{
  std::vector<lanelet::BasicPoint2d> corners;
  for (const auto id : lane_ids) {
    for (const auto & point : centerline_points(map, id)) {
      if (corners.empty() || (point - corners.back()).norm() > 1e-6) {
        corners.push_back(point);
      }
    }
  }
  std::vector<lanelet::BasicPoint2d> trajectory;
  for (std::size_t index = 0; index + 1 < corners.size(); ++index) {
    const lanelet::BasicPoint2d step = corners[index + 1] - corners[index];
    const auto count = static_cast<int>(std::ceil(step.norm()));
    for (int k = 0; k < count; ++k) {
      const lanelet::BasicPoint2d point = corners[index] + step * (static_cast<double>(k) / count);
      if (point.x() >= start_x) {
        trajectory.push_back(point);
      }
    }
  }
  trajectory.push_back(corners.back());
  return trajectory;
}

// The cycle (100 ms each) in which LANE_CHANGING starts, or nullopt within max_cycles. The ego
// stands still at the first trajectory point, with a one-point footprint.
std::optional<int> lane_change_onset_cycle(
  const lanelet::LaneletMapPtr & map, const std::vector<lanelet::Id> & route,
  const std::vector<lanelet::BasicPoint2d> & trajectory, uint8_t turn_indicator, int max_cycles)
{
  Simulator sim{map, make_config()};
  const auto ego = trajectory.front();
  int64_t time_ms = 0;
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(route, ego, sec, nsec, trajectory, {ego}, turn_indicator));
    if (state == DS::LANE_CHANGING) {
      return cycle;
    }
    time_ms += 100;
  }
  return std::nullopt;
}

// A bus-sized footprint at base_link: 5.7 m ahead, 1.5 m behind, 2.4 m wide, along the heading.
std::vector<lanelet::BasicPoint2d> bus_footprint(
  const lanelet::BasicPoint2d & base_link, const lanelet::BasicPoint2d & heading)
{
  const lanelet::BasicPoint2d forward = heading.normalized();
  const lanelet::BasicPoint2d left{-forward.y(), forward.x()};
  constexpr double front_m = 5.7;
  constexpr double rear_m = 1.5;
  constexpr double half_width_m = 1.2;
  return {
    base_link + forward * front_m + left * half_width_m,
    base_link + forward * front_m - left * half_width_m,
    base_link - forward * rear_m - left * half_width_m,
    base_link - forward * rear_m + left * half_width_m};
}

// One cycle of a drive through the bypass.
struct BypassCycle
{
  std::optional<uint8_t> state;
  uint8_t published_state{DS::UNDEFINED};
  lanelet::Id reference_lane_id{lanelet::InvalId};
  bool is_on_route_before_branch_end{true};  // base_link has not reached the end of B
  bool covers_r_or_other_lanes{false};       // a footprint corner is inside R, S, or S1
  bool covers_line_from_branch{false};       // footprint fully inside B, and a corner inside S
};

// Drives a bus footprint along R -> B -> N at 1 m per 100 ms cycle, waiting wait_cycles at the
// start. The trajectory is the rest of the route path.
std::vector<BypassCycle> drive_through_bypass(const test_maps::BypassMap & bypass, int wait_cycles)
{
  Simulator sim{bypass.map, make_config()};
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  const auto path = trajectory_along(bypass.map, route, 0.0);
  const auto inside = [&bypass](lanelet::Id lane_id, const lanelet::BasicPoint2d & point) {
    return lanelet::geometry::inside(bypass.map->laneletLayer.get(lane_id), point);
  };
  const double branch_end_x = bypass.map->laneletLayer.get(bypass.b).centerline2d().back().x();
  std::vector<BypassCycle> cycles;
  int64_t time_ms = 0;
  std::size_t index = 5;
  for (int cycle = 0; index + 8 < path.size(); ++cycle) {
    const auto & ego = path[index];
    const auto footprint = bus_footprint(ego, path[index + 1] - path[index]);
    const std::vector<lanelet::BasicPoint2d> trajectory(
      path.begin() + static_cast<std::ptrdiff_t>(index), path.end());
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    BypassCycle record;
    record.state =
      sim.step(test_maps::make_trajectory_input(route, ego, sec, nsec, trajectory, footprint));
    record.published_state = sim.published_state();
    record.reference_lane_id = sim.reference_lane_id();
    record.is_on_route_before_branch_end = ego.x() < branch_end_x;
    record.covers_r_or_other_lanes =
      std::any_of(footprint.begin(), footprint.end(), [&](const auto & corner) {
        return inside(bypass.r, corner) || inside(bypass.s, corner) || inside(bypass.s1, corner);
      });
    record.covers_line_from_branch =
      std::all_of(
        footprint.begin(), footprint.end(),
        [&](const auto & corner) { return inside(bypass.b, corner); }) &&
      std::any_of(footprint.begin(), footprint.end(), [&](const auto & corner) {
        return inside(bypass.s, corner);
      });
    cycles.push_back(record);
    time_ms += 100;
    if (cycle >= wait_cycles) {
      ++index;
    }
  }
  return cycles;
}

// Index of the first cycle after start that matches, or cycles.size().
template <class Predicate>
std::size_t first_cycle(
  const std::vector<BypassCycle> & cycles, std::size_t start, const Predicate & predicate)
{
  for (std::size_t index = start; index < cycles.size(); ++index) {
    if (predicate(cycles[index])) {
      return index;
    }
  }
  return cycles.size();
}
}  // namespace

// Single left change 48 -> 47: onset confirms, then the footprint settles inside 47.
TEST(LaneChangeTest, single_left_change_onset_then_settle)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map)) << "failed to load " << TEST_MAP_PATH;

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  // Onset: hold the crossing until LANE_CHANGING is confirmed.
  std::optional<uint8_t> state;
  int64_t time_ms = 0;
  bool became_changing = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_48, sec, nsec, crossing_trajectory, {ego_in_48}));
    if (state == DS::LANE_CHANGING) {
      became_changing = true;
      break;
    }
    time_ms += 100;
  }
  ASSERT_TRUE(became_changing);
  EXPECT_EQ(sim.reference_lane_id(), 48);  // reference held to the origin lane at onset

  // Settle: the footprint sits fully inside route primitive 47.
  const auto point_in_47 = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto footprint_47 = footprint_box(point_in_47);
  const auto settle_trajectory = build_lane_trajectory(map, {47}, 0.5, 0.95, 20);

  bool settled = false;
  for (int cycle = 0; cycle < 12; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_47, sec, nsec, settle_trajectory, footprint_47));
    if (!state) {
      settled = true;
      // The cycle that ends the change already re-anchors to 47, so it publishes LANE_FOLLOWING,
      // not one cycle of UNKNOWN against the old reference lane 48.
      EXPECT_EQ(sim.reference_lane_id(), 47);
      EXPECT_TRUE(sim.is_lane_following());
      EXPECT_EQ(sim.published_state(), DS::LANE_FOLLOWING);
      break;
    }
  }
  EXPECT_TRUE(settled)
    << "footprint fully inside 47 for the settle window should complete the change";
}

// Double left change 50 -> 48 -> 47: stays LANE_CHANGING through 48 and settles only at 47.
TEST(LaneChangeTest, double_left_change_settles_only_at_route_primitive)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {50, 48, 47}, 0.3, 0.9, 30);
  const auto ego_in_50 = crossing_trajectory.front();

  std::optional<uint8_t> state;
  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 10 && state != DS::LANE_CHANGING; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_50, sec, nsec, crossing_trajectory, {ego_in_50}));
    time_ms += 100;
  }
  ASSERT_EQ(state, DS::LANE_CHANGING);

  // Footprint fully inside the intermediate off-route lane 48 must NOT settle.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.6);
  const auto footprint_48 = footprint_box(point_in_48);
  const auto through_48_trajectory = build_lane_trajectory(map, {48, 47}, 0.5, 0.95, 20);
  for (int cycle = 0; cycle < 8; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_48, sec, nsec, through_48_trajectory, footprint_48));
    EXPECT_EQ(state, DS::LANE_CHANGING) << "must not settle in intermediate lane 48";
  }

  // Footprint fully inside route primitive 47 settles.
  const auto point_in_47 = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto footprint_47 = footprint_box(point_in_47);
  const auto settle_trajectory = build_lane_trajectory(map, {47}, 0.5, 0.95, 20);
  bool settled = false;
  for (int cycle = 0; cycle < 12; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_47, sec, nsec, settle_trajectory, footprint_47));
    if (!state) {
      settled = true;
      break;
    }
  }
  EXPECT_TRUE(settled);
}

// Double left change 50 -> 48 -> 47: the reported target is 47, not intermediate lane 48.
TEST(LaneChangeTest, double_left_change_targets_the_route_primitive)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  LaneTracker tracker;
  ASSERT_TRUE(tracker.set_lanelet_map(map).has_value());

  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};
  const auto crossing_trajectory = build_lane_trajectory(map, {50, 48, 47}, 0.3, 0.9, 30);
  const auto ego_in_50 = crossing_trajectory.front();
  const auto input =
    test_maps::make_trajectory_input(route, ego_in_50, 0, 0, crossing_trajectory, {ego_in_50});
  [[maybe_unused]] const auto update_result = tracker.update(input);
  ASSERT_EQ(tracker.reference_lane().reference_lane_id, 50);

  const auto config = make_config();
  const LaneChangeGeometry geometry{
    config.crossing_look_ahead_m, config.bypass_max_crossing_angle_deg};
  LaneEventContext context;
  context.update(tracker, input);
  const auto observation = geometry.observe(tracker, input, context);
  ASSERT_TRUE(observation.crossing.has_value());
  EXPECT_EQ(observation.crossing->target_lane_id, 47);
}

// A crossing from on-route 47 toward off-route 48 is never a lane change (straight-on-route skip).
TEST(LaneChangeTest, on_route_primitive_crossing_off_route_is_not_lane_change)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {47, 48}, 0.3, 0.9, 20);
  const auto ego_in_47 = crossing_trajectory.front();

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, crossing_trajectory, {ego_in_47}));
    EXPECT_NE(state, DS::LANE_CHANGING) << "crossing off-route from an on-route primitive";
    time_ms += 100;
  }
}

// Abort: the trajectory swings back into reference lane 48, then the footprint follows.
TEST(LaneChangeTest, abort_when_trajectory_returns_then_geometric_completion)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  std::optional<uint8_t> state;
  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 10 && state != DS::LANE_CHANGING; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_48, sec, nsec, crossing_trajectory, {ego_in_48}));
    time_ms += 100;
  }
  ASSERT_EQ(state, DS::LANE_CHANGING);

  // Trajectory now stays inside the reference lane 48 (planner gives up).
  const auto return_trajectory = build_lane_trajectory(map, {48}, 0.3, 0.9, 20);
  bool aborting = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_48, sec, nsec, return_trajectory, {ego_in_48}));
    if (state == DS::ABORTING_LANE_CHANGE) {
      aborting = true;
      break;
    }
  }
  ASSERT_TRUE(aborting);

  // Footprint fully back inside the reference lane 48 completes the abort.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.6);
  const auto footprint_48 = footprint_box(point_in_48);
  time_ms += 100;
  const auto [sec, nsec] = stamp_from_ms(time_ms);
  state = sim.step(
    test_maps::make_trajectory_input(
      route, point_in_48, sec, nsec, return_trajectory, footprint_48));
  EXPECT_FALSE(state.has_value()) << "abort completes geometrically with no dwell";
}

// From ABORTING, a trajectory back toward the target returns directly to LANE_CHANGING.
TEST(LaneChangeTest, aborting_recommits_to_changing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto return_trajectory = build_lane_trajectory(map, {48}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  std::optional<uint8_t> state;
  int64_t time_ms = 0;
  auto run = [&](const std::vector<lanelet::BasicPoint2d> & trajectory, int cycles, uint8_t until) {
    for (int cycle = 0; cycle < cycles; ++cycle) {
      const auto [sec, nsec] = stamp_from_ms(time_ms);
      state = sim.step(
        test_maps::make_trajectory_input(route, ego_in_48, sec, nsec, trajectory, {ego_in_48}));
      time_ms += 100;
      if (state == until) {
        return true;
      }
    }
    return false;
  };

  ASSERT_TRUE(run(crossing_trajectory, 10, DS::LANE_CHANGING));
  ASSERT_TRUE(run(return_trajectory, 10, DS::ABORTING_LANE_CHANGE));
  EXPECT_TRUE(run(crossing_trajectory, 10, DS::LANE_CHANGING)) << "re-commit from ABORTING";
}

// Blinker confidence signal: with the footprint straddling 48/47, the blinker shortens onset.
TEST(LaneChangeTest, blinker_confidence_signal_shortens_onset_window)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};
  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  // One corner in 48 and one in 47, so the blinker is the only confidence signal.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.4);
  const auto point_in_47 = nearest_point_on(centerline_points(map, 47), point_in_48);
  const std::vector<lanelet::BasicPoint2d> straddling_footprint{point_in_48, point_in_47};

  auto onset_cycle = [&](uint8_t turn_indicator) {
    Simulator sim{map, make_config()};
    int64_t time_ms = 0;
    for (int cycle = 0; cycle < 20; ++cycle) {
      const auto [sec, nsec] = stamp_from_ms(time_ms);
      const auto state = sim.step(
        test_maps::make_trajectory_input(
          route, ego_in_48, sec, nsec, crossing_trajectory, straddling_footprint, turn_indicator));
      if (state == DS::LANE_CHANGING) {
        return cycle;
      }
      time_ms += 100;
    }
    return 999;
  };

  const int cycles_no_blinker = onset_cycle(TurnIndicatorsReport::DISABLE);
  const int cycles_with_blinker = onset_cycle(TurnIndicatorsReport::ENABLE_LEFT);
  EXPECT_LT(cycles_with_blinker, cycles_no_blinker)
    << "blinker toward the target should shorten the onset window";
}

// An empty footprint must not read as "off the route primitives" and shorten the onset window.
TEST(LaneChangeTest, empty_footprint_is_not_a_confidence_signal)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};
  const auto crossing_trajectory = build_lane_trajectory(map, {48, 47}, 0.3, 0.9, 20);
  const auto ego_in_48 = crossing_trajectory.front();

  // Baseline: footprint straddling 48/47 is genuinely not off the route primitives.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.4);
  const auto point_in_47 = nearest_point_on(centerline_points(map, 47), point_in_48);
  const std::vector<lanelet::BasicPoint2d> straddling_footprint{point_in_48, point_in_47};

  auto onset_cycle = [&](const std::vector<lanelet::BasicPoint2d> & footprint) {
    Simulator sim{map, make_config()};
    int64_t time_ms = 0;
    for (int cycle = 0; cycle < 20; ++cycle) {
      const auto [sec, nsec] = stamp_from_ms(time_ms);
      const auto state = sim.step(
        test_maps::make_trajectory_input(
          route, ego_in_48, sec, nsec, crossing_trajectory, footprint,
          TurnIndicatorsReport::DISABLE));
      if (state == DS::LANE_CHANGING) {
        return cycle;
      }
      time_ms += 100;
    }
    return 999;
  };

  const int cycles_straddling = onset_cycle(straddling_footprint);
  const int cycles_empty_footprint = onset_cycle({});
  EXPECT_EQ(cycles_empty_footprint, cycles_straddling)
    << "an empty footprint must not be read as \"off the route primitives\"";
}

// PDDP-886 bypass: the route takes the branch B, which crosses the dashed right bound of the
// straight lane S at about 5.7 degrees.
TEST(LaneChangeBypassTest, bypass_branch_across_dashed_line_is_lane_change)
{
  const auto bypass = test_maps::make_bypass_map();
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  const auto trajectory = trajectory_along(bypass.map, route, 5.0);
  EXPECT_TRUE(
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::DISABLE, 20))
    << "a branch across a painted line of the other successor is a lane change";
}

// The side comes from the crossed bound: the right bound of S gives a change to the right, so
// only the right blinker shortens the onset wait.
TEST(LaneChangeBypassTest, bypass_branch_side_follows_the_crossed_bound)
{
  const auto bypass = test_maps::make_bypass_map();
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  const auto trajectory = trajectory_along(bypass.map, route, 5.0);
  const auto right =
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::ENABLE_RIGHT, 20);
  const auto left =
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::ENABLE_LEFT, 20);
  ASSERT_TRUE(right);
  ASSERT_TRUE(left);
  EXPECT_LT(*right, *left) << "the crossing is to the right, so only the right blinker counts";
}

// S is short, so the crossed line is the right bound of S1, the successor of S.
TEST(LaneChangeBypassTest, bypass_line_of_the_successor_of_the_other_lane_counts)
{
  test_maps::BypassMapOptions options;
  options.sibling_length_m = 10.0;
  const auto bypass = test_maps::make_bypass_map(options);
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  const auto trajectory = trajectory_along(bypass.map, route, 5.0);
  EXPECT_TRUE(
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::DISABLE, 20));
}

// A branch that crosses the line at 45 degrees is over the angle limit.
TEST(LaneChangeBypassTest, steep_branch_is_not_lane_change)
{
  test_maps::BypassMapOptions options;
  options.branch_length_m = 4.0;
  const auto bypass = test_maps::make_bypass_map(options);
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  const auto trajectory = trajectory_along(bypass.map, route, 5.0);
  EXPECT_FALSE(
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::DISABLE, 20));
}

// A merge crosses the line too, but M and S have no shared predecessor.
TEST(LaneChangeBypassTest, merge_across_a_line_is_not_lane_change)
{
  const auto merge = test_maps::make_bypass_merge_map();
  const std::vector<lanelet::Id> route{merge.q, merge.b, merge.s1};
  const auto trajectory = trajectory_along(merge.map, route, 5.0);
  EXPECT_FALSE(
    lane_change_onset_cycle(merge.map, route, trajectory, TurnIndicatorsReport::DISABLE, 20));
}

TEST(LaneChangeBypassTest, branch_beside_a_road_shoulder_is_not_lane_change)
{
  test_maps::BypassMapOptions options;
  options.is_sibling_shoulder = true;
  const auto bypass = test_maps::make_bypass_map(options);
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  const auto trajectory = trajectory_along(bypass.map, route, 5.0);
  EXPECT_FALSE(
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::DISABLE, 20));
}

TEST(LaneChangeBypassTest, branch_across_a_virtual_line_is_not_lane_change)
{
  test_maps::BypassMapOptions options;
  options.is_crossed_line_virtual = true;
  const auto bypass = test_maps::make_bypass_map(options);
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  const auto trajectory = trajectory_along(bypass.map, route, 5.0);
  EXPECT_FALSE(
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::DISABLE, 20));
}

TEST(LaneChangeBypassTest, branch_without_a_virtual_bound_is_not_lane_change)
{
  test_maps::BypassMapOptions options;
  options.has_branch_virtual_bound = false;
  const auto bypass = test_maps::make_bypass_map(options);
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  const auto trajectory = trajectory_along(bypass.map, route, 5.0);
  EXPECT_FALSE(
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::DISABLE, 20));
}

TEST(LaneChangeBypassTest, route_through_the_straight_lane_is_not_lane_change)
{
  const auto bypass = test_maps::make_bypass_map();
  const std::vector<lanelet::Id> route{bypass.r, bypass.s, bypass.s1};
  const auto trajectory = trajectory_along(bypass.map, route, 5.0);
  EXPECT_FALSE(
    lane_change_onset_cycle(bypass.map, route, trajectory, TurnIndicatorsReport::DISABLE, 20));
}

// The bypass event ends by the bypass end: after the body has left R, S, and S1, plus the settle
// time (0.5 s, 5 cycles). Then LANE_FOLLOWING, with no UNKNOWN and no ABORTING_LANE_CHANGE.
TEST(LaneChangeBypassTest, bypass_event_ends_after_the_body_leaves_the_other_lane)
{
  test_maps::BypassMapOptions options;
  options.branch_turn_direction = "right";
  const auto cycles = drive_through_bypass(test_maps::make_bypass_map(options), 0);
  const auto onset =
    first_cycle(cycles, 0, [](const auto & c) { return c.state == DS::LANE_CHANGING; });
  ASSERT_LT(onset, cycles.size());
  const auto body_left =
    first_cycle(cycles, onset, [](const auto & c) { return !c.covers_r_or_other_lanes; });
  const auto end = first_cycle(cycles, onset, [](const auto & c) { return !c.state; });
  ASSERT_LT(body_left, cycles.size());
  ASSERT_LT(end, cycles.size()) << "the event must end within the drive";
  EXPECT_GE(end, body_left + 4) << "the event ends only after the body has left S";
  EXPECT_LE(end, body_left + 7) << "the event ends soon after the body has left S";
  EXPECT_EQ(cycles[end].published_state, DS::LANE_FOLLOWING);
  for (const auto & cycle : cycles) {
    EXPECT_NE(cycle.published_state, DS::UNKNOWN);
    EXPECT_NE(cycle.state, DS::ABORTING_LANE_CHANGE);
  }
}

// The footprint is fully inside B but still covers the line: the event stays LANE_CHANGING.
TEST(LaneChangeBypassTest, bypass_event_stays_while_the_branch_footprint_covers_the_line)
{
  test_maps::BypassMapOptions options;
  options.branch_turn_direction = "right";
  const auto cycles = drive_through_bypass(test_maps::make_bypass_map(options), 0);
  const auto onset =
    first_cycle(cycles, 0, [](const auto & c) { return c.state == DS::LANE_CHANGING; });
  ASSERT_LT(onset, cycles.size());
  int covering_cycles = 0;
  for (std::size_t index = onset; index < cycles.size(); ++index) {
    if (cycles[index].covers_line_from_branch) {
      ++covering_cycles;
      EXPECT_EQ(cycles[index].state, DS::LANE_CHANGING) << "cycle " << index;
    }
  }
  EXPECT_GT(covering_cycles, 5) << "the drive must have cycles inside B that cover the line";
}

// After the onset, the ego waits in R for 1.0 s (longer than the 0.5 s settle): no early end.
TEST(LaneChangeBypassTest, bypass_event_stays_while_the_ego_approaches_the_split)
{
  test_maps::BypassMapOptions options;
  options.branch_turn_direction = "right";
  const auto cycles = drive_through_bypass(test_maps::make_bypass_map(options), 13);
  const auto onset =
    first_cycle(cycles, 0, [](const auto & c) { return c.state == DS::LANE_CHANGING; });
  ASSERT_LT(onset, 13u);
  for (std::size_t index = onset; index < 13u; ++index) {
    EXPECT_EQ(cycles[index].state, DS::LANE_CHANGING) << "cycle " << index;
  }
}

// B has no turn_direction: the held lane moves into B, and the event still ends by the bypass end.
TEST(LaneChangeBypassTest, bypass_branch_without_turn_tag_ends_by_the_bypass_end)
{
  const auto bypass = test_maps::make_bypass_map();
  const auto cycles = drive_through_bypass(bypass, 0);
  const auto onset =
    first_cycle(cycles, 0, [](const auto & c) { return c.state == DS::LANE_CHANGING; });
  ASSERT_LT(onset, cycles.size());
  const auto body_left =
    first_cycle(cycles, onset, [](const auto & c) { return !c.covers_r_or_other_lanes; });
  const auto end = first_cycle(cycles, onset, [](const auto & c) { return !c.state; });
  ASSERT_LT(end, cycles.size());
  EXPECT_GE(end, body_left + 4);
  EXPECT_LE(end, body_left + 7) << "the bypass end, not a settle in the lane after B";
  const auto held_in_b = std::any_of(
    cycles.begin() + static_cast<std::ptrdiff_t>(onset),
    cycles.begin() + static_cast<std::ptrdiff_t>(end),
    [&bypass](const auto & c) { return c.reference_lane_id == bypass.b; });
  EXPECT_TRUE(held_in_b) << "with no turn_direction, the held lane moves into B";
  for (const auto & cycle : cycles) {
    EXPECT_NE(cycle.state, DS::ABORTING_LANE_CHANGE);
  }
}

// B has turn_direction=right: the held lane stays on R for the whole event.
TEST(LaneChangeBypassTest, bypass_branch_with_turn_tag_keeps_the_held_lane_on_r)
{
  test_maps::BypassMapOptions options;
  options.branch_turn_direction = "right";
  const auto bypass = test_maps::make_bypass_map(options);
  const auto cycles = drive_through_bypass(bypass, 0);
  const auto onset =
    first_cycle(cycles, 0, [](const auto & c) { return c.state == DS::LANE_CHANGING; });
  ASSERT_LT(onset, cycles.size());
  const auto end = first_cycle(cycles, onset, [](const auto & c) { return !c.state; });
  for (std::size_t index = onset; index < end; ++index) {
    EXPECT_EQ(cycles[index].reference_lane_id, bypass.r) << "cycle " << index;
  }
}

// In the aborting phase of a bypass event, a footprint fully inside B that still covers the line
// does not end the event by the normal settle.
TEST(LaneChangeBypassTest, bypass_abort_does_not_end_by_the_normal_settle_in_the_branch)
{
  test_maps::BypassMapOptions options;
  options.branch_turn_direction = "right";
  const auto bypass = test_maps::make_bypass_map(options);
  const std::vector<lanelet::Id> route{bypass.r, bypass.b, bypass.n};
  Simulator sim{bypass.map, make_config()};
  const lanelet::BasicPoint2d ego_in_r{15.0, 0.0};
  const auto bypass_trajectory = trajectory_along(bypass.map, route, 15.0);
  const std::vector<lanelet::BasicPoint2d> back_in_r{{15.0, 0.0}, {17.0, 0.0}, {19.0, 0.0}};
  const auto footprint_in_r = bus_footprint(ego_in_r, {1.0, 0.0});
  int64_t time_ms = 0;
  const auto step = [&](
                      const lanelet::BasicPoint2d & ego,
                      const std::vector<lanelet::BasicPoint2d> & trajectory,
                      const std::vector<lanelet::BasicPoint2d> & footprint) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    time_ms += 100;
    return sim.step(test_maps::make_trajectory_input(route, ego, sec, nsec, trajectory, footprint));
  };

  bool is_changing = false;
  for (int cycle = 0; cycle < 10 && !is_changing; ++cycle) {
    is_changing = step(ego_in_r, bypass_trajectory, footprint_in_r) == DS::LANE_CHANGING;
  }
  ASSERT_TRUE(is_changing);
  bool is_aborting = false;
  for (int cycle = 0; cycle < 10 && !is_aborting; ++cycle) {
    is_aborting = step(ego_in_r, back_in_r, footprint_in_r) == DS::ABORTING_LANE_CHANGE;
  }
  ASSERT_TRUE(is_aborting);

  // Fully inside B, still covering the dashed right bound of S, with a path that stays inside.
  const lanelet::BasicPoint2d ego_in_b{30.0, -1.0};
  const auto footprint_in_b = bus_footprint(ego_in_b, {10.0, -1.0});
  const std::vector<lanelet::BasicPoint2d> along_b{{30.0, -1.0}, {32.0, -1.2}, {34.0, -1.4}};
  for (int cycle = 0; cycle < 10; ++cycle) {
    EXPECT_EQ(step(ego_in_b, along_b, footprint_in_b), DS::ABORTING_LANE_CHANGE)
      << "cycle " << cycle << ": the normal settle on B must not end a bypass event";
  }
}

}  // namespace lane_event_classifier
