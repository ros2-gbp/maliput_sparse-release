// Code in this file is inspired by:
// https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/src/Lanelet.cpp
//
// Lanelet2's license follows:
//
// Copyright 2018 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
// disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
// following disclaimer in the documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
// products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//
// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "maliput_sparse/geometry/utility/geometry.h"

#include <algorithm>
#include <cmath>

namespace maliput_sparse {
namespace geometry {
namespace utility {

using maliput::math::Vector2;
using maliput::math::Vector3;
using OptDistance = std::optional<double>;
using Segment3d = std::pair<Vector3, Vector3>;
using Segment2d = std::pair<Vector2, Vector2>;

static constexpr bool kLeft{true};
static constexpr bool kRight{false};

namespace {

Vector2 To2D(const Vector3& vector) { return {vector.x(), vector.y()}; }

Segment2d To2D(const Segment3d& segment) { return {To2D(segment.first), To2D(segment.second)}; }

LineString2d To2D(const LineString3d& line_string) {
  std::vector<Vector2> points;
  for (const auto& point : line_string) {
    points.push_back(To2D(point));
  }
  return LineString2d{points};
}

// Determines whether two line segments intersects.
//
// Based on https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
bool SegmentsIntersect2d(const Segment2d& lhs, const Segment2d& rhs) {
  const auto& s1_xa = lhs.first[0];
  const auto& s1_ya = lhs.first[1];
  const auto& s1_xb = lhs.second[0];
  const auto& s1_yb = lhs.second[1];
  const auto& s2_xa = rhs.first[0];
  const auto& s2_ya = rhs.first[1];
  const auto& s2_xb = rhs.second[0];
  const auto& s2_yb = rhs.second[1];
  const double den = (s1_xa - s1_xb) * (s2_ya - s2_yb) - (s1_ya - s1_yb) * (s2_xa - s2_xb);
  if (den == 0) {
    // They are parallel or coincident.
    return false;
  }
  const double t_num = (s1_xa - s2_xa) * (s2_ya - s2_yb) - (s1_ya - s2_ya) * (s2_xa - s2_xb);
  const double u_num = (s1_xa - s2_xa) * (s1_ya - s1_yb) - (s1_ya - s2_ya) * (s1_xa - s1_xb);
  const double t = t_num / den;
  const double u = u_num / den;
  return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}

// Obtains the distance between @p lhs and @p rhs points projected on the xy plane.
double Distance2d(const Vector3& lhs, const Vector3& rhs) { return (To2D(lhs) - To2D(rhs)).norm(); }

// Determines whether point @p p is located left from segment compound by @p seg_first and @p seg_second.
bool PointIsLeftOf(const Vector3& seg_first, const Vector3& seg_second, const Vector3& p) {
  return ((seg_second - seg_first).cross(p - seg_first)).z() > 0;
}

Vector3 MakeCenterpoint(const Vector3& p1, const Vector3& p2) { return (p1 + p2) / 2.; }

// Helper class for evaluating the boundaries of a lane.
class BoundChecker {
 public:
  BoundChecker(const LineString3d& left, const LineString3d& right)
      : left_(left), right_(right), entry_{right.first(), left.first()}, exit_{left.last(), right.last()} {
    left_segments_ = MakeSegments(left_);
    right_segments_ = MakeSegments(right_);
  }

  bool Intersects2d(const Segment3d& seg) const {
    const Segment2d seg2d{{seg.first.x(), seg.first.y()}, {seg.second.x(), seg.second.y()}};
    return Intersects2dSegments(kLeft, seg2d) || Intersects2dSegments(kRight, seg2d) || CrossesEntry2d(seg2d) ||
           CrossesExit2d(seg2d);
  }

  bool Intersects2dSegments(bool use_left_segments, const Segment2d& seg) const {
    const auto& segments = use_left_segments ? left_segments_ : right_segments_;
    const auto it = std::find_if(segments.begin(), segments.end(), [&seg](const Segment3d& segment) {
      const Segment2d segment_2d{To2D(segment)};
      return SegmentsIntersect2d(seg, segment_2d) ? !(seg.first == segment_2d.first) : false;
    });
    return it != segments.end();
  }

  bool CrossesEntry2d(const Segment2d& seg) const {
    const Segment2d entry_2d{To2D(entry_)};
    return SegmentsIntersect2d(seg, entry_2d)
               ? PointIsLeftOf({entry_2d.first.x(), entry_2d.first.y(), 0.},
                               {entry_2d.second.x(), entry_2d.second.y(), 0.}, {seg.second.x(), seg.second.y(), 0.})
               : false;
  }

  bool CrossesExit2d(const Segment2d& seg) const {
    const Segment2d exit_2d{{exit_.first.x(), exit_.first.y()}, {exit_.second.x(), exit_.second.y()}};
    return SegmentsIntersect2d(seg, exit_2d)
               ? PointIsLeftOf({exit_2d.first.x(), exit_2d.first.y(), 0.}, {exit_2d.second.x(), exit_2d.second.y(), 0.},
                               {seg.second.x(), seg.second.y(), 0.})
               : false;
  }

  template <typename Func>
  void ForEachPointUntil(bool is_left, LineString3d::const_iterator iter, Func&& f) const {
    const LineString3d& points = is_left ? left_ : right_;
    for (auto iter = points.begin(); iter != points.end(); ++iter) {
      if (f(iter)) {
        break;
      }
    }
  }

 private:
  static std::vector<Segment3d> MakeSegments(const LineString3d& line) {
    std::vector<Segment3d> segments;
    segments.reserve(line.size());
    for (auto i = 0u; i < line.size() - 2; ++i) {
      segments.push_back({line[i], line[i + 1]});
    }
    return segments;
  }

  const LineString3d& left_;
  const LineString3d& right_;
  Segment3d entry_, exit_;
  std::vector<Segment3d> left_segments_;
  std::vector<Segment3d> right_segments_;
};

std::pair<LineString3d::const_iterator, OptDistance> FindClosestNonintersectingPoint(
    LineString3d::const_iterator current_position, LineString3d::const_iterator other_point, const BoundChecker& bounds,
    const Vector3& last_point, bool is_left) {
  OptDistance distance;
  LineString3d::const_iterator closest_position{nullptr};
  double d_last_other = (*other_point - last_point).norm();

  auto non_intersecting_point_loop = [&](LineString3d::const_iterator candidate) {
    // Point must be after current_position.
    if (candidate < current_position) {
      return false;
    }
    const double candidate_distance = Distance2d(*candidate, *other_point) / 2.;  // candidate distance
    // We use the triangle inequation to find a distance where we cannot
    // expect a closer point than the current one.
    if (!!distance && candidate_distance / 2. - d_last_other > *distance) {
      return true;  //> Stops the loop.
    }
    if (!!distance && *distance <= candidate_distance) {
      return false;
    }
    // Candidates are only valid candidates when:
    // 1. Their distance is minimal (at least for now). -> checked above
    // 2. The new connection does not intersect with the borders.
    // 3. Connection between point on one bound and point on other bound
    // does not intersect with other parts of the boundary.
    const Segment3d bound_connection(*other_point, *candidate);
    const auto centerline_point_candidate = MakeCenterpoint(bound_connection.first, bound_connection.second);
    const Segment3d centerline_candidate{last_point, centerline_point_candidate};
    if (!bounds.Intersects2d(centerline_candidate)) {
      distance = candidate_distance;
      closest_position = candidate;
    }
    return false;
  };
  bounds.ForEachPointUntil(is_left, other_point, non_intersecting_point_loop);
  return {closest_position, distance};
}

}  // namespace

LineString3d ComputeCenterline3d(const LineString3d& left, const LineString3d& right) {
  std::vector<Vector3> centerline;
  BoundChecker bounds(left, right);
  // Initial point.
  centerline.push_back(MakeCenterpoint(left.first(), right.first()));

  auto left_current = left.begin();
  auto right_current = right.begin();

  while (left_current != left.end() || right_current != right.end()) {
    std::optional<double> left_candidate_distance;
    std::optional<double> right_candidate_distance;

    LineString3d::const_iterator left_candidate;
    LineString3d::const_iterator right_candidate;
    // Determine left candidate.
    std::tie(left_candidate, left_candidate_distance) =
        FindClosestNonintersectingPoint(std::next(left_current), right_current, bounds, centerline.back(), kLeft);

    // Determine right candidate.
    std::tie(right_candidate, right_candidate_distance) =
        FindClosestNonintersectingPoint(std::next(right_current), left_current, bounds, centerline.back(), kRight);
    // Choose the better one.
    if (left_candidate_distance && (!right_candidate_distance || left_candidate_distance <= right_candidate_distance)) {
      MALIPUT_THROW_UNLESS(left_candidate != left.end());

      const auto& left_point = left[static_cast<size_t>(left_candidate - left.begin())];
      const auto& right_point = right[static_cast<size_t>(right_current - right.begin())];
      const auto centerpoint = MakeCenterpoint(left_point, right_point);
      centerline.push_back(centerpoint);
      left_current = left_candidate;
    } else if (right_candidate_distance &&
               (!left_candidate_distance || left_candidate_distance > right_candidate_distance)) {
      MALIPUT_THROW_UNLESS(right_candidate != right.end());

      const auto& left_point = left[static_cast<size_t>(left_current - left.begin())];
      const auto& right_point = right[static_cast<size_t>(right_candidate - right.begin())];
      const auto centerpoint = MakeCenterpoint(left_point, right_point);
      centerline.push_back(centerpoint);
      right_current = right_candidate;
    } else {
      // No next point found. We are done here.
      break;
    }
  }

  // We want the centerpoint defined by the endpoints inside in any case.
  if (!(left_current == std::prev(left.end()) && right_current == std::prev(right.end()))) {
    centerline.push_back(MakeCenterpoint(left.last(), right.last()));
  }
  return LineString3d{centerline};
}

Vector3 InterpolatedPointAtP(const LineString3d& line_string, double p) {
  // Implementation inspired on:
  // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/include/lanelet2_core/geometry/impl/LineString.h#L618
  static constexpr double kEpsilon{1e-12};
  if (p < 0) return line_string.first();
  if (p >= line_string.length()) return line_string.last();

  const auto line_string_points_length = GetBoundPointsAtP(line_string, p);
  const double partial_length{(*line_string_points_length.first - *line_string_points_length.second).norm()};
  const double remaining_distance = p - line_string_points_length.length;
  if (remaining_distance < kEpsilon) {
    return *line_string_points_length.first;
  }
  return *line_string_points_length.first +
         remaining_distance / partial_length * (*line_string_points_length.second - *line_string_points_length.first);
}

double GetSlopeAtP(const LineString3d& line_string, double p) {
  const BoundPointsResult bound_points = GetBoundPointsAtP(line_string, p);
  const double dist{(To2D(*bound_points.second) - To2D(*bound_points.first)).norm()};
  const double delta_z{bound_points.second->z() - bound_points.first->z()};
  MALIPUT_THROW_UNLESS(*bound_points.second != *bound_points.first);
  return delta_z / dist;
}

BoundPointsResult GetBoundPointsAtP(const LineString3d& line_string, double p) {
  MALIPUT_THROW_UNLESS(p >= 0);
  MALIPUT_THROW_UNLESS(p <= line_string.length());

  BoundPointsResult result;
  double current_cumulative_length = 0.0;
  for (auto first = line_string.begin(), second = std::next(line_string.begin()); second != line_string.end();
       ++first, ++second) {
    const auto p1 = *first;
    const auto p2 = *second;
    const double current_length = (p1 - p2).norm();
    if (current_cumulative_length + current_length >= p) {
      return {first, second, current_cumulative_length};
    }
    current_cumulative_length += current_length;
  }
  return {line_string.end() - 1, line_string.end() - 2, line_string.length()};
}

double Get2DHeadingAtP(const LineString3d& line_string, double p) {
  const auto line_string_points_length = GetBoundPointsAtP(line_string, p);
  const Vector3 heading_vector{*line_string_points_length.second - *line_string_points_length.first};
  return std::atan2(heading_vector.y(), heading_vector.x());
}

Vector2 Get2DTangentAtP(const LineString3d& line_string, double p) {
  // const double heading = Get2DHeadingAtP(line_string, p);
  // return {std::cos(heading), std::sin(heading)};
  const auto line_string_points_length = GetBoundPointsAtP(line_string, p);
  const Vector2 d_xy{To2D(*line_string_points_length.second) - To2D(*line_string_points_length.first)};
  return (d_xy / (To2D(line_string).length())).normalized();
}

Vector3 GetTangentAtP(const LineString3d& line_string, double p) {
  const auto line_string_points_length = GetBoundPointsAtP(line_string, p);
  const Vector3 d_xyz{*line_string_points_length.second - *line_string_points_length.first};
  return (d_xyz / (line_string.length())).normalized();
}

ClosestPointResult GetClosestPoint(const Segment3d& segment, const maliput::math::Vector3& xyz) {
  const maliput::math::Vector3 d_segment{segment.second - segment.first};
  const maliput::math::Vector3 d_xyz_to_first{xyz - segment.first};

  const double unsaturated_p = d_xyz_to_first.dot(d_segment.normalized());
  const double p = std::clamp(unsaturated_p, 0., d_segment.norm());
  const maliput::math::Vector3 point = InterpolatedPointAtP(LineString3d{segment.first, segment.second}, p);
  const double distance = (xyz - point).norm();
  return {p, point, distance};
}

ClosestPointResult GetClosestPoint(const LineString3d& line_string, const maliput::math::Vector3& xyz) {
  ClosestPointResult result;
  result.distance = std::numeric_limits<double>::max();
  double length{};
  for (auto first = line_string.begin(), second = std::next(line_string.begin()); second != line_string.end();
       ++first, ++second) {
    const auto closest_point_res = GetClosestPoint(Segment3d{*first, *second}, xyz);
    if (closest_point_res.distance < result.distance) {
      result = closest_point_res;
      result.p += length;
    }
    length += (*second - *first).norm();  //> Adds segment length
  }
  return result;
}

}  // namespace utility
}  // namespace geometry
}  // namespace maliput_sparse
