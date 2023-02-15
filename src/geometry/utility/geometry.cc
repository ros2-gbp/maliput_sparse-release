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
#include <numeric>

#include <maliput/common/range_validator.h>

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
static constexpr double kEpsilon{1e-12};

namespace {

Vector2 To2D(const Vector3& vector) { return {vector.x(), vector.y()}; }

Segment2d To2D(const Segment3d& segment) { return {To2D(segment.first), To2D(segment.second)}; }

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

template <typename CoordinateT>
CoordinateT InterpolatedPointAtP(const LineString<CoordinateT>& line_string, double p, double tolerance) {
  // Implementation inspired on:
  // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/include/lanelet2_core/geometry/impl/LineString.h#L618
  if (p < 0) return line_string.first();
  if (p >= line_string.length()) return line_string.last();
  const auto bound_points = GetBoundPointsAtP(line_string, p, tolerance);
  const CoordinateT& start = line_string[bound_points.idx_start];
  const CoordinateT& end = line_string[bound_points.idx_end];
  const CoordinateT d_segment{end - start};
  const double remaining_distance = p - bound_points.length;
  return remaining_distance < kEpsilon ? start : start + d_segment.normalized() * remaining_distance;
}

double GetSlopeAtP(const LineString3d& line_string, double p, double tolerance) {
  const BoundPointsResult bound_points = GetBoundPointsAtP(line_string, p, tolerance);
  const maliput::math::Vector3& start = line_string[bound_points.idx_start];
  const maliput::math::Vector3& end = line_string[bound_points.idx_end];
  const double delta_z{end.z() - start.z()};
  const double dist{(To2D(end) - To2D(start)).norm()};
  MALIPUT_THROW_UNLESS(start != end);
  return delta_z / dist;
}

template <typename CoordinateT>
BoundPointsResult GetBoundPointsAtP(const LineString<CoordinateT>& line_string, double p, double tolerance) {
  p = maliput::common::RangeValidator::GetAbsoluteEpsilonValidator(0., line_string.length(), tolerance, kEpsilon)(p);
  const auto segment = line_string.segments().at({p});
  return {segment.idx_start, segment.idx_end, segment.p_interval.min};
}

double Get2DHeadingAtP(const LineString3d& line_string, double p, double tolerance) {
  const auto bound_points = GetBoundPointsAtP(line_string, p, tolerance);
  const Vector3 heading_vector{line_string[bound_points.idx_end] - line_string[bound_points.idx_start]};
  return std::atan2(heading_vector.y(), heading_vector.x());
}

Vector2 Get2DTangentAtP(const LineString3d& line_string, double p, double tolerance) {
  const double heading = Get2DHeadingAtP(line_string, p, tolerance);
  return {std::cos(heading), std::sin(heading)};
}

Vector3 GetTangentAtP(const LineString3d& line_string, double p, double tolerance) {
  const auto bound_points = GetBoundPointsAtP(line_string, p, tolerance);
  const Vector3 d_xyz{line_string[bound_points.idx_end] - line_string[bound_points.idx_start]};
  return (d_xyz / (line_string.length())).normalized();
}

template <typename CoordinateT>
ClosestPointToSegmentResult<CoordinateT> GetClosestPointToSegment(const CoordinateT& start_segment_point,
                                                                  const CoordinateT& end_segment_point,
                                                                  const CoordinateT& coordinate, double tolerance) {
  if (start_segment_point == end_segment_point) {
    return {0., start_segment_point, (start_segment_point - coordinate).norm()};
  }
  const CoordinateT d_segment{end_segment_point - start_segment_point};
  const CoordinateT d_segment_normalized{d_segment.normalized()};
  const CoordinateT d_coordinate_to_first{coordinate - start_segment_point};

  const double unsaturated_p = d_coordinate_to_first.dot(d_segment_normalized);
  const double p = std::clamp(unsaturated_p, 0., d_segment.norm());

  // point at p
  const CoordinateT point = p * d_segment_normalized + start_segment_point;
  const double distance = (coordinate - point).norm();
  return {p, point, distance};
}

ClosestPointResult3d GetClosestPoint(const LineString3d& line_string, const maliput::math::Vector3& xyz,
                                     double tolerance) {
  std::optional<LineString3d::Segment> closest_segment{std::nullopt};
  ClosestPointToSegmentResult3d segment_closest_point_result;
  segment_closest_point_result.distance = std::numeric_limits<double>::max();

  const std::vector<LineString3d::Point>& line_string_points = line_string.points();
  const LineString3d::KDTree* kd_tree = line_string.kd_tree();
  MALIPUT_THROW_UNLESS(kd_tree != nullptr);
  const LineString3d::Point& nearest_point = kd_tree->nearest_point(LineString3d::Point{xyz});
  // Nearest point is the closest point to the line string.
  // If the idx is the first or last then obtain the first or last segment.
  // Otherwise, obtain the segment that contains the nearest point.
  MALIPUT_THROW_UNLESS(nearest_point.idx() != std::nullopt);
  const std::size_t nearest_idx = nearest_point.idx().value();
  if (nearest_idx == 0 || nearest_idx == line_string.size() - 1) {
    closest_segment =
        nearest_idx == 0 ? line_string.segments().begin()->second : (--line_string.segments().end())->second;
    segment_closest_point_result = GetClosestPointToSegment(line_string[closest_segment->idx_start],
                                                            line_string[closest_segment->idx_end], xyz, tolerance);
  } else {
    // The closest segment will be the one that contains the nearest point, whether it is the start or end point of the
    // segment.
    const maliput::math::Vector3& nearest_coordinate{line_string[nearest_idx]};
    const maliput::math::Vector3& previous_nearest_point{line_string[nearest_idx - 1]};
    const maliput::math::Vector3& next_nearest_point{line_string[nearest_idx + 1]};

    const auto previous_segment_closest_point_res =
        GetClosestPointToSegment(previous_nearest_point, nearest_coordinate, xyz, tolerance);
    const auto next_segment_closest_point_res =
        GetClosestPointToSegment(nearest_coordinate, next_nearest_point, xyz, tolerance);
    MALIPUT_THROW_UNLESS(nearest_point.p() != std::nullopt);

    // Compares the distance between the nearest point and the closest point to the previous segment and the next
    // segment.
    segment_closest_point_result = previous_segment_closest_point_res.distance < next_segment_closest_point_res.distance
                                       ? previous_segment_closest_point_res
                                       : next_segment_closest_point_res;
    closest_segment =
        previous_segment_closest_point_res.distance < next_segment_closest_point_res.distance
            ? LineString3d::Segment{nearest_idx - 1, nearest_idx,
                                    LineString3d::Segment::Interval{line_string_points.at(nearest_idx - 1).p().value(),
                                                                    nearest_point.p().value()}}
            : LineString3d::Segment{nearest_idx, nearest_idx + 1,
                                    LineString3d::Segment::Interval{
                                        nearest_point.p().value(), line_string_points.at(nearest_idx + 1).p().value()}};
  }
  return {segment_closest_point_result.p + closest_segment->p_interval.min, segment_closest_point_result.point,
          segment_closest_point_result.distance, closest_segment.value()};
}

ClosestPointResult3d GetClosestPointUsing2dProjection(const LineString3d& line_string,
                                                      const maliput::math::Vector3& xyz, double tolerance) {
  // Get the closest segment in 3d.
  const LineString3d::Segment closest_segment_3d = GetClosestPoint(line_string, xyz, tolerance).segment;
  const auto& start = line_string[closest_segment_3d.idx_start];
  const auto& end = line_string[closest_segment_3d.idx_end];

  // Once we have the segment let's project the point to the segment in 2d.
  const auto start_2d = To2D(start);
  const auto end_2d = To2D(end);
  const maliput::math::Vector2 xy = To2D(xyz);
  // Get the closest point in 2d.
  const ClosestPointToSegmentResult2d closest_point_to_segment_2d =
      GetClosestPointToSegment(start_2d, end_2d, xy, tolerance);

  // Scale the p coordinate in 2d to the 3d segment length to obtain the correct p coordinate in 3d.
  const double segment_3d_length = (start - end).norm();
  const double scale_p = segment_3d_length / (start_2d - end_2d).norm();
  const double p_3d = closest_point_to_segment_2d.p * scale_p;
  const double z_coordinate = start.z() + ((end - start).normalized() * p_3d).z();
  // Compound closest point in 3d.
  const maliput::math::Vector3 closest_point_3d{closest_point_to_segment_2d.point.x(),
                                                closest_point_to_segment_2d.point.y(), z_coordinate};

  return {p_3d + closest_segment_3d.p_interval.min, closest_point_3d, (xyz - closest_point_3d).norm(),
          closest_segment_3d};
}

double ComputeDistance(const LineString3d& lhs, const LineString3d& rhs, double tolerance) {
  const LineString3d& base_ls = rhs.size() > lhs.size() ? rhs : lhs;
  const LineString3d& other_ls = rhs.size() > lhs.size() ? lhs : rhs;
  const double sum_distances =
      std::accumulate(base_ls.begin(), base_ls.end(), 0.0, [&other_ls, &tolerance](double sum, const auto& base_point) {
        const auto closest_point_res = GetClosestPoint(other_ls, base_point, tolerance);
        return sum + closest_point_res.distance;
      });
  return sum_distances / static_cast<double>(base_ls.size());
}

// Explicit instantiations

template maliput::math::Vector3 InterpolatedPointAtP(const LineString3d&, double, double);
template maliput::math::Vector2 InterpolatedPointAtP(const LineString2d&, double, double);

template ClosestPointToSegmentResult3d GetClosestPointToSegment(const maliput::math::Vector3&,
                                                                const maliput::math::Vector3&,
                                                                const maliput::math::Vector3&, double);
template ClosestPointToSegmentResult2d GetClosestPointToSegment(const maliput::math::Vector2&,
                                                                const maliput::math::Vector2&,
                                                                const maliput::math::Vector2&, double);

template class ClosestPointResult<maliput::math::Vector3>;
template class ClosestPointResult<maliput::math::Vector2>;

}  // namespace utility
}  // namespace geometry
}  // namespace maliput_sparse
