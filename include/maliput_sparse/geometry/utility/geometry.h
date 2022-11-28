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
#pragma once

#include <optional>

#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace geometry {
namespace utility {

/// Holds the result of #GetBoundPointsAtP method.
struct BoundPointsResult {
  LineString3d::const_iterator first;
  LineString3d::const_iterator second;
  // Length up to first.
  double length;
};

/// Holds the result of #GetClosestPoint method.
struct ClosestPointResult {
  /// P coordinate in the line string matching the closest point.
  double p;
  /// Closest point.
  maliput::math::Vector3 point;
  /// Distance between the closest point and the given point.
  double distance;
};

/// Computes a 3-dimensional centerline out of the @p left and @p right line string.
///
/// Inspired on https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/src/Lanelet.cpp
///
/// @param left Left line string.
/// @param right Right line string.
/// @returns The centerline.
LineString3d ComputeCenterline3d(const LineString3d& left, const LineString3d& right);

/// Returns the piecewise linearly interpolated point at the given distance and the distance from the beginning to the
/// first point.
/// @param line_string the line_string to iterate.
/// @param p distance along line_string.
/// @return The interpolated point (a new point if not perfectly matching).
maliput::math::Vector3 InterpolatedPointAtP(const LineString3d& line_string, double p);

/// Returns the slope of a @p line_string for a given @p p .
/// The slope is calculated as the variation in `z` divided by the variation in the `xy` plane.
/// The result is expected to be contained within (-inf., inf.).
/// @param line_string LineString to be computed the slope from.
/// @param p P parameter at which compute the slope.
/// @throws maliput::common::assertion_error When `p ∉ [0., line_string.length()]`.
/// @throws maliput::common::assertion_error When the points that confine @p p are equal.
double GetSlopeAtP(const LineString3d& line_string, double p);

/// Obtains the points that confines @p p in the @p line_string .
/// @param line_string LineString.
/// @param p P parameter.
/// @throws maliput::common::assertion_error When `p ∉ [0., line_string.length()]`.
BoundPointsResult GetBoundPointsAtP(const LineString3d& line_string, double p);

/// Returns the heading of a @p line_string for a given @p p .
/// @param line_string LineString to be computed the heading from.
/// @param p P parameter at which compute the heading.
/// @throws maliput::common::assertion_error When `p ∉ [0., line_string.length()]`.
double Get2DHeadingAtP(const LineString3d& line_string, double p);

/// Returns the 2d-tangent of a @p line_string for a given @p p .
/// The tangent is calculated from the @p line_string projected on the xy plane.
/// @param line_string LineString to be computed the 2d-tangent from.
/// @param p P parameter at which compute the 2d-tangent.
/// @throws maliput::common::assertion_error When `p ∉ [0., line_string.length()]`.
maliput::math::Vector2 Get2DTangentAtP(const LineString3d& line_string, double p);

/// Returns the 3d-tangent of a @p line_string for a given @p p .
/// @param line_string LineString to be computed the 3d-tangent from.
/// @param p P parameter at which compute the 3d-tangent.
/// @throws maliput::common::assertion_error When `p ∉ [0., line_string.length()]`.
maliput::math::Vector3 GetTangentAtP(const LineString3d& line_string, double p);

/// Gets the closest point in the @p segment to the given @p xyz point.
/// @param segment Segment to be computed the closest point from.
/// @param xyz Point to be computed the closest point to.
/// @return A ClosestPointResult struct containing the closest point, the distance between the closest point and @p xyz
/// and the p coordinate in the segment matching the closest point.
ClosestPointResult GetClosestPoint(const std::pair<maliput::math::Vector3, maliput::math::Vector3>& segment,
                                   const maliput::math::Vector3& xyz);

/// Gets the closest point in the @p line_string to the given @p xyz point.
/// @param line_string LineString3d to be computed the closest point from.
/// @param xyz Point to be computed the closest point to.
/// @return A ClosestPointResult struct containing the closest point, the distance between the closest point and @p xyz
/// and the p coordinate in the LineString3d matching the closest point.
ClosestPointResult GetClosestPoint(const LineString3d& line_string, const maliput::math::Vector3& xyz);

}  // namespace utility
}  // namespace geometry
}  // namespace maliput_sparse
