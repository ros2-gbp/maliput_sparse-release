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

#include <memory>

#include <maliput/api/lane_data.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput/common/range_validator.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/math/vector.h>

#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace geometry {

class LaneGeometry {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(LaneGeometry);

  /// Enum for the type of LineString of the Lane.
  enum class LineStringType {
    kCenterLine = 0,
    kLeftBoundary,
    kRightBoundary,
  };

  /// Constructs a LaneGeometry
  /// @param left Left boundary of the lane.
  /// @param right Right boundary of the lane.
  /// @param linear_tolerance It is expected to be the same as
  ///        maliput::api::RoadGeometry::linear_tolerance(). It must be non
  ///        negative.
  /// @param scale_length It is expected to be the same as
  ///        maliput::api::RoadGeometry::scale_length(). It must be non
  ///        negative.
  /// @throws maliput::common::assertion_error When @p linear_tolerance or
  ///         @p scale_length are negative.
  LaneGeometry(const LineString3d& left, const LineString3d& right, double linear_tolerance, double scale_length);

  /// Constructs a LaneGeometry
  /// @param center Center line of the lane.
  /// @param left Left boundary of the lane.
  /// @param right Right boundary of the lane.
  /// @param linear_tolerance It is expected to be the same as
  ///        maliput::api::RoadGeometry::linear_tolerance(). It must be non
  ///        negative.
  /// @param scale_length It is expected to be the same as
  ///        maliput::api::RoadGeometry::scale_length(). It must be non
  ///        negative.
  /// @throws maliput::common::assertion_error When @p linear_tolerance or
  ///         @p scale_length are negative.
  LaneGeometry(const LineString3d& center, const LineString3d& left, const LineString3d& right, double linear_tolerance,
               double scale_length);

  ~LaneGeometry() = default;

  double p0() const { return 0.; }
  double p1() const { return centerline_.length(); }

  /// @return The centerline of the lane.
  const LineString3d& centerline() const { return centerline_; }

  /// @return The arc length of the centerline of the lane.
  double ArcLength() const;

  /// @return The linear tolerance used to compute all the methods.
  /// @see maliput::api::RoadGeometry::linear_tolerance().
  double linear_tolerance() const { return linear_tolerance_; }

  /// @return The scale length of the tolerance used to compute all the methods.
  /// @see maliput::api::RoadGeometry::scale_length().
  double scale_length() const { return scale_length_; }

  /// Evaluates @f$ W(p, r, h) @f$.
  ///
  /// @param prh A vector in the LaneGeometry domain.
  /// @return A vector in the INERTIAL Frame which is the image of the LaneGeometry.
  maliput::math::Vector3 W(const maliput::math::Vector3& prh) const;

  /// Evaluates @f$ W'(p, r, h) @f$ with respect to @f$ p @f$.
  ///
  /// @param prh A vector in the LaneGeometry domain.
  /// @return The derivative of @f$ W @f$ with respect to @f$ p @f$ at @p prh.
  maliput::math::Vector3 WDot(const maliput::math::Vector3& prh) const;

  /// Evaluates @f$ W'(p, 0, 0) @f$ with respect to @f$ p @f$.
  ///
  /// @param p P parameter of the lane.
  /// @return The derivative of @f$ W @f$ with respect to @f$ p @f$ at @p p.
  maliput::math::Vector3 WDot(double p) const;

  /// Evaluates the orientation in the INERTIAL Frame of the LaneGeometry at
  /// @p prh.
  ///
  /// @param prh A vector in the LaneGeometry domain.
  /// @return The orientation in the INERTIAL Frame of the LaneGeometry at @p prh.
  maliput::math::RollPitchYaw Orientation(const maliput::math::Vector3& prh) const;

  /// Evaluates the orientation in the INERTIAL Frame of the RoadCurve at @p p,
  /// i.e. at @f$ (p, 0, 0) @f$.
  ///
  /// @param p P parameter of the lane.
  /// @return The orientation in the INERTIAL Frame of the RoadCurve at @p p.
  maliput::math::RollPitchYaw Orientation(double p) const;

  /// Evaluates @f$ W⁻¹(x, y, z) @f$.
  ///
  /// @param xyz A point in ℝ³ that would be used to minimize the Euclidean
  ///        distance the image of @f$ W @f$.
  /// @return A vector in LaneGeometry's domain whose image through @f$ W @f$ would
  ///         minimize the Euclidean distance to @p xyz.
  maliput::math::Vector3 WInverse(const maliput::math::Vector3& xyz) const;

  /// Obtains the bounds of the lane.
  /// @param p P parameter of the lane.
  /// @throws maliput::common::assertion_error When @p is not in the domain of [p0(), p1()].
  maliput::api::RBounds RBounds(double p) const;

  /// Converts from p coordinate in the centerline to an equivalent p coordinate in a boundary.
  /// @param line_string_type The type of LineString to convert p coordinate to.
  /// @param p P parameter of the centerline.
  /// @throws maliput::common::assertion_error When @p line_string_type is LineStringType::kCenterline.
  double FromCenterPToLateralP(const LineStringType& line_string_type, double p) const;

  /// Computes the inertial position in a lateral boundary for a given @p p .
  /// @param line_string_type The type of LineString to convert p coordinate to.
  /// @param p P parameter of the centerline.
  /// @throws maliput::common::assertion_error When @p line_string_type is LineStringType::kCenterline.
  maliput::math::Vector3 ToLateralPos(const LineStringType& line_string_type, double p) const;

 private:
  static constexpr double kEpsilon{1e-12};
  const LineString3d left_;
  const LineString3d right_;
  const double linear_tolerance_{};
  const double scale_length_{};
  const LineString3d centerline_;
  const maliput::common::RangeValidator range_validator_;
};

}  // namespace geometry
}  // namespace maliput_sparse
