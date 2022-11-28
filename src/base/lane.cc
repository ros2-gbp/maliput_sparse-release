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
#include "base/lane.h"

#include <maliput/common/maliput_throw.h>

namespace maliput_sparse {

Lane::Lane(const maliput::api::LaneId& id, const maliput::api::HBounds& elevation_bounds,
           std::unique_ptr<geometry::LaneGeometry> lane_geometry)
    : maliput::geometry_base::Lane(id), elevation_bounds_(elevation_bounds), lane_geometry_(std::move(lane_geometry)) {
  MALIPUT_THROW_UNLESS(lane_geometry_ != nullptr);
}

double Lane::do_length() const { return lane_geometry_->ArcLength(); }

maliput::api::RBounds Lane::do_lane_bounds(double s) const { return lane_geometry_->RBounds(s); }

maliput::api::RBounds Lane::do_segment_bounds(double s) const {
  const double bound_left = ComputeDistanceToSegmentBoundary(kToLeft, s);
  const double bound_right = ComputeDistanceToSegmentBoundary(kToRight, s);
  const double tolerance = lane_geometry_->linear_tolerance();
  return {bound_right < tolerance ? -tolerance : -bound_right, bound_left < tolerance ? tolerance : bound_left};
}

double Lane::ComputeDistanceToSegmentBoundary(bool to_left, double s) const {
  const maliput::api::RBounds lane_bounds = do_lane_bounds(s);
  double lane_bound = to_left ? lane_bounds.max() : -lane_bounds.min();
  maliput::api::Lane const* other_lane = to_left ? this->to_left() : this->to_right();
  while (other_lane != nullptr) {
    const double s_other_lane = s * other_lane->length() / length();
    const maliput::api::RBounds other_lane_bounds = other_lane->lane_bounds(s_other_lane);
    lane_bound += other_lane_bounds.max() - other_lane_bounds.min();
    other_lane = to_left ? other_lane->to_left() : other_lane->to_right();
  }
  return lane_bound;
}

maliput::api::HBounds Lane::do_elevation_bounds(double, double) const { return elevation_bounds_; }

maliput::math::Vector3 Lane::DoToBackendPosition(const maliput::api::LanePosition& lane_pos) const {
  return lane_geometry_->W(lane_pos.srh());
}

maliput::api::LanePositionResult Lane::ToLanePositionBackend(const maliput::api::InertialPosition& backend_pos) const {
  maliput::api::LanePosition lane_position;
  maliput::math::Vector3 nearest_backend_pos;
  double distance{};
  DoToLanePositionBackend(backend_pos.xyz(), &lane_position, &nearest_backend_pos, &distance);
  return {lane_position, maliput::api::InertialPosition::FromXyz(nearest_backend_pos), distance};
}

maliput::api::LanePositionResult Lane::ToSegmentPositionBackend(
    const maliput::api::InertialPosition& backend_pos) const {
  maliput::api::LanePosition lane_position;
  maliput::math::Vector3 nearest_backend_pos;
  double distance{};
  DoToSegmentPositionBackend(backend_pos.xyz(), &lane_position, &nearest_backend_pos, &distance);
  return {lane_position, maliput::api::InertialPosition::FromXyz(nearest_backend_pos), distance};
}

void Lane::InertialToLaneSegmentPositionBackend(bool use_lane_boundaries, const maliput::math::Vector3& backend_pos,
                                                maliput::api::LanePosition* lane_position,
                                                maliput::math::Vector3* nearest_backend_pos, double* distance) const {
  MALIPUT_THROW_UNLESS(lane_position != nullptr);
  MALIPUT_THROW_UNLESS(nearest_backend_pos != nullptr);
  MALIPUT_THROW_UNLESS(distance != nullptr);
  // Obtains srh coordinates without saturation in r and h.
  const maliput::math::Vector3 unsaturated_srh = lane_geometry_->WInverse(backend_pos);

  // Saturates r coordinates to the lane bounds.
  const maliput::api::RBounds r_bounds =
      use_lane_boundaries ? lane_geometry_->RBounds(unsaturated_srh[0]) : do_segment_bounds(unsaturated_srh[0]);
  const double saturated_r = std::clamp(unsaturated_srh[1], r_bounds.min(), r_bounds.max());

  // Saturates h coordinates to the elevation bounds.
  const maliput::api::HBounds h_bounds = do_elevation_bounds(unsaturated_srh[0], saturated_r);
  const double saturated_h = std::clamp(unsaturated_srh[2], h_bounds.min(), h_bounds.max());

  *lane_position = maliput::api::LanePosition(unsaturated_srh[0], saturated_r, saturated_h);
  *nearest_backend_pos = DoToBackendPosition(*lane_position);
  *distance = (backend_pos - *nearest_backend_pos).norm();
}

void Lane::DoToLanePositionBackend(const maliput::math::Vector3& backend_pos, maliput::api::LanePosition* lane_position,
                                   maliput::math::Vector3* nearest_backend_pos, double* distance) const {
  InertialToLaneSegmentPositionBackend(kUseLaneBoundaries, backend_pos, lane_position, nearest_backend_pos, distance);
}

void Lane::DoToSegmentPositionBackend(const maliput::math::Vector3& backend_pos,
                                      maliput::api::LanePosition* lane_position,
                                      maliput::math::Vector3* nearest_backend_pos, double* distance) const {
  InertialToLaneSegmentPositionBackend(kUseSegmentBoundaries, backend_pos, lane_position, nearest_backend_pos,
                                       distance);
}

maliput::api::Rotation Lane::DoGetOrientation(const maliput::api::LanePosition& lane_pos) const {
  const auto rpy = lane_geometry_->Orientation(lane_pos.srh());
  return maliput::api::Rotation::FromRpy(rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle());
}

maliput::api::LanePosition Lane::DoEvalMotionDerivatives(const maliput::api::LanePosition& position,
                                                         const maliput::api::IsoLaneVelocity& velocity) const {
  // The definition of path-length of a path along σ yields dσ = |∂W/∂s| ds
  // evaluated at (s, r, h).
  const double ds_dsigma = lane_geometry_->WDot(position.s()).norm() / lane_geometry_->WDot(position.srh()).norm();
  return {ds_dsigma * velocity.sigma_v, velocity.rho_v, velocity.eta_v};
}

}  // namespace maliput_sparse
