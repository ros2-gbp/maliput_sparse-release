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
#include "maliput_sparse/loader/road_geometry_loader.h"

#include <maliput/common/logger.h>

#include "maliput_sparse/builder/builder.h"
#include "maliput_sparse/parser/validator.h"

namespace maliput_sparse {
namespace loader {
namespace {

maliput::api::LaneEnd::Which ToMaliputLaneEndWhich(const parser::LaneEnd::Which end) {
  switch (end) {
    case parser::LaneEnd::Which::kStart:
      return maliput::api::LaneEnd::Which::kStart;
    case parser::LaneEnd::Which::kFinish:
      return maliput::api::LaneEnd::Which::kFinish;
    default:
      MALIPUT_THROW_MESSAGE("Unknown parser::LaneEnd::Which value: " + static_cast<int>(end));
  }
}

}  // namespace

RoadGeometryLoader::RoadGeometryLoader(std::unique_ptr<parser::Parser> parser,
                                       const BuilderConfiguration& builder_configuration)
    : parser_(std::move(parser)), builder_configuration_(builder_configuration) {
  MALIPUT_THROW_UNLESS(parser_ != nullptr);
}

std::unique_ptr<const maliput::api::RoadGeometry> RoadGeometryLoader::operator()() {
  // Validates the parsed data before building the RoadGeometry.
  const auto errors = parser::Validator(
      parser_.get(),
      {parser::Validator::Type::kLogicalLaneAdjacency, parser::Validator::Type::kGeometricalLaneAdjacency},
      parser::ValidatorConfig{builder_configuration_.linear_tolerance})();
  for (const auto& error : errors) {
    switch (error.severity) {
      case parser::Validator::Error::Severity::kError:
        maliput::log()->error("[{}] {}", error.type, error.message);
        break;
      case parser::Validator::Error::Severity::kWarning:
        maliput::log()->warn("[{}] {}", error.type, error.message);
        break;
      default:
        MALIPUT_THROW_MESSAGE("Unknown parser::Validator::Error::Severity value: " + static_cast<int>(error.severity));
    }
  }

  if (std::find_if(errors.begin(), errors.end(), [](const parser::Validator::Error& error) {
        return error.severity == parser::Validator::Error::Severity::kError;
      }) != errors.end()) {
    MALIPUT_THROW_MESSAGE("Errors(" + std::to_string(static_cast<int>(errors.size())) +
                          ") found during validation. Aborting.");
  }

  // Builds the RoadGeometry.
  const std::unordered_map<parser::Junction::Id, parser::Junction>& junctions = parser_->GetJunctions();
  const std::vector<parser::Connection>& connections = parser_->GetConnections();

  maliput_sparse::builder::RoadGeometryBuilder rg_builder{};
  rg_builder.Id(builder_configuration_.road_geometry_id)
      .LinearTolerance(builder_configuration_.linear_tolerance)
      .AngularTolerance(builder_configuration_.angular_tolerance)
      .ScaleLength(builder_configuration_.scale_length)
      .InertialToBackendFrameTranslation(builder_configuration_.inertial_to_backend_frame_translation);

  for (const auto& junction : junctions) {
    maliput_sparse::builder::JunctionBuilder junction_builder = rg_builder.StartJunction();
    junction_builder.Id(maliput::api::JunctionId{junction.first});
    for (const auto& segment : junction.second.segments) {
      maliput_sparse::builder::SegmentBuilder segment_builder = junction_builder.StartSegment();
      segment_builder.Id(maliput::api::SegmentId{segment.first});

      for (const parser::Lane& lane : segment.second.lanes) {
        const maliput::api::LaneId lane_id{lane.id};
        segment_builder.StartLane()
            .Id(lane_id)
            .HeightBounds(maliput::api::HBounds{0., 5.})
            .StartLaneGeometry()
            .LeftLineString(lane.left)
            .RightLineString(lane.right)
            .EndLaneGeometry()
            .EndLane();
      }
      segment_builder.EndSegment();
    }
    junction_builder.EndJunction();
  }
  maliput_sparse::builder::BranchPointBuilder bp_builder = rg_builder.StartBranchPoints();
  for (const auto& connection : connections) {
    bp_builder.Connect(maliput::api::LaneId{connection.from.lane_id}, ToMaliputLaneEndWhich(connection.from.end),
                       maliput::api::LaneId{connection.to.lane_id}, ToMaliputLaneEndWhich(connection.to.end));
  }
  bp_builder.EndBranchPoints();

  return rg_builder.Build();
}

}  // namespace loader
}  // namespace maliput_sparse
