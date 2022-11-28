// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include "base/road_geometry.h"

#include <string>

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_sparse/builder/builder.h"
#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace test {
namespace {

using maliput::api::InertialPosition;
using maliput::api::LanePosition;
using maliput::math::Vector3;
using maliput_sparse::builder::RoadGeometryBuilder;
using maliput_sparse::geometry::LineString3d;

GTEST_TEST(RoadGeometryStubTest, Stub) {
  constexpr double kLinearTolerance{1e-12};
  constexpr double kAngularTolerance{1e-12};
  constexpr double kScaleLength{1.};
  const maliput::math::Vector3 kInertialToBackendFrameTranslation{1., 2., 3.};

  EXPECT_NO_THROW({
    RoadGeometry(maliput::api::RoadGeometryId{"rg_id"}, kLinearTolerance, kAngularTolerance, kScaleLength,
                 kInertialToBackendFrameTranslation);
  });
}

class RoadGeometryTest : public ::testing::Test {
 public:
  void SetUp() override {
    dut_ = builder_.StartJunction()
               .Id(kJunctionAId)
               .StartSegment()
               .Id(kSegmentAId)
               .StartLane()
               .Id(kLaneAId)
               .StartLaneGeometry()
               .LeftLineString(kLeftLineStringA)
               .RightLineString(kRightLineStringA)
               .EndLaneGeometry()
               .EndLane()
               .StartLane()
               .Id(kLaneBId)
               .StartLaneGeometry()
               .LeftLineString(kLeftLineStringB)
               .RightLineString(kRightLineStringB)
               .EndLaneGeometry()
               .EndLane()
               .EndSegment()
               .EndJunction()
               .StartBranchPoints()
               .EndBranchPoints()
               .Build();
  }

  static constexpr double kLinearTolerance{1e-12};
  const maliput::api::RoadGeometryId kRoadGeometryId{"custom_rg_id"};
  const maliput::math::Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};
  const maliput::api::JunctionId kJunctionAId{"junction_a"};
  const maliput::api::SegmentId kSegmentAId{"segment_a"};
  const maliput::api::LaneId kLaneAId{"lane_a"};
  const maliput::api::LaneId kLaneBId{"lane_b"};
  const LineString3d kLeftLineStringA{Vector3{0., 10., 0.}, Vector3{10., 10., 0.}};
  const LineString3d kRightLineStringA{Vector3{0., 5., 0.}, Vector3{10., 5., 0.}};
  const LineString3d kLeftLineStringB{Vector3{0., 5., 0.}, Vector3{10., 5., 0.}};
  const LineString3d kRightLineStringB{Vector3{0., 0., 0.}, Vector3{10., 0., 0.}};

  RoadGeometryBuilder builder_;
  std::unique_ptr<maliput::api::RoadGeometry> dut_;
};

TEST_F(RoadGeometryTest, OnLaneA) {
  const maliput::api::InertialPosition inertial_pos{5., 8., 0.};
  auto lane = dut_->ById().GetLane(kLaneAId);

  const maliput::api::RoadPositionResult expected_road_position_result{
      {lane, LanePosition{5., 0.5, 0.}},
      {5., 8., 0.}, /* nearest position */
      0.,           /* distance */
  };
  EXPECT_TRUE(maliput::api::test::IsRoadPositionResultClose(expected_road_position_result,
                                                            dut_->ToRoadPosition(inertial_pos), kLinearTolerance));
}

TEST_F(RoadGeometryTest, OnLaneB) {
  const maliput::api::InertialPosition inertial_pos{5., 0., 0.};
  auto lane = dut_->ById().GetLane(kLaneBId);

  const maliput::api::RoadPositionResult expected_road_position_result{
      {lane, LanePosition{5., -2.5, 0.}},
      {5., 0., 0.}, /* nearest position */
      0.,           /* distance */
  };
  EXPECT_TRUE(maliput::api::test::IsRoadPositionResultClose(expected_road_position_result,
                                                            dut_->ToRoadPosition(inertial_pos), kLinearTolerance));
}

TEST_F(RoadGeometryTest, CloseToLaneA) {
  const maliput::api::InertialPosition inertial_pos{5., 11., 0.};
  auto lane = dut_->ById().GetLane(kLaneAId);

  const maliput::api::RoadPositionResult expected_road_position_result{
      {lane, LanePosition{5., 2.5, 0.}},
      {5., 10., 0.}, /* nearest position */
      1.,            /* distance */
  };
  EXPECT_TRUE(maliput::api::test::IsRoadPositionResultClose(expected_road_position_result,
                                                            dut_->ToRoadPosition(inertial_pos), kLinearTolerance));
}

}  // namespace
}  // namespace test
}  // namespace maliput_sparse
