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
#include "maliput_sparse/builder/builder.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_sparse/geometry/lane_geometry.h"
#include "maliput_sparse/geometry/line_string.h"
#include "maliput_sparse/geometry/utility/geometry.h"

namespace maliput_sparse {
namespace builder {
namespace test {
namespace {
using maliput::api::test::IsHBoundsClose;
using maliput::api::test::IsLaneEndEqual;
using maliput::math::Vector3;
using maliput_sparse::geometry::LineString3d;

class LaneEndTest : public ::testing::Test {
 protected:
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::LaneEnd::Which kEnd{maliput::api::LaneEnd::Which::kFinish};
};

TEST_F(LaneEndTest, FullyFeaturedConstructor) {
  const LaneEnd dut(kLaneId, kEnd);

  ASSERT_EQ(kLaneId, dut.lane_id);
  ASSERT_EQ(kEnd, dut.end);
}

TEST_F(LaneEndTest, CopyConstructor) {
  const LaneEnd dut(kLaneId, kEnd);
  const LaneEnd another_dut(dut);

  ASSERT_EQ(dut.lane_id, another_dut.lane_id);
  ASSERT_EQ(dut.end, another_dut.end);
}

TEST_F(LaneEndTest, MoveConstructor) {
  LaneEnd dut(kLaneId, kEnd);
  const LaneEnd another_dut = std::move(dut);

  ASSERT_EQ(kLaneId, another_dut.lane_id);
  ASSERT_EQ(kEnd, another_dut.end);
}

TEST_F(LaneEndTest, AssingmentOperator) {
  const LaneEnd dut(kLaneId, kEnd);
  const LaneEnd another_dut = dut;

  ASSERT_EQ(dut.lane_id, another_dut.lane_id);
  ASSERT_EQ(dut.end, another_dut.end);
}

TEST_F(LaneEndTest, EqualityOperator) {
  const LaneEnd dut_a(kLaneId, kEnd);
  const LaneEnd dut_b(dut_a);
  const LaneEnd dut_c(kLaneId, maliput::api::LaneEnd::Which::kStart);
  const LaneEnd dut_d(maliput::api::LaneId("another ID"), kEnd);

  ASSERT_EQ(dut_a, dut_b);
  ASSERT_EQ(dut_b, dut_a);
  ASSERT_NE(dut_a, dut_c);
  ASSERT_NE(dut_a, dut_d);
}

TEST_F(LaneEndTest, Less) {
  const LaneEnd dut_a(kLaneId, kEnd);
  const LaneEnd dut_b(dut_a);
  const LaneEnd dut_c(kLaneId, maliput::api::LaneEnd::Which::kStart);
  const LaneEnd dut_d(maliput::api::LaneId("another ID"), kEnd);

  // Equal elements.
  ASSERT_FALSE(dut_a < dut_b);
  ASSERT_FALSE(dut_b < dut_a);
  // Equal lane_id, defines by end.
  ASSERT_TRUE(dut_c < dut_a);
  ASSERT_FALSE(dut_a < dut_c);
  // Defines by lane_id
  ASSERT_TRUE(dut_d < dut_a);
  ASSERT_FALSE(dut_a < dut_d);
}

class RoadGeometryBuilderTest : public ::testing::Test {
 public:
  static constexpr double kEqualityTolerance{0.};
  static constexpr double kLinearTolerance{1.};
  static constexpr double kAngularTolerance{2.};
  static constexpr double kScaleLength{3.};

  const maliput::api::RoadGeometryId kRoadGeometryId{"custom_rg_id"};
  const maliput::math::Vector3 kInertialToBackendFrameTranslation{4., 5., 6.};
  const maliput::api::JunctionId kJunctionAId{"junction_a"};
  const maliput::api::JunctionId kJunctionBId{"unset_id"};
  const maliput::api::SegmentId kSegmentAId{"segment_a"};
  const maliput::api::SegmentId kSegmentBId{"segment_b"};
  const maliput::api::SegmentId kSegmentCId{"unset_id"};
  const maliput::api::SegmentId kSegmentDId{"segment_d"};
  const maliput::api::LaneId kLaneAId{"lane_a"};
  const maliput::api::LaneId kLaneBId{"lane_b"};
  const maliput::api::LaneId kLaneCId{"lane_c"};
  const maliput::api::LaneId kLaneDId{"unset_id"};
  const LineString3d kLeftLineStringA{Vector3{0., 0., 0.}, Vector3{10., 0., 0.}};
  const LineString3d kRightLineStringA{Vector3{0., 5., 0.}, Vector3{10., 5., 0.}};
  const LineString3d kLeftLineStringB{Vector3{0., 5., 0.}, Vector3{10., 5., 0.}};
  const LineString3d kRightLineStringB{Vector3{0., 10., 0.}, Vector3{10., 10., 0.}};
  const LineString3d kLeftLineStringC{Vector3{0., 0., 0.}, Vector3{0., 10., 0.}};
  const LineString3d kRightLineStringC{Vector3{5., 0., 0.}, Vector3{5., 10., 0.}};
  const LineString3d kLeftLineStringD{Vector3{20., 0., 0.}, Vector3{30., 5., 0.}};
  const LineString3d kRightLineStringD{Vector3{20., 0., 0.}, Vector3{30., 5., 0.}};
  const maliput::api::HBounds kHBoundsA{0., 1.};
  const maliput::api::HBounds kHBoundsB{-1., 2.};
  const maliput::api::HBounds kHBoundsC{-2., 3.};
  const maliput::api::HBounds kHBoundsD{0, 5.};

  RoadGeometryBuilder dut;
};

TEST_F(RoadGeometryBuilderTest, LinearToleranceConstraintFails) {
  EXPECT_THROW(RoadGeometryBuilder().LinearTolerance(-1.0), maliput::common::assertion_error);
  EXPECT_THROW(RoadGeometryBuilder().LinearTolerance(0.0), maliput::common::assertion_error);
}

TEST_F(RoadGeometryBuilderTest, AngularToleranceConstraintFails) {
  EXPECT_THROW(RoadGeometryBuilder().AngularTolerance(-1.0), maliput::common::assertion_error);
  EXPECT_THROW(RoadGeometryBuilder().AngularTolerance(0.0), maliput::common::assertion_error);
}

TEST_F(RoadGeometryBuilderTest, ScaleLengthConstraintFails) {
  EXPECT_THROW(RoadGeometryBuilder().ScaleLength(-1.0), maliput::common::assertion_error);
  EXPECT_THROW(RoadGeometryBuilder().ScaleLength(0.0), maliput::common::assertion_error);
}

TEST_F(RoadGeometryBuilderTest, RoadGeometryBuilderWithoutJunctions) {
  EXPECT_THROW(dut.Build(), maliput::common::assertion_error);
}

TEST_F(RoadGeometryBuilderTest, JunctionBuilderWithoutSegments) {
  EXPECT_THROW(
      // clang-format off
      dut
          .StartJunction()
              .Id(kJunctionAId)
          .EndJunction()
      // clang-format on
      ,
      maliput::common::assertion_error);
}

TEST_F(RoadGeometryBuilderTest, SegmentBuilderWithoutLane) {
  EXPECT_THROW(
      // clang-format off
      dut
          .StartJunction()
              .Id(kJunctionAId)
              .StartSegment()
                  .Id(kSegmentAId)
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
}

TEST_F(RoadGeometryBuilderTest, LaneBuilderWithoutLaneGeometry) {
  EXPECT_THROW(
      // clang-format off
      dut
          .StartJunction()
              .Id(kJunctionAId)
              .StartSegment()
                  .Id(kSegmentAId)
                  .StartLane()
                      .Id(kLaneAId)
                  .EndLane()
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
}

TEST_F(RoadGeometryBuilderTest, LaneGeometryBuilderWithMissingLineStrings) {
  // Fails because of missing left and right LineString3d.
  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionAId)
              .StartSegment()
                  .Id(kSegmentAId)
                  .StartLane()
                      .Id(kLaneAId)
                      .StartLaneGeometry()
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
  // Fails because of missing right LineString3d.
  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionAId)
              .StartSegment()
                  .Id(kSegmentAId)
                  .StartLane()
                      .Id(kLaneAId)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringA)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
  // Fails because of missing left LineString3d.
  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionAId)
              .StartSegment()
                  .Id(kSegmentAId)
                  .StartLane()
                      .Id(kLaneAId)
                      .StartLaneGeometry()
                          .RightLineString(kRightLineStringA)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
}

// No throws as LaneGeometryBuilder is properly used.
// Centerline isn't passed to the builder.
TEST_F(RoadGeometryBuilderTest, LaneGeometryBuilderWithoutCenter) {
  EXPECT_NO_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
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
              .EndSegment()
      // clang-format on
  );
}

// No throws as LaneGeometryBuilder is properly used.
// Centerline is submitted to the builder.
TEST_F(RoadGeometryBuilderTest, LaneGeometryBuilderWithCenter) {
  const auto kCenterline{geometry::utility::ComputeCenterline3d(kLeftLineStringA, kRightLineStringA)};
  EXPECT_NO_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionAId)
              .StartSegment()
                  .Id(kSegmentAId)
                  .StartLane()
                      .Id(kLaneAId)
                      .StartLaneGeometry()
                          .CenterLineString(kCenterline)
                          .LeftLineString(kLeftLineStringA)
                          .RightLineString(kRightLineStringA)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
      // clang-format on
  );
}

// Evaluates a case where all calls are executed at once and none of them fail their invariants.
TEST_F(RoadGeometryBuilderTest, CompleteCase) {
  // clang-format off
  std::unique_ptr<maliput::api::RoadGeometry> rg =
      dut.Id(kRoadGeometryId)
          .LinearTolerance(kLinearTolerance)
          .AngularTolerance(kAngularTolerance)
          .ScaleLength(kScaleLength)
          .InertialToBackendFrameTranslation(kInertialToBackendFrameTranslation)
          .StartJunction()
              .Id(kJunctionAId)
              .StartSegment()
                      .Id(kSegmentAId)
                  .StartLane()
                      .Id(kLaneAId)
                      .HeightBounds(kHBoundsA)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringA)  
                          .RightLineString(kRightLineStringA)
                      .EndLaneGeometry()
                  .EndLane()
                  .StartLane()
                      .Id(kLaneBId)
                      .HeightBounds(kHBoundsB)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringB)  
                          .RightLineString(kRightLineStringB)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
              .StartSegment()
                  .Id(kSegmentBId)
                  .StartLane()
                      .Id(kLaneCId)
                      .HeightBounds(kHBoundsC)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringC)  
                          .RightLineString(kRightLineStringC)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
          .EndJunction()
          .StartJunction()
              .StartSegment()
                  .StartLane()
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringD)  
                          .RightLineString(kRightLineStringD)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
          .EndJunction()
          .StartBranchPoints()
          .EndBranchPoints()
          .Build();
  // clang-format on

  ASSERT_NE(nullptr, rg);
  ASSERT_EQ(kRoadGeometryId, rg->id());
  ASSERT_EQ(2, rg->num_junctions());

  auto* junction_a = rg->junction(0);
  ASSERT_NE(nullptr, junction_a);
  ASSERT_EQ(kJunctionAId, junction_a->id());
  ASSERT_EQ(2, junction_a->num_segments());

  auto* segment_a = junction_a->segment(0);
  ASSERT_NE(nullptr, segment_a);
  ASSERT_EQ(kSegmentAId, segment_a->id());
  ASSERT_EQ(2, segment_a->num_lanes());

  auto* lane_a = segment_a->lane(0);
  ASSERT_NE(nullptr, lane_a);
  ASSERT_EQ(kLaneAId, lane_a->id());
  ASSERT_TRUE(IsHBoundsClose(kHBoundsA, lane_a->elevation_bounds(0., 0.), kEqualityTolerance));

  auto* lane_b = segment_a->lane(1);
  ASSERT_NE(nullptr, lane_b);
  ASSERT_EQ(kLaneBId, lane_b->id());
  ASSERT_TRUE(IsHBoundsClose(kHBoundsB, lane_b->elevation_bounds(0., 0.), kEqualityTolerance));

  ASSERT_EQ(lane_a, lane_b->to_right());
  ASSERT_EQ(lane_b, lane_a->to_left());

  auto* segment_b = junction_a->segment(1);
  ASSERT_NE(nullptr, segment_a);
  ASSERT_EQ(kSegmentAId, segment_a->id());
  ASSERT_EQ(2, segment_a->num_lanes());

  auto* lane_c = segment_b->lane(0);
  ASSERT_NE(nullptr, lane_c);
  ASSERT_EQ(kLaneCId, lane_c->id());
  ASSERT_TRUE(IsHBoundsClose(kHBoundsC, lane_c->elevation_bounds(0., 0.), kEqualityTolerance));

  auto* junction_b = rg->junction(1);
  ASSERT_NE(nullptr, junction_b);
  ASSERT_EQ(kJunctionBId, junction_b->id());
  ASSERT_EQ(1, junction_b->num_segments());

  auto* segment_c = junction_b->segment(0);
  ASSERT_NE(nullptr, segment_c);
  ASSERT_EQ(kSegmentCId, segment_c->id());
  ASSERT_EQ(1, segment_c->num_lanes());

  auto* lane_d = segment_c->lane(0);
  ASSERT_NE(nullptr, lane_d);
  ASSERT_EQ(kLaneDId, lane_d->id());
  ASSERT_TRUE(IsHBoundsClose(kHBoundsD, lane_d->elevation_bounds(0., 0.), kEqualityTolerance));

  ASSERT_EQ(8, rg->num_branch_points());
  // Configure the BranchPoint check struct.
  const auto start = maliput::api::LaneEnd::Which::kStart;
  const auto finish = maliput::api::LaneEnd::Which::kFinish;
  std::unordered_map<const maliput::api::Lane*, std::unordered_map<maliput::api::LaneEnd::Which, int>>
      branch_point_validation = {
          {lane_a, {{start, 0}, {finish, 0}}},
          {lane_b, {{start, 0}, {finish, 0}}},
          {lane_c, {{start, 0}, {finish, 0}}},
          {lane_d, {{start, 0}, {finish, 0}}},
      };
  for (int i = 0; i < 8; ++i) {
    const maliput::api::BranchPoint* bp = rg->branch_point(i);
    ASSERT_EQ(1, bp->GetASide()->size());
    const maliput::api::LaneEnd lane_end = bp->GetASide()->get(0);
    EXPECT_NO_THROW(branch_point_validation.at(lane_end.lane).at(lane_end.end) += 1);

    ASSERT_EQ(0, bp->GetBSide()->size());
  }
  ASSERT_EQ(1, branch_point_validation.at(lane_a).at(start));
  ASSERT_EQ(1, branch_point_validation.at(lane_a).at(finish));
  ASSERT_EQ(1, branch_point_validation.at(lane_b).at(start));
  ASSERT_EQ(1, branch_point_validation.at(lane_b).at(finish));
  ASSERT_EQ(1, branch_point_validation.at(lane_c).at(start));
  ASSERT_EQ(1, branch_point_validation.at(lane_c).at(finish));
  ASSERT_EQ(1, branch_point_validation.at(lane_d).at(start));
  ASSERT_EQ(1, branch_point_validation.at(lane_d).at(finish));
}

/**
  Builds a geometry like follows to show and test how to define connecting lanes
   <pre>
                            *
                           //
                          //
                         lane_b
                        //
                       //
   *======lane_a======*======lane_c======*
                       \\
                        \\
                         lane_d
                          \\
                           \\
                            *
   </pre>
**/
TEST_F(RoadGeometryBuilderTest, OutgoingBranches) {
  const LineString3d kLeftLineStringA{Vector3{0., 0., 0.}, Vector3{100., 0., 0.}};
  const LineString3d kRightLineStringA{Vector3{0., 5., 0.}, Vector3{100., 5., 0.}};
  const LineString3d kLeftLineStringB{Vector3{100., 0., 0.}, Vector3{200., 55., 0.}};
  const LineString3d kRightLineStringB{Vector3{100., 5., 0.}, Vector3{200., 50., 0.}};
  const LineString3d kLeftLineStringC{Vector3{100., 0., 0.}, Vector3{200., 0., 0.}};
  const LineString3d kRightLineStringC{Vector3{100., 5., 0.}, Vector3{200., 5., 0.}};
  const LineString3d kLeftLineStringD{Vector3{100., 0., 0.}, Vector3{200., -55., 0.}};
  const LineString3d kRightLineStringD{Vector3{100., 5., 0.}, Vector3{200., -50., 0.}};
  const auto start = maliput::api::LaneEnd::Which::kStart;
  const auto finish = maliput::api::LaneEnd::Which::kFinish;
  const maliput::api::SegmentId kSegmentCId{"segment_c"};
  const maliput::api::LaneId kLaneDId{"lane_d"};

  // clang-format off
  std::unique_ptr<maliput::api::RoadGeometry> rg =
      dut.Id(kRoadGeometryId)
          .LinearTolerance(kLinearTolerance)
          .AngularTolerance(kAngularTolerance)
          .ScaleLength(kScaleLength)
          .InertialToBackendFrameTranslation(kInertialToBackendFrameTranslation)
          .StartJunction()
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
              .EndSegment()
              .StartSegment()
                  .Id(kSegmentBId)
                  .StartLane()
                      .Id(kLaneBId)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringB)  
                          .RightLineString(kRightLineStringB)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
              .StartSegment()
                  .Id(kSegmentCId)
                  .StartLane()
                      .Id(kLaneCId)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringC)  
                          .RightLineString(kRightLineStringC)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
              .StartSegment()
                  .Id(kSegmentDId)
                  .StartLane()
                      .Id(kLaneDId)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringD)  
                          .RightLineString(kRightLineStringD)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
          .EndJunction()
          .StartBranchPoints()
              .Connect(kLaneAId, finish, kLaneBId, start)
              .Connect(kLaneAId, finish, kLaneCId, start)
              .Connect(kLaneAId, finish, kLaneDId, start)
          .EndBranchPoints()
          .Build();
  // clang-format on

  ASSERT_NE(nullptr, rg);
  ASSERT_EQ(kRoadGeometryId, rg->id());
  ASSERT_EQ(1, rg->num_junctions());

  auto* junction_a = rg->junction(0);
  ASSERT_NE(nullptr, junction_a);
  ASSERT_EQ(kJunctionAId, junction_a->id());
  ASSERT_EQ(4, junction_a->num_segments());

  auto* segment_a = junction_a->segment(0);
  ASSERT_NE(nullptr, segment_a);
  ASSERT_EQ(kSegmentAId, segment_a->id());
  ASSERT_EQ(1, segment_a->num_lanes());

  auto* lane_a = segment_a->lane(0);
  ASSERT_NE(nullptr, lane_a);
  ASSERT_EQ(kLaneAId, lane_a->id());

  auto* segment_b = junction_a->segment(1);
  ASSERT_NE(nullptr, segment_b);
  ASSERT_EQ(kSegmentBId, segment_b->id());
  ASSERT_EQ(1, segment_b->num_lanes());

  auto* lane_b = segment_b->lane(0);
  ASSERT_NE(nullptr, lane_b);
  ASSERT_EQ(kLaneBId, lane_b->id());

  auto* segment_c = junction_a->segment(2);
  ASSERT_NE(nullptr, segment_c);
  ASSERT_EQ(kSegmentCId, segment_c->id());
  ASSERT_EQ(1, segment_c->num_lanes());

  auto* lane_c = segment_c->lane(0);
  ASSERT_NE(nullptr, lane_c);
  ASSERT_EQ(kLaneCId, lane_c->id());

  auto* segment_d = junction_a->segment(3);
  ASSERT_NE(nullptr, segment_d);
  ASSERT_EQ(kSegmentDId, segment_d->id());
  ASSERT_EQ(1, segment_d->num_lanes());

  auto* lane_d = segment_d->lane(0);
  ASSERT_NE(nullptr, lane_d);
  ASSERT_EQ(kLaneDId, lane_d->id());

  ASSERT_EQ(5, rg->num_branch_points());

  auto lane_end_is_in = [](const maliput::api::LaneEnd& lane_end, const std::vector<maliput::api::LaneEnd>& lane_ends) {
    return std::find_if(lane_ends.begin(), lane_ends.end(), [lane_end](const auto& le) {
             return lane_end.lane == le.lane && lane_end.end == le.end;
           }) != lane_ends.end();
  };
  const std::vector<maliput::api::LaneEnd> expected_lane_ends{
      maliput::api::LaneEnd(lane_b, start), maliput::api::LaneEnd(lane_c, start), maliput::api::LaneEnd(lane_d, start)};
  const maliput::api::LaneEnd lane_end_a_finish(lane_a, finish);
  // Evaluates the BranchPoints from the Lanes.
  ASSERT_EQ(1, lane_a->GetConfluentBranches(start)->size());
  ASSERT_EQ(0, lane_a->GetOngoingBranches(start)->size());
  ASSERT_EQ(1, lane_a->GetConfluentBranches(finish)->size());
  ASSERT_EQ(3, lane_a->GetOngoingBranches(finish)->size());
  ASSERT_TRUE(lane_end_is_in(lane_a->GetOngoingBranches(finish)->get(0), expected_lane_ends));
  ASSERT_TRUE(lane_end_is_in(lane_a->GetOngoingBranches(finish)->get(1), expected_lane_ends));
  ASSERT_TRUE(lane_end_is_in(lane_a->GetOngoingBranches(finish)->get(2), expected_lane_ends));

  ASSERT_EQ(3, lane_b->GetConfluentBranches(start)->size());
  ASSERT_TRUE(lane_end_is_in(lane_b->GetConfluentBranches(start)->get(0), expected_lane_ends));
  ASSERT_TRUE(lane_end_is_in(lane_b->GetConfluentBranches(start)->get(1), expected_lane_ends));
  ASSERT_TRUE(lane_end_is_in(lane_b->GetConfluentBranches(start)->get(2), expected_lane_ends));
  ASSERT_EQ(1, lane_b->GetOngoingBranches(start)->size());
  ASSERT_TRUE(IsLaneEndEqual(lane_end_a_finish, lane_b->GetOngoingBranches(start)->get(0)));
  ASSERT_EQ(1, lane_b->GetConfluentBranches(finish)->size());
  ASSERT_EQ(0, lane_b->GetOngoingBranches(finish)->size());

  ASSERT_EQ(3, lane_c->GetConfluentBranches(start)->size());
  ASSERT_TRUE(lane_end_is_in(lane_c->GetConfluentBranches(start)->get(0), expected_lane_ends));
  ASSERT_TRUE(lane_end_is_in(lane_c->GetConfluentBranches(start)->get(1), expected_lane_ends));
  ASSERT_TRUE(lane_end_is_in(lane_c->GetConfluentBranches(start)->get(2), expected_lane_ends));
  ASSERT_EQ(1, lane_c->GetOngoingBranches(start)->size());
  ASSERT_TRUE(IsLaneEndEqual(lane_end_a_finish, lane_c->GetOngoingBranches(start)->get(0)));
  ASSERT_EQ(1, lane_c->GetConfluentBranches(finish)->size());
  ASSERT_EQ(0, lane_c->GetOngoingBranches(finish)->size());

  ASSERT_EQ(3, lane_d->GetConfluentBranches(start)->size());
  ASSERT_TRUE(lane_end_is_in(lane_d->GetConfluentBranches(start)->get(0), expected_lane_ends));
  ASSERT_TRUE(lane_end_is_in(lane_d->GetConfluentBranches(start)->get(1), expected_lane_ends));
  ASSERT_TRUE(lane_end_is_in(lane_d->GetConfluentBranches(start)->get(2), expected_lane_ends));
  ASSERT_EQ(1, lane_d->GetOngoingBranches(start)->size());
  ASSERT_TRUE(IsLaneEndEqual(lane_end_a_finish, lane_d->GetOngoingBranches(start)->get(0)));
  ASSERT_EQ(1, lane_d->GetConfluentBranches(finish)->size());
  ASSERT_EQ(0, lane_d->GetOngoingBranches(finish)->size());
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace maliput_sparse
