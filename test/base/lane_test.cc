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
#include "base/lane.h"

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/common/assertion_error.h>
#include <maliput/geometry_base/segment.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_math_compare.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace test {
namespace {

using geometry::LineString3d;
using maliput::api::InertialPosition;
using maliput::api::IsoLaneVelocity;
using maliput::api::LaneId;
using maliput::api::LanePosition;
using maliput::api::LanePositionResult;
using maliput::api::RBounds;
using maliput::api::Rotation;
using maliput::api::test::IsInertialPositionClose;
using maliput::api::test::IsLanePositionClose;
using maliput::api::test::IsLanePositionResultClose;
using maliput::api::test::IsRotationClose;
using maliput::math::Vector2;
using maliput::math::Vector3;

struct LaneTestCase {
  LineString3d left{};
  LineString3d right{};
  std::vector<LanePosition> srh{};
  std::vector<InertialPosition> expected_backend_pos{};
  std::vector<Rotation> expected_rotation{};
  double expected_length{};
  std::vector<LanePositionResult> expected_lane_position_result{};
};

std::vector<LaneTestCase> LaneTestCases() {
  return {{
              // Straight lane:
              //   ______
              //  0______100m
              //
              LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
              LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
              {{0., 0., 0.}} /* srh */,
              {{0., 0., 0.}} /* expected_backend_pos */,
              {Rotation::FromRpy(0., 0., 0.)} /* expected_rotation */,
              100. /* expected_length */,
              {{
                  {0., 0., 0.} /* lane_position */, {0., 0., 0.} /* nearest_position */, 0. /* distance */
              }} /* expected_lane_position_result */
          },
          {
              // Arc-like lane:
              //    | |  --> no elevation change
              //  __/ /  --> no elevation change
              //  __ /   --> no elevation change
              LineString3d{{0., 2., 100.}, {100., 2., 100.}, {200., 102., 100.}, {200., 200., 100.}} /* left*/,
              LineString3d{{0., -2., 100.}, {100., -2., 100.}, {204., 102., 100.}, {204., 200., 100.}} /* right*/,
              {
                  {50., 0., 0.},
                  {50., -2., -2.},
                  {100. + 102. * std::sqrt(2.), 0., 0.},
                  {100. + 102. * std::sqrt(2.) + 98., -1., 4.},
              } /* srh */,
              {
                  {50., 0., 100.},
                  {50., -2., 98.},
                  {202., 102., 100.},
                  {203., 200., 104.},
              } /* expected_backend_pos */,
              {
                  Rotation::FromRpy(0., 0., 0.),
                  Rotation::FromRpy(0., 0., 0.),
                  Rotation::FromRpy(0., 0., M_PI / 4.),
                  Rotation::FromRpy(0., 0., M_PI / 2.),
              } /* expected_rotation */,
              100. + 102. * std::sqrt(2.) + 98. /* expected_length */,
              {{
                   {50., 0., 0.} /* lane_position */, {50., 0., 100.} /* nearest_position */, 0. /* distance */
               },
               {
                   {50., -2., -2.} /* lane_position */, {50., -2., 98.} /* nearest_position */, 0. /* distance */
               },
               {
                   {100. + 102. * std::sqrt(2.), 0., 0.} /* lane_position */,
                   {202., 102., 100.} /* nearest_position */,
                   0. /* distance */
               },
               {
                   {100. + 102. * std::sqrt(2.) + 98., -1., 4.} /* lane_position */,
                   {203., 200., 104.} /* nearest_position */,
                   0. /* distance */
               }}     /* expected_lane_position_result */
          },
          {
              // Lane with elevation:
              //  __
              //  __   --> linear elevation
              LineString3d{{0., 2., 0.}, {100., 2., 100.}} /* left*/,
              LineString3d{{0., -2., 0.}, {100., -2., 100.}} /* right*/,
              {
                  {50. * std::sqrt(2.), 0., 0.}, {50. * std::sqrt(2.), 4., 4.},  //< Off the lane
              } /* srh */,
              {
                  {50., 0., 50.},
                  {50. - 4. / std::sqrt(2.), 4., 50. + 4. / std::sqrt(2.)},
              } /* expected_backend_pos */,
              {
                  Rotation::FromRpy(0., -M_PI / 4., 0.),
                  Rotation::FromRpy(0., -M_PI / 4., 0.),
              } /* expected_rotation */,
              100. * std::sqrt(2.) /* expected_length */,
              {
                  {
                      {50. * std::sqrt(2.), 0., 0.} /* lane_position */,
                      {50., 0., 50.} /* nearest_position */,
                      0. /* distance */
                  },
                  {
                      {50. * std::sqrt(2.), 2., 4.} /* lane_position */,
                      {50. - 4. / std::sqrt(2.), 2., 50. + 4. / std::sqrt(2.)} /* nearest_position */,
                      2. /* distance */
                  },
              } /* expected_lane_position_result */
          },
          {
              // Lane with superelevation:
              //  __
              //  __   --> constant superelevation
              LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
              LineString3d{{0., -2., 4.}, {100., -2., 4.}} /* right*/,
              {
                  {50., 0., 0.},
                  {50., 2., 0.},
                  {50., -2., 0.},
              } /* srh */,
              {
                  {50., 0., 2.},
                  {50., 2. / std::sqrt(2.), 2. - 2. / std::sqrt(2.)},
                  {50., -2. / std::sqrt(2.), 2. + 2. / std::sqrt(2.)},
              } /* expected_backend_pos */,
              {
                  Rotation::FromRpy(-M_PI_4, 0., 0.),
                  Rotation::FromRpy(-M_PI_4, 0., 0.),
                  Rotation::FromRpy(-M_PI_4, 0., 0.),
              } /* expected_rotation */,
              100. /* expected_length */,
              {
                  {
                      {50., 0., 0.} /* lane_position */, {50., 0., 2.} /* nearest_position */, 0. /* distance */
                  },
                  {
                      {50., 2., 0.} /* lane_position */,
                      {50., 2. / std::sqrt(2.), 2. - 2. / std::sqrt(2.)} /* nearest_position */,
                      0. /* distance */
                  },
                  {
                      {50., -2., 0.} /* lane_position */,
                      {50., -2. / std::sqrt(2.), 2. + 2. / std::sqrt(2.)} /* nearest_position */,
                      0. /* distance */
                  },
              } /* expected_lane_position_result */
          }};
}

class LaneTest : public ::testing::TestWithParam<LaneTestCase> {
 public:
  static constexpr double kTolerance{1.e-5};
  static constexpr double kScaleLength{1.};

  void SetUp() override {
    ASSERT_EQ(case_.srh.size(), case_.expected_backend_pos.size()) << ">>>>> Test case is ill-formed.";
    ASSERT_EQ(case_.srh.size(), case_.expected_rotation.size()) << ">>>>> Test case is ill-formed.";
  }

  const maliput::api::LaneId kLaneId{"dut id"};
  const maliput::api::HBounds kHBounds{-5., 5.};
  LaneTestCase case_ = GetParam();
  std::unique_ptr<geometry::LaneGeometry> lane_geometry_ =
      std::make_unique<geometry::LaneGeometry>(case_.left, case_.right, kTolerance, kScaleLength);
};

TEST_P(LaneTest, Test) {
  std::unique_ptr<Lane> dut = std::make_unique<Lane>(kLaneId, kHBounds, std::move(lane_geometry_));
  EXPECT_DOUBLE_EQ(case_.expected_length, dut->length());
  for (std::size_t i = 0; i < case_.srh.size(); ++i) {
    const auto backend_pos = dut->ToBackendPosition(case_.srh[i]);
    const auto rpy = dut->GetOrientation(case_.srh[i]);
    const auto lane_position_result = dut->ToLanePositionBackend(case_.expected_backend_pos[i]);
    EXPECT_TRUE(IsRotationClose(case_.expected_rotation[i], rpy, kTolerance));
    EXPECT_TRUE(
        IsInertialPositionClose(case_.expected_backend_pos[i], InertialPosition::FromXyz(backend_pos), kTolerance));
    EXPECT_TRUE(IsLanePositionResultClose(case_.expected_lane_position_result[i], lane_position_result, kTolerance));
  }
}

INSTANTIATE_TEST_CASE_P(LaneTestGroup, LaneTest, ::testing::ValuesIn(LaneTestCases()));

// left / right
using LaneLimits = std::pair<LineString3d, LineString3d>;

struct ToLaneSegmentPositionTestCase {
  std::vector<LaneLimits> lanes;
  double expected_length{};
  RBounds segment_bounds{};  //-> at s=0
  std::vector<InertialPosition> backend_pos{};
  std::vector<LanePositionResult> expected_lane_position_result{};
  std::vector<LanePositionResult> expected_segment_lane_position_result{};
};

std::vector<ToLaneSegmentPositionTestCase> ToLaneSegmentPositionTestCases() {
  return {{
              // 5-lane straight road:
              //   _____________
              //  4_____________
              //  3_____________
              //  2_____________
              //  1_____________
              //  0_____________
              //
              {{
                   LineString3d{{0., -4., 0.}, {100., -4., 0.}} /* left */,
                   LineString3d{{0., -8., 0.}, {100., -8., 0.}} /* right */,
               },
               {
                   LineString3d{{0., 0., 0.}, {100., 0., 0.}} /* left */,
                   LineString3d{{0., -4., 0.}, {100., -4., 0.}} /* right */,
               },
               {
                   LineString3d{{0., 4., 0.}, {100., 4., 0.}} /* left */,
                   LineString3d{{0., 0., 0.}, {100., 0., 0.}} /* right */,
               },
               {
                   LineString3d{{0., 8., 0.}, {100., 8., 0.}} /* left */,
                   LineString3d{{0., 4., 0.}, {100., 4., 0.}} /* right */,
               },
               {
                   LineString3d{{0., 12., 0.}, {100., 12., 0.}} /* left */,
                   LineString3d{{0., 8., 0.}, {100., 8., 0.}} /* right */,
               }},
              100. /* expected_length */,
              {-10., 10.} /* segment_bounds */,
              {
                  // In the central lane's centerline.
                  {50., 2., 0.},
                  // In the adjacent's lane's centerline.
                  {50., 6., 0.},
                  // In the rightmost lane's right boundary.
                  {50., -8., 0.},
                  // Outside the segment boundaries.
                  {50., -80., 0.},
              } /* backend_pos */,
              {
                  {
                      {50., 0., 0.} /* lane_position */, {50., 2., 0.} /* nearest_position */, 0. /* distance */
                  },
                  {
                      {50., 2., 0.} /* lane_position */, {50., 4., 0.} /* nearest_position */, 2. /* distance */
                  },
                  {
                      {50., -2., 0.} /* lane_position */, {50., 0., 0.} /* nearest_position */, 8. /* distance */
                  },
                  {
                      {50., -2., 0.} /* lane_position */, {50., 0., 0.} /* nearest_position */, 80. /* distance */
                  },
              } /* expected_lane_position_result */,
              {
                  {
                      {50., 0., 0.} /* lane_position */, {50., 2., 0.} /* nearest_position */, 0. /* distance */
                  },
                  {
                      {50., 4., 0.} /* lane_position */, {50., 6., 0.} /* nearest_position */, 0. /* distance */
                  },
                  {
                      {50., -10., 0.} /* lane_position */, {50., -8., 0.} /* nearest_position */, 0. /* distance */
                  },
                  {
                      {50., -10., 0.} /* lane_position */, {50., -8., 0.} /* nearest_position */, 72. /* distance */
                  },
              } /* expected_segment_lane_position_result */
          },
          {
              // Arc-like lane:
              //    | |  --> no elevation
              //  __/ /  --> no elevation
              //  __ /   --> no elevation
              {{
                  LineString3d{{0., 2., 100.}, {100., 2., 100.}, {200., 102., 100.}, {200., 200., 100.}} /* left*/,
                  LineString3d{{0., -2., 100.}, {100., -2., 100.}, {204., 102., 100.}, {204., 200., 100.}} /* right*/,
                  // LineString3d{{0., 0., 0.}, {100., 0., 100.}, {202., 102, 100.}, {202., 200., 100.}} /* center */
              }},
              100. + 102. * std::sqrt(2.) + 98. /* expected_length */,
              {-2., 2.} /* segment_bounds */,
              {
                  {50., 0., 100.},
                  {50., 2., 100.},
                  {50., 10., 100.},
              } /* backend_pos */,
              {
                  // In the centerline.
                  {
                      {50., 0., 0.} /* lane_position */, {50., 0., 100.} /* nearest_position */, 0. /* distance */
                  },
                  // At the edge of the lane.
                  {
                      {50., 2., 0.} /* lane_position */, {50., 2., 100.} /* nearest_position */, 0. /* distance */
                  },
                  // Outside boundary of the lane.
                  {
                      // Because of the scaling of the boundaries' linestring the r value is slightly different.
                      {50., 2.042240, 0.} /* lane_position */,
                      {50., 2.042240, 100.} /* nearest_position */,
                      7.9577602284126803, /* distance */
                  },
              } /* expected_lane_position_result */,
              {
                  // In the centerline.
                  {
                      {50., 0., 0.} /* lane_position */, {50., 0., 100.} /* nearest_position */, 0. /* distance */
                  },
                  // At the edge of the lane.
                  {
                      {50., 2., 0.} /* lane_position */, {50., 2., 100.} /* nearest_position */, 0. /* distance */
                  },
                  // Outside boundary of the lane.
                  {
                      // Because of the scaling of the boundaries' linestring the r value is slightly different.
                      {50., 2.042240, 0.} /* lane_position */,
                      {50., 2.042240, 100.} /* nearest_position */,
                      7.9577602284126803, /* distance */
                  },
              } /* expected_segment_lane_position_result */
          }};
}

class ToLaneSegmentPositionTest : public ::testing::TestWithParam<ToLaneSegmentPositionTestCase> {
 public:
  static constexpr double kTolerance{1.e-5};
  static constexpr double kScaleLength{1.};

  void SetUp() override {
    ASSERT_EQ(case_.backend_pos.size(), case_.expected_lane_position_result.size()) << ">>>>> Test case is ill-formed.";
    for (int i = 0; i < static_cast<int>(case_.lanes.size()); ++i) {
      auto lane_geometry = std::make_unique<geometry::LaneGeometry>(case_.lanes[i].first, case_.lanes[i].second,
                                                                    kTolerance, kScaleLength);
      auto lane = std::make_unique<Lane>(LaneId(std::to_string(i)), kHBounds, std::move(lane_geometry));
      segment_.AddLane(std::move(lane));
    }
    // Store pointer of the lane in the middle of the segment.
    // When having odd number of lanes, from the two lanes in the middle the leftmost lane is picked.
    dut_ = dynamic_cast<const Lane*>(segment_.lane(segment_.num_lanes() / 2));
  }

  const maliput::api::HBounds kHBounds{-5., 5.};
  const Lane* dut_{nullptr};
  const ToLaneSegmentPositionTestCase case_ = GetParam();
  maliput::geometry_base::Segment segment_{maliput::api::SegmentId{"segment_id"}};
};

TEST_P(ToLaneSegmentPositionTest, Test) {
  EXPECT_DOUBLE_EQ(case_.expected_length, dut_->length());
  const RBounds seg_bounds = dut_->segment_bounds(0.);
  EXPECT_DOUBLE_EQ(case_.segment_bounds.min(), seg_bounds.min());
  EXPECT_DOUBLE_EQ(case_.segment_bounds.max(), seg_bounds.max());
  for (std::size_t i = 0; i < case_.backend_pos.size(); ++i) {
    const LanePositionResult lane_position_result = dut_->ToLanePositionBackend(case_.backend_pos[i]);
    EXPECT_TRUE(IsLanePositionResultClose(case_.expected_lane_position_result[i], lane_position_result, kTolerance));
    const LanePositionResult segment_position_result = dut_->ToSegmentPositionBackend(case_.backend_pos[i]);
    EXPECT_TRUE(
        IsLanePositionResultClose(case_.expected_segment_lane_position_result[i], segment_position_result, kTolerance));
  }
}

INSTANTIATE_TEST_CASE_P(ToLaneSegmentPositionTestGroup, ToLaneSegmentPositionTest,
                        ::testing::ValuesIn(ToLaneSegmentPositionTestCases()));

struct EvalMotionDerivativesTestCase {
  LineString3d left{};
  LineString3d right{};
  IsoLaneVelocity velocity{};
  std::vector<LanePosition> srh{};
  std::vector<LanePosition> expected_motion_derivatives{};
};

std::vector<EvalMotionDerivativesTestCase> EvalMotionDerivativesTestCases() {
  return {{
              // Straight lane:
              //   ______
              //  0______100m
              //
              LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
              LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
              {3., 2., 1.}, /* velocity */
              {{0., 0., 0.}, {50., 2., 0.}, {50., -2., 0.}} /* srh */,
              {{3., 2., 1.}, {3., 2., 1.}, {3., 2., 1.}} /* expected_motion_derivatives */
          },
          {
              // Arc-like no elevated lane:
              //    | |
              //  __/ /
              //  __ /
              LineString3d{{0., 2., 0.}, {100., 2., 0.}, {200., 102., 0.}, {200., 200., 0.}} /* left*/,
              LineString3d{{0., -2., 0.}, {100., -2., 0.}, {204., 102., 0.}, {204., 200., 0.}} /* right*/,
              // Centerline : {0., 0., 0.}, {100., 0., 0.}, {202., 102, 0.}, {202., 200., 0.}
              {3., 2., 1.}, /* velocity */
              {{50., 0., 0.},
               {50., -2., 0.},
               {100., 0., 0.},
               {100. + 50 * std::sqrt(2.), 0., 0.},
               {100. + 50 * std::sqrt(2.), 2., 0.}} /* srh */,
              {{3, 2., 1.}, {3, 2., 1.}, {3, 2., 1.}, {3, 2., 1.}, {3, 2., 1.}} /* expected_motion_derivatives */
          }};
}

class LaneEvalMotionDerivativesTestTest : public ::testing::TestWithParam<EvalMotionDerivativesTestCase> {
 public:
  static constexpr double kTolerance{1.e-5};
  static constexpr double kScaleLength{1.};

  void SetUp() override {
    ASSERT_EQ(case_.srh.size(), case_.expected_motion_derivatives.size()) << ">>>>> Test case is ill-formed.";
  }

  const maliput::api::LaneId kLaneId{"dut id"};
  const maliput::api::HBounds kHBounds{-5., 5.};
  EvalMotionDerivativesTestCase case_ = GetParam();
  std::unique_ptr<geometry::LaneGeometry> lane_geometry_ =
      std::make_unique<geometry::LaneGeometry>(case_.left, case_.right, kTolerance, kScaleLength);
};

TEST_P(LaneEvalMotionDerivativesTestTest, Test) {
  std::unique_ptr<Lane> dut = std::make_unique<Lane>(kLaneId, kHBounds, std::move(lane_geometry_));
  for (std::size_t i = 0; i < case_.srh.size(); ++i) {
    IsLanePositionClose(case_.expected_motion_derivatives[i], dut->EvalMotionDerivatives(case_.srh[i], case_.velocity),
                        kTolerance);
  }
}

INSTANTIATE_TEST_CASE_P(LaneEvalMotionDerivativesTestTestGroup, LaneEvalMotionDerivativesTestTest,
                        ::testing::ValuesIn(EvalMotionDerivativesTestCases()));

}  // namespace
}  // namespace test
}  // namespace maliput_sparse
