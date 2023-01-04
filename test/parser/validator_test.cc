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
#include "maliput_sparse/parser/validator.h"

#include <string>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_sparse/parser/parser.h"
#include "parser/validation_methods.h"

namespace maliput_sparse {
namespace parser {
namespace test {
namespace {

using maliput_sparse::geometry::LineString3d;

class MockParser : public Parser {
 public:
  MockParser(const std::unordered_map<Junction::Id, Junction>& junctions = {},
             const std::vector<Connection>& connections = {})
      : junctions_(junctions), connections_(connections) {}

 private:
  const std::unordered_map<Junction::Id, Junction>& DoGetJunctions() const override { return junctions_; }
  const std::vector<Connection>& DoGetConnections() const override { return connections_; }
  const std::unordered_map<Junction::Id, Junction> junctions_;
  const std::vector<Connection> connections_;
};

Lane CreateLane(const Lane::Id& id, const LineString3d& left, const LineString3d& right,
                const std::optional<Lane::Id>& left_lane_id, const std::optional<Lane::Id>& right_lane_id,
                const std::unordered_map<Lane::Id, LaneEnd>& successors,
                const std::unordered_map<Lane::Id, LaneEnd>& predecessors) {
  return Lane{id, left, right, left_lane_id, right_lane_id, successors, predecessors};
}

class ValidatorTest : public ::testing::Test {
 public:
  static MockParser CreateMockParser(const std::vector<Lane>& lanes) {
    const Junction junction{"junction", {{"segment", Segment{"segment", lanes}}}};
    return MockParser({{junction.id, junction}}, {});
  }

 protected:
  const Lane wrong_adj_lane{CreateLane("0", LineString3d{{1., 1., 1.}, {10., 1., 1.}},
                                       LineString3d{{1., -1., 1.}, {10., -1., 1.}}, {"23123"}, std::nullopt, {}, {})};
  const Lane lane_0{CreateLane("0", LineString3d{{1., 1., 1.}, {10., 1., 1.}},
                               LineString3d{{1., -1., 1.}, {10., -1., 1.}}, {"1"}, std::nullopt, {}, {})};
  const Lane not_geometrical_adj_lane_to_lane_0{CreateLane("1", LineString3d{{1., 3., 1.}, {10., 3., 1.}},
                                                           LineString3d{{1., 2., 1.}, {10., 2., 1.}}, std::nullopt,
                                                           {"0"}, {}, {})};
};

TEST_F(ValidatorTest, InvalidParser) {
  EXPECT_THROW(Validator(nullptr, {}, ValidatorConfig{1e-3}), maliput::common::assertion_error);
}

TEST_F(ValidatorTest, AllOptionsDisabled) {
  const Validator::Types types{/* No validation selected */};
  const MockParser parser = CreateMockParser({wrong_adj_lane});
  const auto errors = Validator(&parser, types, ValidatorConfig{1e-3})();
  ASSERT_EQ(0., errors.size());
}

// Wrong adjacent lane id. Only logical errors will be found.
TEST_F(ValidatorTest, LogicalLaneAdjacencyOptionCaseA) {
  const Validator::Types types{Validator::Type::kLogicalLaneAdjacency};
  const MockParser parser = CreateMockParser({wrong_adj_lane});
  const auto errors = Validator(&parser, types, ValidatorConfig{1e-3})();
  ASSERT_NE(0., errors.size());
  for (const auto& error : errors) {
    EXPECT_TRUE(error.type == Validator::Type::kLogicalLaneAdjacency);
  }
}

// As GeometryLaneAdjacency depends on LogicalLaneAdjacency, and there is only one lane then a logical error will be
// found.
TEST_F(ValidatorTest, GeometricalLaneAdjacencyOptionCaseA) {
  const Validator::Types types{Validator::Type::kGeometricalLaneAdjacency};
  const MockParser parser = CreateMockParser({wrong_adj_lane});
  const auto errors = Validator(&parser, types, ValidatorConfig{1e-3})();
  ASSERT_NE(0., errors.size());
  for (const auto& error : errors) {
    EXPECT_TRUE(error.type == Validator::Type::kLogicalLaneAdjacency);
  }
}

// Wrong geometric adjacency. No errors will be found
TEST_F(ValidatorTest, LogicalLaneAdjacencyOptionCaseB) {
  const Validator::Types types{Validator::Type::kLogicalLaneAdjacency};
  const MockParser parser = CreateMockParser({lane_0, not_geometrical_adj_lane_to_lane_0});
  const auto errors = Validator(&parser, types, ValidatorConfig{1e-3})();
  ASSERT_EQ(0., errors.size());
}

// Wrong geometric adjacency. Only geometrical errors will be found.
TEST_F(ValidatorTest, GeometricalLaneAdjacencyOptionCaseB) {
  const Validator::Types types{Validator::Type::kGeometricalLaneAdjacency};
  const MockParser parser = CreateMockParser({lane_0, not_geometrical_adj_lane_to_lane_0});
  const auto errors = Validator(&parser, types, ValidatorConfig{1e-3})();
  ASSERT_NE(0., errors.size());
  for (const auto& error : errors) {
    EXPECT_TRUE(error.type == Validator::Type::kGeometricalLaneAdjacency);
  }
}

class ValidateLaneAdjacencyTest : public ValidatorTest {
 public:
  const Lane lane_0 = CreateLane("0", LineString3d{{1., 1., 1.}, {10., 1., 1.}},
                                 LineString3d{{1., -1., 1.}, {10., -1., 1.}}, {"1"}, std::nullopt, {}, {});
  const Lane lane_1 = CreateLane("1", LineString3d{{1., 3., 1.}, {10., 3., 1.}},
                                 LineString3d{{1., 1., 1.}, {10., 1., 1.}}, {"2"}, {"0"}, {}, {});
  const Lane lane_2 = CreateLane("2", LineString3d{{1., 5., 1.}, {10., 5., 1.}},
                                 LineString3d{{1., 3., 1.}, {10., 3., 1.}}, {"3"}, {"1"}, {}, {});
  const Lane lane_3 = CreateLane("3", LineString3d{{1., 7., 1.}, {10., 7., 1.}},
                                 LineString3d{{1., 5., 1.}, {10., 5., 1.}}, std::nullopt, {"2"}, {}, {});
  const bool kEnableGeometricalValidation = true;
  const bool kDisableGeometricalValidation = !kEnableGeometricalValidation;
};

TEST_F(ValidateLaneAdjacencyTest, NoErrors) {
  const Segment segment{"segment", {lane_0, lane_1, lane_2, lane_3}};
  const Junction junction{"junction", {{segment.id, segment}}};
  const MockParser parser({{junction.id, junction}}, {});
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  EXPECT_EQ(0., errors.size());
}

TEST_F(ValidateLaneAdjacencyTest, NoValidRightLaneInTheSegment) {
  const std::vector<Validator::Error> expected_errors{
      {"Wrong ordering of lanes: Lane 1 has a left lane id (2) that has a right lane id (41856) that is not the lane "
       "1.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Adjacent lane isn't part of the segment: Lane 2 has a right lane id (41856) that is not part of the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 2 has a right lane id (41856) that is not the previous lane in the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
  };
  Lane lane_2_no_valid_right_id{lane_2};
  lane_2_no_valid_right_id.right_lane_id = "41856";
  const Segment segment{"segment", {lane_0, lane_1, lane_2_no_valid_right_id, lane_3}};
  const Junction junction{"junction", {{segment.id, segment}}};
  const MockParser parser({{junction.id, junction}}, {});
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoValidLeftLaneInTheSegment) {
  const std::vector<Validator::Error> expected_errors{
      {"Adjacent lane isn't part of the segment: Lane 2 has a left lane id (41856) that is not part of the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 2 has a left lane id (41856) that is not the next lane in the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 3 has a right lane id (2) that has a left lane id (41856) that is not the lane "
       "3.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
  };
  Lane lane_2_no_valid_left_id{lane_2};
  lane_2_no_valid_left_id.left_lane_id = "41856";
  const MockParser parser{CreateMockParser({lane_0, lane_1, lane_2_no_valid_left_id, lane_3})};
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoCorrespondenceForRightLane) {
  const std::vector<Validator::Error> expected_errors{
      {"Adjacent lane isn't part of the segment: Lane 0 has a left lane id (54656465) that is not part of the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 0 has a left lane id (54656465) that is not the next lane in the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 1 has a right lane id (0) that has a left lane id (54656465) that is not the "
       "lane 1.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 1 has no left lane id but it isn't the last lane in the segment segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 2 has a right lane id (1) that has no left lane id.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
  };
  Lane lane_0_with_other_left_lane_id{lane_0};
  lane_0_with_other_left_lane_id.left_lane_id = "54656465";
  Lane lane_1_without_left_lane_id{lane_1};
  lane_1_without_left_lane_id.left_lane_id = std::nullopt;
  const MockParser parser{
      CreateMockParser({lane_0_with_other_left_lane_id, lane_1_without_left_lane_id, lane_2, lane_3})};
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoCorrespondenceForLeftLane) {
  const std::vector<Validator::Error> expected_errors{
      {"Wrong ordering of lanes: Lane 1 has a left lane id (2) that has no right lane id.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 2 has no right lane id but it isn't the first lane in the segment segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 2 has a left lane id (3) that has a right lane id (54656465) that is not the "
       "lane 2.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Adjacent lane isn't part of the segment: Lane 3 has a right lane id (54656465) that is not part of the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 3 has a right lane id (54656465) that is not the previous lane in the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
  };
  Lane lane_2_without_right_lane_id{lane_2};
  lane_2_without_right_lane_id.right_lane_id = std::nullopt;
  Lane lane_3_with_other_right_lane_id{lane_3};
  lane_3_with_other_right_lane_id.right_lane_id = "54656465";
  const MockParser parser{
      CreateMockParser({lane_0, lane_1, lane_2_without_right_lane_id, lane_3_with_other_right_lane_id})};
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoRightLaneIdForANonFirstLane) {
  const std::vector<Validator::Error> expected_errors{
      {"Wrong ordering of lanes: Lane 0 has a left lane id (1) that has no right lane id.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 1 has no right lane id but it isn't the first lane in the segment segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
  };
  Lane lane_1_without_right_lane_id{lane_1};
  lane_1_without_right_lane_id.right_lane_id = std::nullopt;
  const MockParser parser{CreateMockParser({lane_0, lane_1_without_right_lane_id, lane_2, lane_3})};
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoLeftLaneIdForANonLastLane) {
  const std::vector<Validator::Error> expected_errors{
      {"Wrong ordering of lanes: Lane 1 has no left lane id but it isn't the last lane in the segment segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Wrong ordering of lanes: Lane 2 has a right lane id (1) that has no left lane id.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
  };
  Lane lane_1_without_left_lane_id{lane_1};
  lane_1_without_left_lane_id.left_lane_id = std::nullopt;
  const MockParser parser{CreateMockParser({lane_0, lane_1_without_left_lane_id, lane_2, lane_3})};
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoRightLanes) {
  const std::vector<Validator::Error> expected_errors{
      {"Wrong ordering of lanes: Lane 3 has a right lane id (2) but is the first lane in the segment segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Adjacent lane isn't part of the segment: Lane 3 has a right lane id (2) that is not part of the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
  };
  const MockParser parser{CreateMockParser({lane_3})};
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoLeftLanes) {
  const std::vector<Validator::Error> expected_errors{
      {"Wrong ordering of lanes: Lane 0 has a left lane id (1) but is the last lane in the segment segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
      {"Adjacent lane isn't part of the segment: Lane 0 has a left lane id (1) that is not part of the segment "
       "segment.",
       Validator::Type::kLogicalLaneAdjacency, Validator::Error::Severity::kError},
  };
  const MockParser parser{CreateMockParser({lane_0})};
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoGeometricallyAdjacentLanes) {
  Lane lane_0_no_geometrically_adj{lane_0};
  lane_0_no_geometrically_adj.left = LineString3d{{1., 7., 1.}, {10., 7., 1.}};
  const std::vector<Validator::Error> expected_errors{
      {"Lane 0 and lane 1 are not adjacent under the tolerance 0.001000.", Validator::Type::kGeometricalLaneAdjacency,
       Validator::Error::Severity::kError},
      {"Lane 1 and lane 0 are not adjacent under the tolerance 0.001000.", Validator::Type::kGeometricalLaneAdjacency,
       Validator::Error::Severity::kError},
  };
  const MockParser parser{CreateMockParser({lane_0_no_geometrically_adj, lane_1, lane_2, lane_3})};
  const auto errors = ValidateLaneAdjacency(&parser, kEnableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
  EXPECT_EQ(expected_errors, errors);
}

TEST_F(ValidateLaneAdjacencyTest, NoGeometricallyAdjacentLanesButNoGeometricalValidation) {
  Lane lane_0_no_geometrically_adj{lane_0};
  lane_0_no_geometrically_adj.left = LineString3d{{1., 7., 1.}, {10., 7., 1.}};
  const std::vector<Validator::Error> expected_errors{/* empty */};
  const MockParser parser{CreateMockParser({lane_0_no_geometrically_adj, lane_1, lane_2, lane_3})};
  const auto errors = ValidateLaneAdjacency(&parser, kDisableGeometricalValidation, ValidatorConfig{1e-3});
  ASSERT_EQ(expected_errors.size(), errors.size());
}

}  // namespace
}  // namespace test
}  // namespace parser
}  // namespace maliput_sparse
