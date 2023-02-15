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
#include "maliput_sparse/geometry/line_string.h"

#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>

namespace maliput_sparse {
namespace geometry {
namespace test {
namespace {

using maliput::math::Vector2;
using maliput::math::Vector3;

// Used tolerance for the length computation.
const constexpr double kTolerance{1e-15};

// Alternative function to compute the accumulated distance between points in a LineString.
template <typename VectorT>
struct SquaredDistanceFunction {
  double operator()(const VectorT& v1, const VectorT& v2) const { return std::pow((v1 - v2).norm(), 2.); }
};

class SegmentIntervalTest : public ::testing::Test {};

TEST_F(SegmentIntervalTest, Constructors) {
  EXPECT_NO_THROW((LineString3d::Segment::Interval{0., 1.}));
  EXPECT_THROW((LineString3d::Segment::Interval{1., 0.}), maliput::common::assertion_error);
}

TEST_F(SegmentIntervalTest, ValuesUsing2ArgsConstructor) {
  const LineString3d::Segment::Interval dut(0., 1.);
  EXPECT_EQ(0., dut.min);
  EXPECT_EQ(1., dut.max);
}

TEST_F(SegmentIntervalTest, ValuesUsing1ArgsConstructor) {
  const LineString3d::Segment::Interval dut(10.);
  EXPECT_EQ(10., dut.min);
  EXPECT_EQ(10., dut.max);
}

TEST_F(SegmentIntervalTest, OperatorLessThan) {
  const LineString3d::Segment::Interval dut(0., 10.);
  // True because dut's min is less than the other's min.
  EXPECT_TRUE(dut < LineString3d::Segment::Interval(5., 15.));
  // False because dut's min is greater than the other's max.
  EXPECT_FALSE(dut < LineString3d::Segment::Interval(-5., 0.));
  // True because both p are greater than dut's.
  EXPECT_TRUE(dut < LineString3d::Segment::Interval(15., 20.));
  // False because the intervals are equal.
  EXPECT_FALSE(dut < LineString3d::Segment::Interval(0., 10.));
}

TEST_F(SegmentIntervalTest, Map) {
  std::map<LineString3d::Segment::Interval, int> map{
      {LineString3d::Segment::Interval(0., 10.), 0},  {LineString3d::Segment::Interval(10., 20.), 1},
      {LineString3d::Segment::Interval(20., 30.), 2}, {LineString3d::Segment::Interval(30., 40.), 3},
      {LineString3d::Segment::Interval(40., 50.), 4}, {LineString3d::Segment::Interval(50., 60.), 5},
      {LineString3d::Segment::Interval(60., 70.), 6}, {LineString3d::Segment::Interval(70., 80.), 7},
      {LineString3d::Segment::Interval(80., 90.), 8}, {LineString3d::Segment::Interval(90., 100.), 9},
  };
  EXPECT_EQ(0, map.at(LineString3d::Segment::Interval(0.)));
  EXPECT_EQ(0, map.at(LineString3d::Segment::Interval(5.)));
  EXPECT_EQ(1, map.at(LineString3d::Segment::Interval(10.)));
  EXPECT_EQ(1, map.at(LineString3d::Segment::Interval(15.)));
  EXPECT_EQ(2, map.at(LineString3d::Segment::Interval(20.)));
}

class SegmentTest : public ::testing::Test {
 public:
  const std::vector<maliput::math::Vector3> points_3d_ = {
      maliput::math::Vector3(0., 0., 0.),
      maliput::math::Vector3(10., 0., 0.),
      maliput::math::Vector3(20., 0., 0.),
  };
  LineString3d::Segment::Interval interval_1{0., 10.};
  LineString3d::Segment::Interval interval_2{10., 20.};
};

TEST_F(SegmentTest, Constructor) { EXPECT_NO_THROW((LineString3d::Segment{0, 1, interval_1})); }

TEST_F(SegmentTest, Values) {
  const LineString3d::Segment dut_1{0, 1, interval_1};
  EXPECT_EQ(static_cast<std::size_t>(0), dut_1.idx_start);
  EXPECT_EQ(static_cast<std::size_t>(1), dut_1.idx_end);
  EXPECT_EQ(interval_1.min, dut_1.p_interval.min);
  EXPECT_EQ(interval_1.max, dut_1.p_interval.max);

  const LineString3d::Segment dut_2{1, 2, interval_2};
  EXPECT_EQ(static_cast<std::size_t>(1), dut_2.idx_start);
  EXPECT_EQ(static_cast<std::size_t>(2), dut_2.idx_end);
  EXPECT_EQ(interval_2.min, dut_2.p_interval.min);
  EXPECT_EQ(interval_2.max, dut_2.p_interval.max);
}

class PointTest : public ::testing::Test {};

TEST_F(PointTest, BadConstructors) {
  const maliput::math::Vector3 coordinate{1., 2., 3.};
  EXPECT_THROW((LineString3d::Point{coordinate, 0, -1.}), maliput::common::assertion_error);
}

TEST_F(PointTest, Constructors) {
  {
    const LineString3d::Point dut{};
    ASSERT_NE(nullptr, dut.coordinate());
    EXPECT_EQ(maliput::math::Vector3{}, *dut.coordinate());
    EXPECT_EQ(std::nullopt, dut.p());
    EXPECT_EQ(std::nullopt, dut.idx());
  }
  {
    const maliput::math::Vector3 coordinate{1., 2., 3.};
    const LineString3d::Point dut(coordinate);
    ASSERT_NE(nullptr, dut.coordinate());
    EXPECT_EQ(coordinate, *dut.coordinate());
    EXPECT_EQ(std::nullopt, dut.p());
    EXPECT_EQ(std::nullopt, dut.idx());
  }
  {
    const maliput::math::Vector3 coordinate{1., 2., 3.};
    const std::size_t idx = 213;
    const double p = 0.123;
    const LineString3d::Point dut(coordinate, idx, p);
    ASSERT_NE(nullptr, dut.coordinate());
    EXPECT_EQ(coordinate, *dut.coordinate());
    EXPECT_EQ(p, dut.p());
    EXPECT_EQ(idx, dut.idx());
  }
}

class LineString3dTest : public ::testing::Test {
 public:
  const Vector3 p1{Vector3::UnitX()};
  const Vector3 p2{Vector3::UnitY()};
  const Vector3 p3{Vector3::UnitZ()};
};

TEST_F(LineString3dTest, ConstructorWithInitializerListIsSuccessful) { const LineString3d dut{p1, p2, p3}; }

TEST_F(LineString3dTest, ConstructorWithVectorIsSuccessful) {
  const LineString3d dut(std::vector<Vector3>{p1, p2, p3});
}

TEST_F(LineString3dTest, ConstructorWithIteratorsIsSucceful) {
  const std::array<Vector3, 3> points{p1, p2, p3};
  const LineString3d dut(points.begin(), points.end());
}

TEST_F(LineString3dTest, Api) {
  const LineString3d dut(std::vector<Vector3>{p1, p2, p3});
  EXPECT_TRUE(p1 == dut.first());
  EXPECT_TRUE(p1 == *dut.begin());
  EXPECT_TRUE(p3 == dut.last());
  EXPECT_TRUE(p3 == *(dut.end() - 1));
  EXPECT_TRUE(p1 == dut.at(0));
  EXPECT_TRUE(p2 == dut.at(1));
  EXPECT_TRUE(p3 == dut.at(2));
  EXPECT_EQ(static_cast<size_t>(3), dut.size());
  EXPECT_NEAR(2. * std::sqrt(2.), dut.length(), kTolerance);
}

TEST_F(LineString3dTest, SameConsecutivePoints) {
  const LineString3d dut(std::vector<Vector3>{p1, p2, p3, p3, p2, p1, p1, p2});
  const std::vector<Vector3> expected_points{p1, p2, p3, p2, p1, p2};
  ASSERT_EQ(expected_points.size(), dut.size());
  for (size_t i = 0; i < dut.size() - 1; ++i) {
    EXPECT_EQ(expected_points.at(i), dut.at(i));
  }
}

TEST_F(LineString3dTest, Segments) {
  const LineString3d dut(std::vector<Vector3>{p1, p2, p3});
  const auto segments = dut.segments();
  ASSERT_EQ(static_cast<size_t>(2), segments.size());

  const double p_in_p1_p2 = (p1 - p2).norm() / 2.;
  const double p_in_p2_p3 = (p1 - p2).norm() + (p2 - p3).norm() / 2.;

  const auto segment_p1_p2 = segments.at(LineString3d::Segment::Interval(p_in_p1_p2));
  EXPECT_TRUE(p1 == dut[segment_p1_p2.idx_start]);
  EXPECT_TRUE(p2 == dut[segment_p1_p2.idx_end]);
  EXPECT_NEAR(0., segment_p1_p2.p_interval.min, kTolerance);
  EXPECT_NEAR((p1 - p2).norm(), segment_p1_p2.p_interval.max, kTolerance);

  const auto segment_p2_p3 = segments.at(LineString3d::Segment::Interval(p_in_p2_p3));
  EXPECT_TRUE(p2 == dut[segment_p2_p3.idx_start]);
  EXPECT_TRUE(p3 == dut[segment_p2_p3.idx_end]);
  EXPECT_NEAR((p1 - p2).norm(), segment_p2_p3.p_interval.min, kTolerance);
  EXPECT_NEAR((p1 - p2).norm() + (p2 - p3).norm(), segment_p2_p3.p_interval.max, kTolerance);
}

TEST_F(LineString3dTest, Points) {
  const LineString3d dut(std::vector<Vector3>{p1, p2, p3});
  const std::vector<LineString3d::Point>& points = dut.points();

  ASSERT_EQ(static_cast<size_t>(3), points.size());

  std::size_t idx{0};
  ASSERT_NE(nullptr, points.at(idx).coordinate());
  EXPECT_EQ(p1, *(points.at(idx).coordinate()));
  ASSERT_NE(std::nullopt, points.at(idx).p());
  EXPECT_EQ(0., points.at(idx).p().value());
  ASSERT_NE(std::nullopt, points.at(idx).idx());
  EXPECT_EQ(idx, points.at(0).idx().value());

  idx = 1;
  ASSERT_NE(nullptr, points.at(idx).coordinate());
  EXPECT_EQ(p2, *(points.at(idx).coordinate()));
  ASSERT_NE(std::nullopt, points.at(idx).p());
  EXPECT_EQ((p2 - p1).norm(), points.at(idx).p().value());
  ASSERT_NE(std::nullopt, points.at(idx).idx());
  EXPECT_EQ(idx, points.at(idx).idx().value());

  idx = 2;
  ASSERT_NE(nullptr, points.at(idx).coordinate());
  EXPECT_EQ(p3, *(points.at(idx).coordinate()));
  ASSERT_NE(std::nullopt, points.at(idx).p());
  EXPECT_EQ((p2 - p1).norm() + (p3 - p2).norm(), points.at(idx).p().value());
  ASSERT_NE(std::nullopt, points.at(idx).idx());
  EXPECT_EQ(idx, points.at(idx).idx().value());
}

TEST_F(LineString3dTest, KDTree) {
  const LineString3d dut(std::vector<Vector3>{p1, p2, p3});
  const auto kd_tree = dut.kd_tree();
  ASSERT_NE(nullptr, kd_tree);
  const auto res = kd_tree->nearest_point(LineString3d::Point{p2});
  EXPECT_EQ(p2, *res.coordinate());
}

TEST_F(LineString3dTest, LengthInjectedDistanceFunction) {
  const LineString<Vector3, SquaredDistanceFunction<Vector3>> dut(std::vector<Vector3>{p1, p2, p3});
  EXPECT_NEAR(4., dut.length(), 1e-14);
}

class LineString2dTest : public ::testing::Test {
 public:
  const Vector2 p1{Vector2::UnitX()};
  const Vector2 p2{Vector2::UnitY()};
  const Vector2 p3{1., 1.};
};

TEST_F(LineString2dTest, ConstructorWithInitializerListIsSuccessful) { const LineString2d dut{p1, p2, p3}; }

TEST_F(LineString2dTest, ConstructorWithVectorIsSuccessful) {
  const LineString2d dut(std::vector<Vector2>{p1, p2, p3});
}

TEST_F(LineString2dTest, ConstructorWithIteratorsIsSucceful) {
  const std::array<Vector2, 3> points{p1, p2, p3};
  const LineString2d dut(points.begin(), points.end());
}

TEST_F(LineString2dTest, Api) {
  const LineString2d dut(std::vector<Vector2>{p1, p2, p3});
  EXPECT_TRUE(p1 == dut.first());
  EXPECT_TRUE(p3 == dut.last());
  EXPECT_TRUE(p1 == dut.at(0));
  EXPECT_TRUE(p2 == dut.at(1));
  EXPECT_TRUE(p3 == dut.at(2));
  EXPECT_EQ(static_cast<size_t>(3), dut.size());
  EXPECT_NEAR(1. + std::sqrt(2.), dut.length(), kTolerance);
}

TEST_F(LineString2dTest, LengthWithInjectedDistanceFunction) {
  const LineString<Vector2, SquaredDistanceFunction<Vector2>> dut(std::vector<Vector2>{p1, p2, p3});
  EXPECT_NEAR(3., dut.length(), kTolerance);
}

}  // namespace
}  // namespace test
}  // namespace geometry
}  // namespace maliput_sparse
