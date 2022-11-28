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
