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
#pragma once

#include <cmath>
#include <initializer_list>
#include <iterator>
#include <vector>

#include <maliput/common/maliput_throw.h>
#include <maliput/math/vector.h>

namespace maliput_sparse {
namespace geometry {
namespace details {

/// Calculates the Euclidean distance between two coordinates.
/// @tparam CoordinateT The type of the coordinates.
template <typename CoordinateT>
struct EuclideanDistance {
  /// Obtains Euclidean distance between two coordinates.
  /// @param lhs First point.
  /// @param rhs Second point.
  /// @returns The Euqclidean distance between the two coordinates.
  /// @throws maliput::common::assertion_error When coordinate sizes are different.
  double operator()(const CoordinateT& lhs, const CoordinateT& rhs) const {
    MALIPUT_THROW_UNLESS(lhs.size() == rhs.size());
    double dist = 0;
    for (std::size_t i = 0; i < lhs.size(); ++i) {
      const double d = lhs[i] - rhs[i];
      dist += d * d;
    }
    return std::sqrt(dist);
  }
};

}  // namespace details

/// Defines a polyline in the `CoordinateT` domain composed of at least 2 points.
///
/// @tparam CoordinateT The coordinate point type to be used.
/// @tparam DistanceFunction The function to compute the distance with. By default, the Euclidean
/// distance is used.
template <typename CoordinateT, typename DistanceFunction = details::EuclideanDistance<CoordinateT>>
class LineString final {
 public:
  using iterator = typename std::vector<CoordinateT>::iterator;
  using const_iterator = typename std::vector<CoordinateT>::const_iterator;

  /// Constructs a LineString from a std::vector.
  ///
  /// This function calls LineString(coordinates.begin, coordinates.end)
  ///
  /// @param coordinates A vector of CoordinateT to define this LineString.
  explicit LineString(const std::vector<CoordinateT>& coordinates)
      : LineString(coordinates.begin(), coordinates.end()) {}

  /// Constructs a LineString form an initializer list.
  ///
  /// This function calls LineString(coordinates.begin, coordinates.end)
  ///
  /// @param coordinates An initializer list to define this LineString.
  explicit LineString(std::initializer_list<CoordinateT> coordinates)
      : LineString(coordinates.begin(), coordinates.end()) {}

  /// Constructs a LineString form the begin and end iterators.
  ///
  /// After loading the coordinates, this iterates through the list of points and computes the total length.
  /// This operation is O(n) in the list size.
  ///
  /// @param begin The initial iterator.
  /// @param end The final iterator.
  /// @tparam Iterator the iterator type.
  /// @throws maliput::common::assertion_error When there are less than two points.
  template <typename Iterator>
  LineString(Iterator begin, Iterator end) : coordinates_(begin, end) {
    MALIPUT_THROW_UNLESS(coordinates_.size() > 1);
    length_ = ComputeLength();
  }

  /// @return The first point in the LineString.
  const CoordinateT& first() const { return coordinates_.front(); }

  /// @return The last point in the LineString.
  const CoordinateT& last() const { return coordinates_.back(); }

  /// @return The point at @p i index in the LineString.
  /// @throws std::out_of_range When @p i is not in [0, size()) bounds.
  const CoordinateT& at(size_t i) const { return coordinates_.at(i); }

  /// @return The number of points this LineString has.
  size_t size() const { return coordinates_.size(); }

  /// @return The accumulated length between consecutive points in this LineString by means of DistanceFunction.
  double length() const { return length_; }

  /// @returns begin iterator of the underlying collection.
  iterator begin() { return coordinates_.begin(); }
  /// @returns begin const iterator of the underlying collection.
  const_iterator begin() const { return coordinates_.begin(); }
  /// @returns end iterator of the underlying collection.
  iterator end() { return coordinates_.end(); }
  /// @returns end const iterator of the underlying collection.
  const_iterator end() const { return coordinates_.end(); }

  const CoordinateT& operator[](std::size_t index) const { return coordinates_[index]; }
  CoordinateT& operator[](std::size_t index) { return coordinates_[index]; }

  /// Equality operator.
  bool operator==(const LineString<CoordinateT, DistanceFunction>& other) const {
    return coordinates_ == other.coordinates_;
  }

 private:
  // @return The accumulated Length of this LineString.
  const double ComputeLength() const {
    double accumulated_{0.};
    for (size_t i = 0; i < size() - 1; ++i) {
      accumulated_ += DistanceFunction()(at(i), at(i + 1));
    }
    return accumulated_;
  }

  std::vector<CoordinateT> coordinates_{};
  double length_{};
};

// Convenient aliases.
using LineString2d = LineString<maliput::math::Vector2>;
using LineString3d = LineString<maliput::math::Vector3>;

}  // namespace geometry
}  // namespace maliput_sparse
