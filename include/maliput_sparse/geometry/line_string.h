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
#include <map>
#include <vector>

#include <maliput/common/logger.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/math/kd_tree.h>
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

  /// A segment of a LineString.
  /// A segment is defined by a:
  /// - start index: index of the first coordinate of the segment.
  /// - end index: index of the last coordinate of the segment, it is expected to be `start index + 1` as `LineString`'s
  /// constructor
  ///              builds the segments from consecutive coordinates.
  struct Segment {
    /// Defines an interval in the @f$ p @f$ value of the parametrized LineString.
    /// The Less than operator is defined to allow the use of this struct as a key in a collection like std::map.
    struct Interval {
      /// Creates a Interval.
      /// @param min_in Is the minimum value of the interval.
      /// @param max_in Is the maximum value of the interval.
      /// @throw maliput::common::assertion_error When `min_in` is greater than `max_in`.
      Interval(double min_in, double max_in) : min(min_in), max(max_in) { MALIPUT_THROW_UNLESS(min_in <= max_in); }

      /// Creates a Interval where
      /// the minimum value is equal to the maximum value.
      /// @param min_max Is the minimum and maximum value of the interval.
      Interval(double min_max) : min(min_max), max(min_max) {}

      /// Less than operator.
      bool operator<(const Interval& rhs) const {
        if (min < rhs.min) {
          return max <= rhs.max ? true : false;
        } else {
          return false;
        }
      }

      // Min p value.
      double min{};
      // Max p value.
      double max{};
    };

    /// Start index of the segment.
    std::size_t idx_start;
    /// End index of the segment.
    std::size_t idx_end;
    /// Interval of the segment.
    Segment::Interval p_interval;
  };

  using Segments = std::map<typename Segment::Interval, Segment>;

  /// Extends the CoordinateT class with information about the index of the coordinate in the LineString and the
  /// @f$ p @f$ value up-to the coordinate in the parametrized LineString.
  /// Convenient to use with the KDTree.
  class Point : public CoordinateT {
   public:
    /// Default constructor.
    Point() = default;

    /// Creates a Point.
    /// @param coordinate The coordinate of the point.
    /// @param idx The index of the coordinate in the LineString.
    /// @param p The @f$ p @f$ value up-to the coordinate in the parametrized LineString.
    /// @throw maliput::common::assertion_error When `p` is negative.
    Point(const CoordinateT& coordinate, std::size_t idx, double p) : CoordinateT(coordinate), idx_(idx), p_(p) {
      MALIPUT_THROW_UNLESS(p >= 0.);
    }

    /// Creates a point
    /// @param coordinate The coordinate of the point.
    explicit Point(const CoordinateT& coordinate) : CoordinateT(coordinate) {}

    /// @return If provided via constructor, the index of the coordinate in the LineString, std::nullopt otherwise.
    std::optional<std::size_t> idx() const { return idx_; }
    /// @return If provided via constructor, the @f$ p @f$ value up-to the coordinate in the parametrized LineString,
    /// std::nullopt otherwise.
    std::optional<double> p() const { return p_; }

    /// @return The underlying coordinate of the point.
    const CoordinateT* coordinate() const { return this; }

   private:
    std::optional<std::size_t> idx_{};
    std::optional<double> p_{};
  };

  using KDTree = maliput::math::KDTree<Point, CoordinateT::kDimension>;

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
    // Remove consecutive points that are numerically the same.
    // Duplicated points creates zero length segments, which leads to a wrong lookup when querying the segment map.
    std::vector<std::size_t> remove_idx;
    for (std::size_t idx{}; idx < coordinates_.size() - 1; ++idx) {
      const double segment_length = DistanceFunction()(coordinates_[idx], coordinates_[idx + 1]);
      if (segment_length <= std::numeric_limits<double>::epsilon()) {
        maliput::log()->warn("LineString: consecutive points are numerically the same, removing duplicated point: {}",
                             coordinates_[idx + 1]);
        remove_idx.push_back(idx + 1);
      }
    }
    // Remove the duplicated points.
    for (auto it = remove_idx.rbegin(); it != remove_idx.rend(); ++it) {
      coordinates_.erase(coordinates_.begin() + *it);
    }
    MALIPUT_THROW_UNLESS(coordinates_.size() > 1);
    // Fill up the segments collection and the points collection.
    double p = 0;
    points_.reserve(coordinates_.size());
    for (std::size_t idx{}; idx < coordinates_.size() - 1; ++idx) {
      const double segment_length = DistanceFunction()(coordinates_[idx], coordinates_[idx + 1]);
      const typename Segment::Interval interval{p, p + segment_length};
      // Add the segment.
      segments_.emplace(interval, Segment{idx, idx + 1, interval});
      // Add the point.
      points_.push_back(Point(coordinates_[idx], idx, p));
      p += segment_length;
    }
    // Add the last point.
    points_.push_back(Point(coordinates_[coordinates_.size() - 1], coordinates_.size() - 1, p));
    // Build the KDTree.
    kd_tree_ = std::make_shared<KDTree>(points_.begin(), points_.end());
    // Set the length.
    length_ = p;
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

  /// Return the segments of this LineString.
  /// @return A vector of segments.
  const Segments& segments() const { return segments_; }

  /// Return the points of this LineString.
  const std::vector<Point>& points() const { return points_; }

  /// Get KD Tree of the LineString.
  const KDTree* kd_tree() const { return kd_tree_.get(); }

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
  std::vector<CoordinateT> coordinates_{};
  std::vector<Point> points_{};
  Segments segments_{};
  double length_{};
  std::shared_ptr<KDTree> kd_tree_;
};

// Convenient aliases.
using LineString2d = LineString<maliput::math::Vector2>;
using LineString3d = LineString<maliput::math::Vector3>;

}  // namespace geometry
}  // namespace maliput_sparse
