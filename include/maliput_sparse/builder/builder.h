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

/// @file
/// Builder API to construct a maliput::api::RoadGeometry.
///
/// @details It allows simple construction of nested nodes in the maliput::api::RoadGeometry graph. The
/// geometry details are expected to be computed / loaded outside this method to allow parallelization
/// of that process. The graph construction is a synchronous operation though and requires
/// that the geometry information is available when building maliput::api::Lanes.
///
/// A super simple 2-lane dragway could be constructed as follows:
///
/// @code{cpp}
/// const maliput::math:Vector3 start_left_lane_1{0., 0., 0.};
/// const maliput::math:Vector3 end_left_lane_1{100., 0., 0.};
/// const maliput::math:Vector3 start_right_lane_1{0., 5., 0.};
/// const maliput::math:Vector3 end_right_lane_1{100., 5., 0.};
/// const maliput::math:Vector3 start_left_lane_2 = start_right_lane_1;
/// const maliput::math:Vector3 end_left_lane_2 = end_right_lane_1;
/// const maliput::math:Vector3 start_right_lane_2{0., 10., 0.};
/// const maliput::math:Vector3 end_right_lane_2{100., 10., 0.};
/// const maliput::api::HBounds hbounds_l1{0., 5.};
/// const maliput::api::HBounds hbounds_l2{0., 10.};
///
/// LineString3d left_line_string_1{start_left_lane_1, end_left_lane_1};
/// LineString3d right_line_string_1{start_right_lane_1, end_right_lane_1};
/// LineString3d left_line_string_2{start_left_lane_2, end_left_lane_2};
/// LineString3d right_line_string_2{start_right_lane_2, end_right_lane_2};
///
/// RoadGeometryBuilder rg_builder;
/// std::unique_ptr<RoadGeometry> road_geometry = RoadGeometryBuilder()
///   .Id("two_lane_dragway")
///   .StartJunction()
///     .Id("j0")
///     .StartSegment()
///       .Id("j0_s0")
///       .StartLane()
///         .Id("j0_s0_l1")
///         .HeightBounds(hbounds_l1)
///         .StartLaneGeometry()
///           .LeftLineString(left_line_string_1)
///           .RightLineString(right_line_string_1)
///         .EndLaneGeometry()
///       .EndLane()
///       .StartLane()
///         .Id("j0_s0_l2")
///         .HeightBounds(hbounds_l2)
///         .StartLaneGeometry()
///           .LeftLineString(left_line_string_2)
///           .RightLineString(right_line_string_2)
///         .EndLaneGeometry()
///       .EndLane()
///     .EndSegment()
///   .EndJunction()
///   .StartBranchPoints()
///   .EndBranchPoints()
///   .Build();
/// @endcode{cpp}

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>
#include <maliput/common/passkey.h>
#include <maliput/geometry_base/branch_point.h>
#include <maliput/geometry_base/junction.h>
#include <maliput/geometry_base/lane.h>
#include <maliput/geometry_base/road_geometry.h>
#include <maliput/geometry_base/segment.h>

#include "maliput_sparse/geometry/lane_geometry.h"
#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace builder {
namespace details {

/// @brief Holds the parent Builder class and offers a small set of convenient methods to manage
/// the Builder lifecycle.
/// @tparam ParentT The parent BuilderBase type.
template <typename ParentT>
class NestedBuilder {
 public:
  virtual ~NestedBuilder() = default;

  /// @brief Construct a new nested builder object.
  /// @param parent The pointer to the parent builder. Must not be nullptr.
  /// @throws maliput::common::assertion_error When @p parent is nulltr.
  explicit NestedBuilder(ParentT* parent) : parent_(parent) { MALIPUT_THROW_UNLESS(parent_ != nullptr); }

  /// @return A reference to the parent builder.
  ParentT& End() { return *parent_; }

  /// @return A pointer to the parent builder.
  ParentT* Parent() { return parent_; }

 private:
  ParentT* parent_{};
};

}  // namespace details

// Forward declaration of the classes in this headerfile to enable their use before their full declaration.
class RoadGeometryBuilder;
class JunctionBuilder;
class SegmentBuilder;
class LaneBuilder;

/// @brief Builder class for maliput_sparse::geometry::LaneGeometry.
class LaneGeometryBuilder final : public details::NestedBuilder<LaneBuilder> {
 public:
  /// @brief Construct a new Lane Geometry Builder object.
  /// @param parent The parent LaneBuilder. It must not be nullptr.
  explicit LaneGeometryBuilder(LaneBuilder* parent) : details::NestedBuilder<LaneBuilder>(parent) {}

  /// @brief Set the left maliput_sparse::geometry::LineString of the LaneGeometry.
  /// @param left_line_string The left maliput_sparse::geometry::LineString to set in the LaneGeometry.
  /// @return A reference to this LaneGeometryBuilder.
  LaneGeometryBuilder& LeftLineString(const maliput_sparse::geometry::LineString3d& left_line_string);

  /// @brief Set the right maliput_sparse::geometry::LineString of the LaneGeometry.
  /// @param right_line_string The right maliput_sparse::geometry::LineString to set in the LaneGeometry.
  /// @return A reference to this LaneGeometryBuilder.
  LaneGeometryBuilder& RightLineString(const maliput_sparse::geometry::LineString3d& right_line_string);

  /// @brief Set the center maliput_sparse::geometry::LineString of the LaneGeometry.
  /// @param center_line_string The center maliput_sparse::geometry::LineString to set in the LaneGeometry.
  /// @return A reference to this LaneGeometryBuilder.
  LaneGeometryBuilder& CenterLineString(const maliput_sparse::geometry::LineString3d& center_line_string);

  /// @brief Finalizes the construction of the LaneGeometry and sets it to the parent LaneBuilder.
  /// @pre Left and right LineStrings must be set before calling this method.
  /// @throws maliput::common::assertion_error When the left and right LineStrings were not set.
  /// @return The reference to the parent LaneBuilder.
  LaneBuilder& EndLaneGeometry();

 private:
  std::optional<maliput_sparse::geometry::LineString3d> center_line_string_{};
  std::optional<maliput_sparse::geometry::LineString3d> left_line_string_{};
  std::optional<maliput_sparse::geometry::LineString3d> right_line_string_{};
};

/// @brief Builder class for maliput::api::Lanes.
class LaneBuilder final : public details::NestedBuilder<SegmentBuilder> {
 public:
  /// @brief Construct a new Lane Builder object.
  /// @param parent The parent SegmentBuilder. It must not be nullptr.
  explicit LaneBuilder(SegmentBuilder* parent) : details::NestedBuilder<SegmentBuilder>(parent) {}

  /// @brief Sets the maliput::api::LaneId of the maliput::api::Lane.
  /// @param lane_id The maliput::api::LaneId.
  /// @return A reference to this LaneBuilder.
  LaneBuilder& Id(const maliput::api::LaneId& lane_id);

  /// @brief Sets the maliput::api::maliput::api::HBounds of the maliput::api::Lane.
  /// @param hbounds A maliput::api::HBounds to set to the Lane.
  /// @return A reference to this LaneBuilder.
  LaneBuilder& HeightBounds(const maliput::api::HBounds& hbounds);

  /// @brief Starts the LaneGeometry builder for this Lane.
  /// @return A LaneGeometryBuilder.
  LaneGeometryBuilder StartLaneGeometry();

  /// @brief Finalizes the construction process of this Lane by inserting the Lane into the
  /// parent SegmentBuilder.
  /// @throws maliput::common::assertion_error When there is no LaneGeometry to be set into the Lane.
  /// @return A reference to the SegmentBuilder.
  SegmentBuilder& EndLane();

  /// @brief Sets a maliput_sparse::geometry::LaneGeometry into this builder to fill in the Lane.
  /// @details This method is only intended to be called by LaneGeometryBuilder instances.
  /// @see maliput::common::Passkey class description for further details.
  /// @param lane_geometry A maliput_sparse::geometry::LaneGeometry to be stored into the Lane. It must not be nullptr.
  /// @throws maliput::common::assertion_error When @p lane_geometry is nullptr.
  void SetLaneGeometry(maliput::common::Passkey<LaneGeometryBuilder>,
                       std::unique_ptr<maliput_sparse::geometry::LaneGeometry> lane_geometry);

 private:
  maliput::api::LaneId id_{"unset_id"};
  maliput::api::HBounds hbounds_{0., 5.};
  std::unique_ptr<maliput_sparse::geometry::LaneGeometry> lane_geometry_{};
};

/// @brief Builder class for maliput::api::Segments.
class SegmentBuilder final : public details::NestedBuilder<JunctionBuilder> {
 public:
  /// @brief Construct a new Segment Builder object.
  /// @param parent The parent JunctionBuilder. It must not be nullptr.
  explicit SegmentBuilder(JunctionBuilder* parent) : details::NestedBuilder<JunctionBuilder>(parent) {}

  /// @brief Sets the maliput::api::SegmentId of the maliput::api::Segment.
  /// @param segment_id The maliput::api::SegmentId.
  /// @return A reference to this SegmentBuilder.
  SegmentBuilder& Id(const maliput::api::SegmentId& segment_id);

  /// @brief Starts the Lane builder for this Segment.
  /// @details Lanes should be added from right to left as the order they are set will determine the
  /// index they will receive in consecutive natural numbers (0 to +inf). That order must follow
  /// the indexing order required by the maliput::api::Segment::index() contract. This builder makes
  /// no assertion of that.
  /// @return A LaneBuilder.
  LaneBuilder StartLane();

  /// @brief Finalizes the construction process of this Segment by inserting the Segment into the
  /// parent JunctionBuilder.
  /// @throws maliput::common::assertion_error When there is no lane to be set into the Segment.
  /// @return A reference to the JunctionBuilder.
  JunctionBuilder& EndSegment();

  /// @brief Sets a maliput::geometry_base::Lane into this builder to fill in the Segment.
  /// @details This method is only intended to be called by LaneBuilder instances. Call this method
  /// in order to determine a specific rigth to left ordering for this Segment.
  /// @see maliput::common::Passkey class description for further details.
  /// @param lane A lane to be stored into the Segment. It must not be nullptr.
  /// @throws maliput::common::assertion_error When @p lane is nullptr.
  void SetLane(maliput::common::Passkey<LaneBuilder>, std::unique_ptr<maliput::geometry_base::Lane> lane);

 private:
  maliput::api::SegmentId id_{"unset_id"};
  std::vector<std::unique_ptr<maliput::geometry_base::Lane>> lanes_{};
};

/// @brief Builder class for maliput::api::Junctions.
class JunctionBuilder final : public details::NestedBuilder<RoadGeometryBuilder> {
 public:
  /// @brief Construct a new Junction Builder object.
  /// @param parent The parent RoadGeometryBuilder. It must not be nullptr.
  explicit JunctionBuilder(RoadGeometryBuilder* parent) : details::NestedBuilder<RoadGeometryBuilder>(parent) {}

  /// @brief Sets the maliput::api::JunctionId of the maliput::api::Junction.
  /// @param junction_id The maliput::api::JunctionId.
  /// @return A reference to this JunctionBuilder.
  JunctionBuilder& Id(const maliput::api::JunctionId& junction_id);

  /// @brief Starts the Segment builder for this Junction.
  /// @return A JunctionBuilder.
  SegmentBuilder StartSegment();

  /// @brief Finalizes the construction process of this Junction by inserting the Junction into the
  /// parent RoadGeometryBuilder.
  /// @throws maliput::common::assertion_error When there is no segment to be set into the Junction.
  /// @return A reference to the RoadGeometryBuilder.
  RoadGeometryBuilder& EndJunction();

  /// @brief Sets a maliput::geometry_base::Segment into this builder to fill in the Junction.
  /// @details This method is only intended to be called by SegmentBuilder instances.
  /// @see maliput::common::Passkey class description for further details.
  /// @param segment A segment to be stored into the Junction. It must not be nullptr.
  /// @throws maliput::common::assertion_error When @p segment is nullptr.
  void SetSegment(maliput::common::Passkey<SegmentBuilder>, std::unique_ptr<maliput::geometry_base::Segment> segment);

 private:
  maliput::api::JunctionId id_{"unset_id"};
  std::vector<std::unique_ptr<maliput::geometry_base::Segment>> segments_{};
};

/// maliput::api::LaneEnd is not convenient in the building stage because there is no
/// valid Lane pointer yet. This struct aims for the same functionality but at the building
/// phase.
/// Note the non-default constructor definitions, those are required by maliput::api::LaneId
/// whose default constructor / empty string is not allowed.
struct LaneEnd {
  /// Constructs a LaneEnd from a @p lane_id_in and @p end_in.
  /// @param lane_id_in The LaneId that this LaneEnd refers to.
  /// @param end_in The end this LaneEnd refers to.
  LaneEnd(const maliput::api::LaneId& lane_id_in, const maliput::api::LaneEnd::Which end_in)
      : lane_id(lane_id_in), end(end_in) {}

  /// Copy constructor.
  LaneEnd(const LaneEnd& lane_end) : lane_id(lane_end.lane_id), end(lane_end.end) {}

  /// Move constructor.
  LaneEnd(LaneEnd&& lane_end) : lane_id(lane_end.lane_id), end(lane_end.end) {}

  /// Assingment operator.
  LaneEnd& operator=(LaneEnd other) {
    std::swap(lane_id, other.lane_id);
    std::swap(end, other.end);
    return *this;
  }

  /// Returns true when this LaneEnd is equal to @p other.
  bool operator==(const LaneEnd& other) const { return lane_id == other.lane_id && end == other.end; }

  /// Returns true when this LaneEnd is different from @p other.
  bool operator!=(const LaneEnd& other) const { return !(*this == other); }

  /// Returns true when this LaneEnd is less from @p other.
  /// First, LaneIds are compared by their string values and then the end are used.
  /// ends are compared by their int-value representation.
  bool operator<(const LaneEnd& other) const {
    if (lane_id.string() < other.lane_id.string()) {
      return true;
    } else if (lane_id.string() == other.lane_id.string()) {
      return static_cast<int>(end) < static_cast<int>(other.end);
    }
    return false;
  }

  maliput::api::LaneId lane_id;
  maliput::api::LaneEnd::Which end;
};

class BranchPointBuilder final : details::NestedBuilder<RoadGeometryBuilder> {
 public:
  // Convenient alias.
  using LaneEndsMultimap = std::multimap<LaneEnd, LaneEnd>;

  /// @brief Construct a new BranchPoint Builder object.
  /// @param parent The parent RoadGeometryBuilder. It must not be nullptr.
  explicit BranchPointBuilder(RoadGeometryBuilder* parent) : details::NestedBuilder<RoadGeometryBuilder>(parent) {}

  /// @brief Creates a connection between @p lane_id_a at @p which_a end with @p lane_id_b at @p which_b end.
  /// @param lane_id_a The maliput::api::LaneId of Lane A.
  /// @param which_a The maliput::api::LaneEnd::Which end of Lane A.
  /// @param lane_id_b The maliput::api::LaneId of Lane B.
  /// @param which_b The maliput::api::LaneEnd::Which end of Lane B.
  /// @throws maliput::common::assertion_error When @p lane_id_a or @p lane_id_b do not exist.
  /// @return A reference to this builder.
  BranchPointBuilder& Connect(const maliput::api::LaneId& lane_id_a, const maliput::api::LaneEnd::Which which_a,
                              const maliput::api::LaneId& lane_id_b, const maliput::api::LaneEnd::Which which_b);

  /// @brief Finalizes the construction process of all the BranchPoints.
  /// @throws maliput::common::assertion_error When there are no BranchPoints to be created.
  /// @return A reference to the RoadGeometryBuilder.
  RoadGeometryBuilder& EndBranchPoints();

 private:
  LaneEndsMultimap lane_ends_{};
};

/// @brief Builder class for maliput::api::RoadGeometry.
class RoadGeometryBuilder final {
 public:
  /// @brief Construct a new RoadGeometry Builder object.
  RoadGeometryBuilder() = default;

  /// @brief Sets the maliput::api::RoadGeometryId of the maliput::api::RoadGeometry.
  /// @param road_geometry_id The maliput::api::RoadGeometryId.
  /// @return A reference to this RoadGeometryBuilder.
  RoadGeometryBuilder& Id(const maliput::api::RoadGeometryId& road_geometry_id);

  /// @brief Sets the linear tolerance of the maliput::api::RoadGeometry.
  /// @param linear_tolerance The linear tolerance of the maliput::api::RoadGeometry. It must be positive.
  /// @return A reference to this RoadGeometryBuilder.
  RoadGeometryBuilder& LinearTolerance(double linear_tolerance);

  /// @brief Sets the angular tolerance of the maliput::api::RoadGeometry.
  /// @param angular_tolerance The angular tolerance of the maliput::api::RoadGeometry. It must be positive.
  /// @return A reference to this RoadGeometryBuilder.
  RoadGeometryBuilder& AngularTolerance(double angular_tolerance);

  /// @brief Sets the scale length of the maliput::api::RoadGeometry.
  /// @param scale_length The scale length of the maliput::api::RoadGeometry. It must be positive.
  /// @return A reference to this RoadGeometryBuilder.
  RoadGeometryBuilder& ScaleLength(double scale_length);

  /// @brief Sets the initial to backend frame translation vector of the maliput::api::RoadGeometry.
  /// @param translation The initial to backend frame translation vector of the maliput::api::RoadGeometry.
  /// @return A reference to this RoadGeometryBuilder.
  RoadGeometryBuilder& InertialToBackendFrameTranslation(const maliput::math::Vector3& translation);

  /// @brief Starts the Junction builder for this RoadGeometry.
  /// @return A JunctionBuilder.
  JunctionBuilder StartJunction();

  /// @brief Starts the BranchPoint builder for this RoadGeometry.
  /// @return A BranchPointBuilder.
  BranchPointBuilder StartBranchPoints();

  /// @brief Builds a maliput::api::RoadGeometry.
  /// @details The underlying type of the RoadGeometry is maliput_sparse::RoadGeometry which is derived from
  /// maliput::geometry_base::RoadGeometry.
  /// @throws maliput::common::assertion_error When there is no Junction to add to the RoadGeometry.
  /// @throws maliput::common::assertion_error When there is no BranchPoint to add to the RoadGeometry.
  /// @return A std::unique_ptr<maliput::api::RoadGeometry>.
  std::unique_ptr<maliput::api::RoadGeometry> Build();

  /// @brief Sets a maliput::geometry_base::Junction into this builder to fill in the RoadGeometry.
  /// @details This method is only intended to be called by JunctionBuilder instances.
  /// @see maliput::common::Passkey class description for further details.
  /// @param junction A junction to be stored into the RoadGeometry. It must not be nullptr.
  /// @throws maliput::common::assertion_error When @p junction is nullptr.
  void SetJunction(maliput::common::Passkey<JunctionBuilder>,
                   std::unique_ptr<maliput::geometry_base::Junction> junction);

  void SetBranchPoints(maliput::common::Passkey<BranchPointBuilder>,
                       std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>>&& branch_points);

  std::unordered_map<maliput::api::LaneId, const maliput::geometry_base::Lane*> GetLanes(
      maliput::common::Passkey<BranchPointBuilder>) const;

  /// @brief Getter for LaneGeometry of linear_tolerance.
  /// @see maliput::common::Passkey class description for further details.
  /// @return The linear_tolerance.
  double linear_tolerance(maliput::common::Passkey<LaneGeometryBuilder>) const { return linear_tolerance_; }

  /// @brief Getter for LaneGeometry of scale_length.
  /// @see maliput::common::Passkey class description for further details.
  /// @return The scale_length.
  double scale_length(maliput::common::Passkey<LaneGeometryBuilder>) const { return scale_length_; }

 private:
  maliput::api::RoadGeometryId id_{"unset_id"};
  double linear_tolerance_{1e-6};
  double angular_tolerance_{1e-6};
  double scale_length_{1.};
  maliput::math::Vector3 inertial_to_backend_frame_translation_{0., 0., 0.};
  std::vector<std::unique_ptr<maliput::geometry_base::Junction>> junctions_{};
  std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>> branch_points_{};
};

}  // namespace builder
}  // namespace maliput_sparse
