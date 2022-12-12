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
#pragma once

#include <maliput/api/road_geometry.h>
#include <maliput/math/vector.h>

namespace maliput_sparse {
namespace loader {
namespace config {

/// @defgroup builder_configuration_keys RoadNetwork configuration builder keys
///
/// Parameters used during the RoadNetwork building process.
///
/// When parameters are omitted the default value will be used.
///
/// @{

/// A string that works as ID of the RoadGeometry.
///   - Default: @e "maliput"
static constexpr char const* kRoadGeometryId{"road_geometry_id"};

/// RoadGeometry's linear tolerance.
///   - Default: @e "5e-2"
static constexpr char const* kLinearTolerance{"linear_tolerance"};

/// RoadGeometry's angular tolerance.
///   - Default: @e "1e-3"
static constexpr char const* kAngularTolerance{"angular_tolerance"};

/// RoadGeometry's scale length.
///   - Default: @e "1.0"
static constexpr char const* kScaleLength{"scale_length"};

/// Translation from maliput to maliput_osm inertial frame.
/// The format of the 3-dimensional vector that is expected to be passed
/// should be {X, Y, Z}. Same format as maliput::math::Vector3 is
/// serialized.
///   - Default: @e "{0., 0., 0.}"
static constexpr char const* kInertialToBackendFrameTranslation{"inertial_to_backend_frame_translation"};

/// Path to the configuration file to load a RoadRulebook
///   - Default: ""
static constexpr char const* kRoadRuleBook{"road_rule_book"};

/// Path to the configuration file to load a RoadRulebook
///   - Default: ""
static constexpr char const* kRuleRegistry{"rule_registry"};

/// Path to the configuration file to load a TrafficLightBook
///   - Default: ""
static constexpr char const* kTrafficLightBook{"traffic_light_book"};

/// Path to the configuration file to load a PhaseRingBook
///   - Default: ""
static constexpr char const* kPhaseRingBook{"phase_ring_book"};

/// Path to the configuration file to load a IntersectionBook
///   - Default: ""
static constexpr char const* kIntersectionBook{"intersection_book"};

/// @}

}  // namespace config
}  // namespace loader
}  // namespace maliput_sparse
