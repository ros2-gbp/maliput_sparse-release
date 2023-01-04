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

#include <maliput/common/logger.h>

#include "maliput_sparse/geometry/utility/geometry.h"
#include "maliput_sparse/parser/junction.h"
#include "maliput_sparse/parser/lane.h"
#include "maliput_sparse/parser/segment.h"
#include "parser/validation_methods.h"

namespace maliput_sparse {
namespace parser {

namespace {

// Convenient constant for indicating left or right.
static constexpr bool kLeft{true};
static constexpr bool kRight{!kLeft};

// Helper function to geometrically check lane adjacency.
// It relies on geometry::utility::ComputeDistance to compute the distance between the two adjacent line strings.
// Returns true if the lanes are adjacent under certain tolerance, false otherwise.

bool AreAdjacent(const Lane& lane, const Lane& adjacent_lane, bool left, double tolerance) {
  const auto line_string_a = left ? lane.left : lane.right;
  const auto line_string_b = left ? adjacent_lane.right : adjacent_lane.left;
  const double distance = geometry::utility::ComputeDistance(line_string_a, line_string_b, tolerance);
  return distance <= tolerance;
}

}  // namespace

const std::unordered_map<Validator::Type, std::unordered_set<Validator::Type>> Validator::kDependentTypes{
    {Validator::Type::kLogicalLaneAdjacency, {}},
    {Validator::Type::kGeometricalLaneAdjacency, {Validator::Type::kLogicalLaneAdjacency}},
};

Validator::Validator(const Parser* parser, const Types& types, const ValidatorConfig& config)
    : parser_{parser}, config_{config}, types_{types} {
  MALIPUT_THROW_UNLESS(parser_);

  // Evaluate dependencies of the validation types.
  for (const auto& type : types_) {
    const auto& dependencies = kDependentTypes.at(type);
    for (const auto& dependency : dependencies) {
      if (std::find(types_.begin(), types_.end(), dependency) == types_.end()) {
        maliput::log()->debug("Validator: {} depends on {}, adding it to the validation types.", type, dependency);
        types_.insert(dependency);
      }
    }
  }
}

std::vector<Validator::Error> Validator::operator()() const {
  std::vector<Validator::Error> errors;

  // Lane adjacency validation.
  const bool logical_lane_adjacency_enabled =
      std::find(types_.begin(), types_.end(), Validator::Type::kLogicalLaneAdjacency) != types_.end();
  const bool geometrical_lane_adjacency_enabled =
      std::find(types_.begin(), types_.end(), Validator::Type::kGeometricalLaneAdjacency) != types_.end();
  if (logical_lane_adjacency_enabled || geometrical_lane_adjacency_enabled) {
    const auto lane_adjacency_errors = ValidateLaneAdjacency(parser_, geometrical_lane_adjacency_enabled, config_);
    errors.insert(errors.end(), lane_adjacency_errors.begin(), lane_adjacency_errors.end());
  }

  return errors;
}

std::vector<Validator::Error> ValidateLaneAdjacency(const Parser* parser, bool validate_geometric_adjacency,
                                                    const ValidatorConfig& config) {
  std::vector<Validator::Error> errors;
  auto evaluate = [&errors](bool condition, const std::string& message, const Validator::Type& error_type) {
    if (condition) {
      errors.push_back({message, error_type, Validator::Error::Severity::kError});
    }
    return condition;
  };
  const auto junctions = parser->GetJunctions();
  for (const auto junction : junctions) {
    for (const auto segment : junction.second.segments) {
      const std::vector<Lane>& lanes = segment.second.lanes;
      // Iterates lanes from right to left.
      // | idx=N-1 | idx=3 | idx=2 | idx=1 | idx=0 |
      for (int idx{}; idx < static_cast<int>(lanes.size()); ++idx) {
        const Lane& lane = lanes[idx];

        // Note: The following code is a bit repetitive for left and right adjacency checks, but it is easier to read
        // and understand.
        //
        // Check right adjacency <--------------------------------------------------
        if (lane.right_lane_id) {
          // Check if there is a idx to the right.
          evaluate(idx == 0,
                   {"Wrong ordering of lanes: Lane " + lane.id + " has a right lane id (" + lane.right_lane_id.value() +
                    ") but is the first lane in the segment " + segment.first + "."},
                   Validator::Type::kLogicalLaneAdjacency);
          // Check if right lane id is in the same segment
          const auto adj_lane_it = std::find_if(lanes.begin(), lanes.end(), [&lane](const Lane& lane_it) {
            return lane.right_lane_id.value() == lane_it.id;
          });
          if (!evaluate(adj_lane_it == lanes.end(),
                        {"Adjacent lane isn't part of the segment: Lane " + lane.id + " has a right lane id (" +
                         lane.right_lane_id.value() + ") that is not part of the segment " + segment.first + "."},
                        Validator::Type::kLogicalLaneAdjacency)) {
            // Check if right lane id has the lane id as left lane id.
            !evaluate(!adj_lane_it->left_lane_id.has_value(),
                      {"Wrong ordering of lanes: Lane " + lane.id + " has a right lane id (" +
                       lane.right_lane_id.value() + ") that has no left lane id."},
                      Validator::Type::kLogicalLaneAdjacency) &&
                evaluate(adj_lane_it->left_lane_id.value() != lane.id,
                         {"Wrong ordering of lanes: Lane " + lane.id + " has a right lane id (" +
                          lane.right_lane_id.value() + ") that has a left lane id (" +
                          adj_lane_it->left_lane_id.value() + ") that is not the lane " + lane.id + "."},
                         Validator::Type::kLogicalLaneAdjacency);

            // Check geometrical adjacency.
            if (validate_geometric_adjacency) {
              evaluate(!AreAdjacent(lane, *adj_lane_it, kRight, config.linear_tolerance),
                       {"Lane " + lane.id + " and lane " + adj_lane_it->id + " are not adjacent under the tolerance " +
                        std::to_string(config.linear_tolerance) + "."},
                       Validator::Type::kGeometricalLaneAdjacency);
            }
          }

          // Check if idx - 1 lane is the right lane id.
          evaluate((idx - 1 >= 0) && (lanes[idx - 1].id != lane.right_lane_id.value()),
                   {"Wrong ordering of lanes: Lane " + lane.id + " has a right lane id (" + lane.right_lane_id.value() +
                    ") that is not the previous lane in the segment " + segment.first + "."},
                   Validator::Type::kLogicalLaneAdjacency);

        } else {
          // Check if idx is the first lane in the segment.
          evaluate(idx != 0,
                   {"Wrong ordering of lanes: Lane " + lane.id +
                    " has no right lane id but it isn't the first lane in the segment " + segment.first + "."},
                   Validator::Type::kLogicalLaneAdjacency);
        }

        // Check left adjacency <--------------------------------------------------
        if (lane.left_lane_id) {
          // Check if there is a idx to the left.
          evaluate(idx == static_cast<int>(lanes.size()) - 1,
                   {"Wrong ordering of lanes: Lane " + lane.id + " has a left lane id (" + lane.left_lane_id.value() +
                    ") but is the last lane in the segment " + segment.first + "."},
                   Validator::Type::kLogicalLaneAdjacency);

          // Check if left lane id is in the same segment
          const auto adj_lane_it = std::find_if(lanes.begin(), lanes.end(), [&lane](const Lane& lane_it) {
            return lane.left_lane_id.value() == lane_it.id;
          });
          if (!evaluate(adj_lane_it == lanes.end(),
                        {"Adjacent lane isn't part of the segment: Lane " + lane.id + " has a left lane id (" +
                         lane.left_lane_id.value() + ") that is not part of the segment " + segment.first + "."},
                        Validator::Type::kLogicalLaneAdjacency)) {
            // Check if left lane id has the lane id as right lane id.
            !evaluate(!adj_lane_it->right_lane_id.has_value(),
                      {"Wrong ordering of lanes: Lane " + lane.id + " has a left lane id (" +
                       lane.left_lane_id.value() + ") that has no right lane id."},
                      Validator::Type::kLogicalLaneAdjacency) &&
                evaluate(adj_lane_it->right_lane_id.value() != lane.id,
                         {"Wrong ordering of lanes: Lane " + lane.id + " has a left lane id (" +
                          lane.left_lane_id.value() + ") that has a right lane id (" +
                          adj_lane_it->right_lane_id.value() + ") that is not the lane " + lane.id + "."},
                         Validator::Type::kLogicalLaneAdjacency);

            // Check geometrical adjacency.
            if (validate_geometric_adjacency) {
              evaluate(!AreAdjacent(lane, *adj_lane_it, kLeft, config.linear_tolerance),
                       {"Lane " + lane.id + " and lane " + adj_lane_it->id + " are not adjacent under the tolerance " +
                        std::to_string(config.linear_tolerance) + "."},
                       Validator::Type::kGeometricalLaneAdjacency);
            }
          }

          // Check if idx + 1 lane is the left lane id.
          if ((idx + 1 <= static_cast<int>(lanes.size()) - 1) && (lanes[idx + 1].id != lane.left_lane_id.value())) {
            // Error.
            evaluate(true,
                     {"Wrong ordering of lanes: Lane " + lane.id + " has a left lane id (" + lane.left_lane_id.value() +
                      ") that is not the next lane in the segment " + segment.first + "."},
                     Validator::Type::kLogicalLaneAdjacency);
          }

        } else {
          // Check if idx is the last lane in the segment.
          evaluate(idx != static_cast<int>(lanes.size()) - 1,
                   {"Wrong ordering of lanes: Lane " + lane.id +
                    " has no left lane id but it isn't the last lane in the segment " + segment.first + "."},
                   Validator::Type::kLogicalLaneAdjacency);
        }
      }
    }
  }
  return errors;
}

bool Validator::Error::operator==(const Error& other) const {
  return message == other.message && type == other.type && severity == other.severity;
}

bool Validator::Error::operator!=(const Error& other) const { return !(*this == other); }

std::ostream& operator<<(std::ostream& os, const Validator::Type& type) {
  switch (type) {
    case Validator::Type::kLogicalLaneAdjacency:
      os << "Logical Lane Adjacency";
      break;
    case Validator::Type::kGeometricalLaneAdjacency:
      os << "Geometrical Lane Adjacency";
      break;
  }
  return os;
}

}  // namespace parser
}  // namespace maliput_sparse
