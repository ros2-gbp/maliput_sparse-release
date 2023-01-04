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

#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "maliput_sparse/parser/parser.h"

namespace maliput_sparse {
namespace parser {

/// ValidatorConfig struct that contains the configuration for the Validator.
struct ValidatorConfig {
  double linear_tolerance{1e-12};
};

/// After parsing a road network, the Validator can be used to check for errors before
/// creating a maliput::api::RoadNetwork via the maliput_sparse::loader::RoadNetworkLoader.
/// The Validator can be configured to check for different types of errors and provide an interface to retrieve
/// the errors found.
/// The errors are stored in a vector of Error structs. The Error struct contains a message, the type of error, and the
/// severity. It's on the user to decide how to handle the errors.
class Validator {
 public:
  /// The type of validation.
  enum class Type {
    kLogicalLaneAdjacency,
    kGeometricalLaneAdjacency,
  };

  /// Error struct that contains the error message, type, and severity.
  struct Error {
    /// The severity of the error.
    enum class Severity {
      kWarning,
      kError,
    };

    /// Equality operator for Error.
    bool operator==(const Error& other) const;
    /// Inequality operator for Error.
    bool operator!=(const Error& other) const;

    /// Message describing the error.
    std::string message;
    /// The type of error.
    Validator::Type type;
    /// The severity of the error.
    Severity severity;
  };

  using Types = std::unordered_set<Validator::Type>;

  /// Constructor for Validator.
  /// During construction, the Validator will perform the validation checks.
  ////
  /// @param parser The maliput_sparse::parser::Parser instance to validate.
  /// @param types The types of validation to perform.
  /// @param config The maliput_sparse::parser::ValidatorConfig to use.
  Validator(const Parser* parser, const Types& types, const ValidatorConfig& config);

  /// Returns the errors found during validation.
  std::vector<Error> operator()() const;

 private:
  // Returns the types of validation that are dependent on the given type.
  static const std::unordered_map<Type, std::unordered_set<Type>> kDependentTypes;

  // The maliput_sparse::parser::Parser instance to validate.
  const Parser* parser_{nullptr};
  // The maliput_sparse::parser::ValidatorConfig to use.
  const ValidatorConfig config_;
  // The types of validation to perform.
  Types types_;
};

/// Serialize Validator::Type to ostream.
std::ostream& operator<<(std::ostream& os, const Validator::Type& type);

}  // namespace parser
}  // namespace maliput_sparse
