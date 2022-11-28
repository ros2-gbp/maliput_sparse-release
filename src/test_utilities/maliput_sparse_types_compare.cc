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
#include "maliput_sparse/test_utilties/maliput_sparse_types_compare.h"

#include <maliput/test_utilities/maliput_math_compare.h>

namespace maliput_sparse {
namespace test {

::testing::AssertionResult CompareLineString3d(const maliput_sparse::geometry::LineString3d& lhs,
                                               const maliput_sparse::geometry::LineString3d& rhs, double tolerance) {
  if (lhs.size() != rhs.size()) {
    return ::testing::AssertionFailure() << "LineString3d size mismatch: " << lhs.size() << " != " << rhs.size();
  }
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (!maliput::math::test::CompareVectors(lhs[i], rhs[i], tolerance)) {
      return ::testing::AssertionFailure()
             << "LineString3d point mismatch at index " << i << ": " << lhs[i] << " != " << rhs[i];
    }
  }
  return ::testing::AssertionSuccess();
}

}  // namespace test
}  // namespace maliput_sparse
