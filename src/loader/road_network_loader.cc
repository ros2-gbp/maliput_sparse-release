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
#include "maliput_sparse/loader/road_network_loader.h"

#include <memory>

#include <maliput/base/intersection_book.h>
#include <maliput/base/intersection_book_loader.h>
#include <maliput/base/manual_discrete_value_rule_state_provider.h>
#include <maliput/base/manual_phase_ring_book.h>
#include <maliput/base/manual_range_value_rule_state_provider.h>
#include <maliput/base/manual_rulebook.h>
#include <maliput/base/phase_based_right_of_way_rule_state_provider.h>
#include <maliput/base/phase_ring_book_loader.h>
#include <maliput/base/phased_discrete_rule_state_provider.h>
#include <maliput/base/road_rulebook_loader.h>
#include <maliput/base/rule_registry.h>
#include <maliput/base/rule_registry_loader.h>
#include <maliput/base/traffic_light_book.h>
#include <maliput/base/traffic_light_book_loader.h>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_unused.h>

namespace maliput_sparse {
namespace loader {

RoadNetworkLoader::RoadNetworkLoader(std::unique_ptr<parser::Parser> parser,
                                     const BuilderConfiguration& builder_configuration)
    : road_geometry_loader_(std::make_unique<RoadGeometryLoader>(std::move(parser), builder_configuration)),
      builder_configuration_(builder_configuration) {
  MALIPUT_THROW_UNLESS(road_geometry_loader_ != nullptr);
}

std::unique_ptr<maliput::api::RoadNetwork> RoadNetworkLoader::operator()() {
  maliput::log()->trace("Loading RoadGeometry...");
  std::unique_ptr<const maliput::api::RoadGeometry> rg = road_geometry_loader_->operator()();

  maliput::log()->trace("Building TrafficLightBook...");
  maliput::log()->trace("{}", builder_configuration_.traffic_light_book.has_value()
                                  ? "TrafficLight file provided: " + builder_configuration_.traffic_light_book.value()
                                  : "No TrafficLight file provided");
  auto traffic_light_book =
      !builder_configuration_.traffic_light_book.has_value()
          ? std::make_unique<maliput::TrafficLightBook>()
          : maliput::LoadTrafficLightBookFromFile(builder_configuration_.traffic_light_book.value());

  maliput::log()->trace("Building RuleRegistry...");

  maliput::log()->trace("{}", builder_configuration_.rule_registry.has_value()
                                  ? "RuleRegistry file provided: " + builder_configuration_.rule_registry.value()
                                  : "No RuleRegistry file provided");
  auto rule_registry = !builder_configuration_.rule_registry.has_value()
                           ? std::make_unique<maliput::api::rules::RuleRegistry>()
                           : maliput::LoadRuleRegistryFromFile(builder_configuration_.rule_registry.value());

  maliput::log()->trace("Built RuleRegistry...");

  maliput::log()->trace("Building RuleRoadBook...");

  maliput::log()->trace("{}", builder_configuration_.road_rule_book.has_value()
                                  ? "RoadRulebook file provided: " + builder_configuration_.road_rule_book.value()
                                  : "No RoadRulebook file provided");

  auto rule_book =
      builder_configuration_.road_rule_book.has_value()
          ? maliput::LoadRoadRulebookFromFile(rg.get(), builder_configuration_.road_rule_book.value(), *rule_registry)
          : std::make_unique<maliput::ManualRulebook>();

  maliput::log()->trace("Built RuleRoadBook.");

  maliput::log()->trace("Building PhaseRingBook...");

  maliput::log()->trace("{}", builder_configuration_.phase_ring_book.has_value()
                                  ? "PhaseRingBook file provided: " + builder_configuration_.phase_ring_book.value()
                                  : "No PhaseRingBook file provided");

  auto phase_ring_book = builder_configuration_.phase_ring_book.has_value()
                             ? maliput::LoadPhaseRingBookFromFile(rule_book.get(), traffic_light_book.get(),
                                                                  builder_configuration_.phase_ring_book.value())
                             : std::make_unique<maliput::ManualPhaseRingBook>();

  maliput::log()->trace("Built PhaseRingBook.");

  maliput::log()->trace("Building PhaseProvider...");
  auto phase_provider = maliput::ManualPhaseProvider::GetDefaultPopulatedManualPhaseProvider(phase_ring_book.get());
  maliput::log()->trace("Built PhaseProvider.");

  maliput::log()->trace("Building DiscreteValueRuleStateProvider...");
  auto discrete_value_rule_state_provider =
      maliput::PhasedDiscreteRuleStateProvider::GetDefaultPhasedDiscreteRuleStateProvider(
          rule_book.get(), phase_ring_book.get(), phase_provider.get());
  maliput::log()->trace("Built DiscreteValueRuleStateProvider.");

  maliput::log()->trace("Building RangeValueRuleStateProvider...");
  auto range_value_rule_state_provider =
      maliput::ManualRangeValueRuleStateProvider::GetDefaultManualRangeValueRuleStateProvider(rule_book.get());
  maliput::log()->trace("Built RangeValueRuleStateProvider.");

  maliput::log()->trace("Building IntersectionBook...");

  maliput::log()->trace("{}",
                        builder_configuration_.intersection_book.has_value()
                            ? "IntersectionBook file provided: " + builder_configuration_.intersection_book.value()
                            : "No IntersectionBook file provided");

  auto intersection_book =
      !builder_configuration_.intersection_book.has_value()
          ? std::make_unique<maliput::IntersectionBook>(rg.get())
          : maliput::LoadIntersectionBookFromFile(builder_configuration_.intersection_book.value(), *rule_book,
                                                  *phase_ring_book, rg.get(), phase_provider.get());
  maliput::log()->trace("Built IntersectionBook.");

  maliput::log()->trace("Building RuleStateProvider...");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // We are not supporting old rule api therefore maliput::api::RightOfWayRuleStateProvider class isn't fully set up.
  auto state_provider =
      std::make_unique<maliput::PhaseBasedRightOfWayRuleStateProvider>(phase_ring_book.get(), phase_provider.get());
#pragma GCC diagnostic pop
  maliput::log()->trace("Built RuleStateProvider.");

  return std::make_unique<maliput::api::RoadNetwork>(
      std::move(rg), std::move(rule_book), std::move(traffic_light_book), std::move(intersection_book),
      std::move(phase_ring_book), std::move(state_provider), std::move(phase_provider), std::move(rule_registry),
      std::move(discrete_value_rule_state_provider), std::move(range_value_rule_state_provider));
}

}  // namespace loader
}  // namespace maliput_sparse
