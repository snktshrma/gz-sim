/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_SIM_SYSTEMS_SETMODELSTATE_HH_
#define GZ_SIM_SYSTEMS_SETMODELSTATE_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class SetModelStatePrivate;

  /// \brief This system sets a specified model state during Configure and
  /// Reset.
  ///
  /// ## Components
  ///
  /// This system uses the following components:
  ///
  /// - gz::sim::components::JointPositionReset
  /// - gz::sim::components::JointVelocityReset
  /// TODO: add components to reset link pose and velocity
  class SetModelState
      : public System,
        public ISystemConfigure,
        public ISystemReset
  {
    /// \brief Constructor
    public: SetModelState();

    /// \brief Destructor
    public: ~SetModelState() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void Reset(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<SetModelStatePrivate> dataPtr;
  };
  }
}
}
}

#endif