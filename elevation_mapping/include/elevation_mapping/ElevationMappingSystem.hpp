#pragma once

// Elevation mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"

// Drake
#include "drake/systems/framework/leaf_system.h"

namespace elevation_mapping {

class ElevationMappingSystem : public drake::systems::LeafSystem<double> {
 public:
  ElevationMappingSystem();

 private:

  drake::systems::EventStatus PeriodicUnrestrictedUpdateEvent(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state);

  drake::systems::InputPortIndex point_cloud_input_port_;
  drake::systems::InputPortIndex robot_state_input_port_;
  drake::systems::InputPortIndex robot_pose_covariance_input_port_;
  
};

}