#include "elevation_mapping/ElevationMappingSystem.hpp"

namespace elevation_mapping {

using drake::systems::Context;
using drake::multibody::MultibodyPlant;

ElevationMappingSystem::ElevationMappingSystem(
    const MultibodyPlant<double>& plant,
    Context<double>* context,
    const std::vector<sensor_pose_params>& sensor_poses) :
    plant_(plant), context_(context) {

  for (const auto& pose_param : sensor_poses) {
    DRAKE_DEMAND(plant_.HasBodyNamed(pose_param.sensor_parent_body_));
    DRAKE_DEMAND(sensor_poses_.count(pose_param.sensor_name_) == 0);
    sensor_poses_.insert({pose_param.sensor_name_, pose_param});

    input_ports_pcl_.insert({pose_param.sensor_name_, DeclareAbstractInputPort(
          "Point_cloud_" + pose_param.sensor_name_,
          drake::Value<PointCloudType::Ptr>()
      ).get_index()
    });
  }

}

}