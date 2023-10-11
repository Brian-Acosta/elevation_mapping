#include <iostream>
#include "drake/math/rigid_transform.h"
#include "grid_map_core/grid_map_core.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace elevation_mapping {

int do_main() {
  drake::math::RigidTransformd transform = drake::math::RigidTransformd::Identity();
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  std::cout << "Success" << std::endl;
  return 0;
}

}

int main(int argc, char** argv) {
  return elevation_mapping::do_main();
}