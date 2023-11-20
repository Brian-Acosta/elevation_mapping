/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

// PCL
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>


// STL
#include <cmath>
#include <limits>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

// drake
#include "drake/common/text_logging.h"

namespace elevation_mapping {

using drake::math::RigidTransformd;

SensorProcessorBase::SensorProcessorBase(const GeneralParameters& generalConfig)
    : firstTfAvailable_(false),
    transformationSensorToMap_(Eigen::Affine3d::Identity()) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  generalParameters_ = generalConfig;

  drake::log()->trace(
      "Sensor processor general parameters are:"
      "\n\t- robot_base_frame_id: {}"
      "\n\t- map_frame_id: {}",
      generalConfig.robotBaseFrameId_, generalConfig.mapFrameId_
  );
}

SensorProcessorBase::~SensorProcessorBase() = default;

bool SensorProcessorBase::readParameters(const std::string& yaml_filename) {
  drake::log()->trace("[Elevation mapping sensor_processor_base] "
                      "TODO: implement parameter reading from yaml");
  return true;
}

bool SensorProcessorBase::process(const PointCloudType::ConstPtr pointCloudInput,
                                  const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                  PointCloudType::Ptr pointCloudMapFrame,
                                  Eigen::VectorXf& variances,
                                  const RigidTransformd& sensorPoseInBaseFrame,
                                  const RigidTransformd& robotBaseFramePoseInMapFrame) {

  const Parameters parameters{parameters_.getData()};

  updateTransformations(sensorPoseInBaseFrame, robotBaseFramePoseInMapFrame);

  // For now, assume the Point Cloud is already in the sensor frame
  PointCloudType::Ptr pointCloudSensorFrame(new PointCloudType);
  pcl::copyPointCloud(*pointCloudInput, *pointCloudSensorFrame);
  //  transformPointCloud(pointCloudInput, pointCloudSensorFrame, sensorFrameId_);

  // Remove Nans (optional voxel grid filter)
  filterPointCloud(pointCloudSensorFrame);

  // Specific filtering per sensor type
  filterPointCloudSensorType(pointCloudSensorFrame);

  // Remove outside limits in map frame
  if (!transformPointCloud(
      pointCloudSensorFrame,
      pointCloudMapFrame,
      robotBaseFramePoseInMapFrame * sensorPoseInBaseFrame
      )) {
    return false;
  }
  std::vector<PointCloudType::Ptr> pointClouds({pointCloudMapFrame, pointCloudSensorFrame});
  removePointsOutsideLimits(pointCloudMapFrame, pointClouds);

  // Compute variances
  return computeVariances(pointCloudSensorFrame, robotPoseCovariance, variances);
}

bool SensorProcessorBase::updateTransformations(
    const RigidTransformd& sensorPoseInBaseFrame,
    const RigidTransformd& robotBaseFramePoseInMapFrame) {
  rotationBaseToSensor_ = sensorPoseInBaseFrame.rotation().inverse();
  translationBaseToSensorInBaseFrame_ = sensorPoseInBaseFrame.translation();
  rotationMapToBase_ = robotBaseFramePoseInMapFrame.rotation().inverse();
  translationMapToBaseInMapFrame_ = robotBaseFramePoseInMapFrame.translation();
  // X_MS = X_MB * X_BS
  transformationSensorToMap_.linear() = (robotBaseFramePoseInMapFrame * sensorPoseInBaseFrame).rotation().matrix();
  transformationSensorToMap_.translation() = (robotBaseFramePoseInMapFrame * sensorPoseInBaseFrame).translation();
  return true;
}

bool SensorProcessorBase::transformPointCloud(PointCloudType::ConstPtr pointCloud,
                                              PointCloudType::Ptr pointCloudTransformed,
                                              const RigidTransformd& transformPointCloudToTargetFrame) {

  Eigen::Affine3d transform = transformPointCloudToTargetFrame.GetAsIsometry3();
  pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());

  return true;
}

void SensorProcessorBase::removePointsOutsideLimits(PointCloudType::ConstPtr reference, std::vector<PointCloudType::Ptr>& pointClouds) {
  const Parameters parameters{parameters_.getData()};
  if (!std::isfinite(parameters.ignorePointsLowerThreshold_) && !std::isfinite(parameters.ignorePointsUpperThreshold_)) {
    return;
  }
  drake::log()->debug(
      "Limiting point cloud to the height interval of [{}, {}] "
      "relative to the robot base.",
      parameters.ignorePointsLowerThreshold_,
      parameters.ignorePointsUpperThreshold_
  );

  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z");  // TODO(max): Should this be configurable?
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + parameters.ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + parameters.ignorePointsUpperThreshold_;
  passThroughFilter.setFilterLimits(relativeLowerThreshold, relativeUpperThreshold);
  pcl::IndicesPtr insideIndices(new pcl::Indices);
  passThroughFilter.filter(*insideIndices);

  for (auto& pointCloud : pointClouds) {
    pcl::ExtractIndices<pcl::PointXYZRGBConfidenceRatio> extractIndicesFilter;
    extractIndicesFilter.setInputCloud(pointCloud);
    extractIndicesFilter.setIndices(insideIndices);
    PointCloudType tempPointCloud;
    extractIndicesFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }

  drake::log()->debug(
      "removePointsOutsideLimits() reduced point cloud to {} points.",
      static_cast<int>(pointClouds[0]->size())
  );
}

bool SensorProcessorBase::filterPointCloud(const PointCloudType::Ptr pointCloud) {
  const Parameters parameters{parameters_.getData()};
  PointCloudType tempPointCloud;

  // Remove nan points.
  pcl::Indices indices;
  if (!pointCloud->is_dense) {
    pcl::removeNaNFromPointCloud(*pointCloud, tempPointCloud, indices);
    tempPointCloud.is_dense = true;
    pointCloud->swap(tempPointCloud);
  }

  // Reduce points using VoxelGrid filter.
  if (parameters.applyVoxelGridFilter_) {
    pcl::VoxelGrid<pcl::PointXYZRGBConfidenceRatio> voxelGridFilter;
    voxelGridFilter.setInputCloud(pointCloud);
    double filter_size = parameters.sensorParameters_.at("voxelgrid_filter_size");
    voxelGridFilter.setLeafSize(filter_size, filter_size, filter_size);
    voxelGridFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }
  drake::log()->debug(
      "cleanPointCloud() reduced point cloud to {} points.",
      static_cast<int>(pointCloud->size())
  );
  return true;
}

bool SensorProcessorBase::filterPointCloudSensorType(const PointCloudType::Ptr /*pointCloud*/) {
  return true;
}

} /* namespace elevation_mapping */
