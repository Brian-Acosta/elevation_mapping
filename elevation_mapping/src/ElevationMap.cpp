/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <chrono>
#include <cstring>

#include <Eigen/Dense>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

#include "drake/common/text_logging.h"

namespace {
/**
 * Store an unsigned integer value in a float
 * @param input integer
 * @return A float with the bit pattern of the input integer
 */
float intAsFloat(const uint32_t input) {
  float output{std::nanf("1")};
  std::memcpy(&output, &input, sizeof(uint32_t));
  return output;
}
}  // namespace

namespace elevation_mapping {

ElevationMap::ElevationMap(const std::string& parameter_yaml)
    : rawMap_({
        "elevation",
        "variance",
        "horizontal_variance_x",
        "horizontal_variance_y",
        "horizontal_variance_xy",
        "color",
        "time",
        "dynamic_time",
        "lowest_scan_point",
        "sensor_x_at_lowest_scan",
        "sensor_y_at_lowest_scan",
        "sensor_z_at_lowest_scan"
      }),
      fusedMap_({"elevation", "upper_bound", "lower_bound", "color"}),
      hasUnderlyingMap_(false) {

  // Should be copy-constructable or Cloneable to be used as a Drake
  // AbstractState. If memory allocation proves to be a pain-point,
  // might need to rethink this strategy  but for now we will
  // just require that the default copy constructor exists.
  static_assert(std::is_copy_constructible_v<ElevationMap>);
  static_assert(std::is_copy_assignable_v<ElevationMap>);

  if (not parameter_yaml.empty()) {
    bool parameter_loading_success = loadParams(parameter_yaml);
    DRAKE_DEMAND(parameter_loading_success);
  }
  rawMap_.setBasicLayers({"elevation", "variance"});
  fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  clear();
}

ElevationMap::~ElevationMap() = default;

bool ElevationMap::loadParams(const std::string& parameter_yaml) {
  drake::log()->warn("Param yaml not yet implemented for ElevationMap");
  return true;
}

void ElevationMap::setGeometry(const grid_map::Length& length,
                               const double& resolution,
                               const grid_map::Position& position) {
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
//  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  rawMap_.setGeometry(length, resolution, position);
  fusedMap_.setGeometry(length, resolution, position);
  drake::log()->trace(
      "Elevation map grid resized to {} rows and {} columns",
      rawMap_.getSize()(0),
      rawMap_.getSize()(1)
  );
}
bool ElevationMap::add(const PointCloudType::Ptr pointCloud,
                       Eigen::VectorXf& pointCloudVariances,
                       double timestamp,
                       const drake::math::RigidTransformd& transformationSensorToMap) {
  const Parameters parameters{parameters_};
  if (static_cast<unsigned int>(pointCloud->size()) != static_cast<unsigned int>(pointCloudVariances.size())) {
    drake::log()->error(
        "ElevationMap::add: Size of point cloud {} and variances {} do not agree.",
        (int)pointCloud->size(),
        (int)pointCloudVariances.size()
    );
    return false;
  }

  // Initialization for time calculation.
  const auto methodStartTime = std::chrono::high_resolution_clock::now();
  const float currentTimeSecondsPattern{
    intAsFloat(static_cast<uint32_t>(static_cast<uint64_t>(timestamp)))
  };

//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  // Update initial time if it is not initialized.
  if (initialTime_ < 0) {
    initialTime_ = timestamp;
  }
  const float scanTimeSinceInitialization = (timestamp - initialTime_);

  // Store references for efficient interation.
  auto& elevationLayer = rawMap_["elevation"];
  auto& varianceLayer = rawMap_["variance"];
  auto& horizontalVarianceXLayer = rawMap_["horizontal_variance_x"];
  auto& horizontalVarianceYLayer = rawMap_["horizontal_variance_y"];
  auto& horizontalVarianceXYLayer = rawMap_["horizontal_variance_xy"];
  auto& colorLayer = rawMap_["color"];
  auto& timeLayer = rawMap_["time"];
  auto& dynamicTimeLayer = rawMap_["dynamic_time"];
  auto& lowestScanPointLayer = rawMap_["lowest_scan_point"];
  auto& sensorXatLowestScanLayer = rawMap_["sensor_x_at_lowest_scan"];
  auto& sensorYatLowestScanLayer = rawMap_["sensor_y_at_lowest_scan"];
  auto& sensorZatLowestScanLayer = rawMap_["sensor_z_at_lowest_scan"];

  std::vector<Eigen::Ref<const grid_map::Matrix>> basicLayers_;
  for (const std::string& layer : rawMap_.getBasicLayers()) {
    basicLayers_.emplace_back(rawMap_.get(layer));
  }

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    auto& point = pointCloud->points[i];
    grid_map::Index index;
    grid_map::Position position(point.x, point.y);  // NOLINT(cppcoreguidelines-pro-type-union-access)
    if (!rawMap_.getIndex(position, index)) {
      continue;  // Skip this point if it does not lie within the elevation map.
    }

    auto& elevation = elevationLayer(index(0), index(1));
    auto& variance = varianceLayer(index(0), index(1));
    auto& horizontalVarianceX = horizontalVarianceXLayer(index(0), index(1));
    auto& horizontalVarianceY = horizontalVarianceYLayer(index(0), index(1));
    auto& horizontalVarianceXY = horizontalVarianceXYLayer(index(0), index(1));
    auto& color = colorLayer(index(0), index(1));
    auto& time = timeLayer(index(0), index(1));
    auto& dynamicTime = dynamicTimeLayer(index(0), index(1));
    auto& lowestScanPoint = lowestScanPointLayer(index(0), index(1));
    auto& sensorXatLowestScan = sensorXatLowestScanLayer(index(0), index(1));
    auto& sensorYatLowestScan = sensorYatLowestScanLayer(index(0), index(1));
    auto& sensorZatLowestScan = sensorZatLowestScanLayer(index(0), index(1));

    const float& pointVariance = pointCloudVariances(i);
    bool isValid = std::all_of(basicLayers_.begin(), basicLayers_.end(),
                               [&](Eigen::Ref<const grid_map::Matrix> layer) { return std::isfinite(layer(index(0), index(1))); });
    if (!isValid) {
      // No prior information in elevation map, use measurement.
      elevation = point.z;  // NOLINT(cppcoreguidelines-pro-type-union-access)
      variance = pointVariance;
      horizontalVarianceX = parameters.minHorizontalVariance_;
      horizontalVarianceY = parameters.minHorizontalVariance_;
      horizontalVarianceXY = 0.0;
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    // Deal with multiple heights in one cell.
    const double mahalanobisDistance = fabs(point.z - elevation) / sqrt(variance);  // NOLINT(cppcoreguidelines-pro-type-union-access)
    if (mahalanobisDistance > parameters.mahalanobisDistanceThreshold_) {
      if (scanTimeSinceInitialization - time <= parameters.scanningDuration_ &&
          elevation > point.z) {  // NOLINT(cppcoreguidelines-pro-type-union-access)
        // Ignore point if measurement is from the same point cloud (time comparison) and
        // if measurement is lower then the elevation in the map.
      } else if (scanTimeSinceInitialization - time <= parameters.scanningDuration_) {
        // If point is higher.
        elevation = parameters.increaseHeightAlpha_ * elevation +
                    (1.0 - parameters.increaseHeightAlpha_) * point.z;  // NOLINT(cppcoreguidelines-pro-type-union-access)
        variance = parameters.increaseHeightAlpha_ * variance + (1.0 - parameters.increaseHeightAlpha_) * pointVariance;
      } else {
        variance += parameters.multiHeightNoise_;
      }
      continue;
    }

    // Store lowest points from scan for visibility checking.
    const float pointHeightPlusUncertainty =
        point.z + 3.0 * sqrt(pointVariance);  // 3 sigma. // NOLINT(cppcoreguidelines-pro-type-union-access)
    if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint) {
      lowestScanPoint = pointHeightPlusUncertainty;
      const grid_map::Position3 sensorTranslation(transformationSensorToMap.translation());
      sensorXatLowestScan = sensorTranslation.x();
      sensorYatLowestScan = sensorTranslation.y();
      sensorZatLowestScan = sensorTranslation.z();
    }

    // Fuse measurement with elevation map data.
    elevation =
        (variance * point.z + pointVariance * elevation) / (variance + pointVariance);  // NOLINT(cppcoreguidelines-pro-type-union-access)
    variance = (pointVariance * variance) / (pointVariance + variance);
    // TODO(max): Add color fusion.
    grid_map::colorVectorToValue(point.getRGBVector3i(), color);
    time = scanTimeSinceInitialization;
    dynamicTime = currentTimeSecondsPattern;

    // Horizontal variances are reset.
    horizontalVarianceX = parameters.minHorizontalVariance_;
    horizontalVarianceY = parameters.minHorizontalVariance_;
    horizontalVarianceXY = 0.0;
  }

  clean();
  // Point cloud stores time in nanoseconds.
  rawMap_.setTimestamp(1e9 * timestamp);

  double duration = static_cast<std::chrono::duration<double>>(
      std::chrono::high_resolution_clock::now() - methodStartTime
    ).count();
  drake::log()->trace(
      "Raw map has been updated with a new point cloud in {} s.", duration
  );
  return true;
}

bool ElevationMap::update(const grid_map::Matrix& varianceUpdate,
                          const grid_map::Matrix& horizontalVarianceUpdateX,
                          const grid_map::Matrix& horizontalVarianceUpdateY,
                          const grid_map::Matrix& horizontalVarianceUpdateXY,
                          double time) {
//  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

  const auto& size = rawMap_.getSize();

  if (!((grid_map::Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateXY.rows(), horizontalVarianceUpdateXY.cols()) == size).all())) {
    drake::log()->error("The size of the update matrices does not match.");
    return false;
  }

  rawMap_.get("variance") += varianceUpdate;
  rawMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
  rawMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
  rawMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
  clean();
  rawMap_.setTimestamp(1e9 * time);

  return true;
}

bool ElevationMap::fuseAll() {
  drake::log()->debug("Requested to fuse entire elevation map.");
//  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return fuse(grid_map::Index(0, 0), fusedMap_.getSize());
}

bool ElevationMap::fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length) {
  drake::log()->debug(
      "Requested to fuse an area of the elevation map with center at ({}, {}) "
      "and side lengths ({}, {})",
      position[0], position[1], length[0], length[1]
  );

  grid_map::Index topLeftIndex;
  grid_map::Index submapBufferSize;

  // These parameters are not used in this function.
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

//  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  grid_map::getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap, position, length,
                                 rawMap_.getLength(), rawMap_.getPosition(), rawMap_.getResolution(), rawMap_.getSize(),
                                 rawMap_.getStartIndex());

  return fuse(topLeftIndex, submapBufferSize);
}

bool ElevationMap::fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size) {
  drake::log()->debug("Fusing elevation map...");
  // Nothing to do.
  if ((size == 0).any()) {
    return false;
  }

  // Initializations.
  auto methodStartTime = std::chrono::high_resolution_clock::now();

  // Copy raw elevation map data for safe multi-threading.
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  auto rawMapCopy = rawMap_;
//  scopedLockForRawData.unlock();

//  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  // More initializations.
  const double halfResolution = fusedMap_.getResolution() / 2.0;
  const float minimalWeight = std::numeric_limits<float>::epsilon() * static_cast<float>(2.0);
  // Conservative cell inclusion for ellipse iterator.
  const double ellipseExtension = M_SQRT2 * fusedMap_.getResolution();

  // Check if there is the need to reset out-dated data.
  if (fusedMap_.getTimestamp() != rawMapCopy.getTimestamp()) {
    resetFusedData();
  }

  // Align fused map with raw map.
  if (rawMapCopy.getPosition() != fusedMap_.getPosition()) {
    fusedMap_.move(rawMapCopy.getPosition());
  }

  // For each cell in requested area.
  for (grid_map::SubmapIterator areaIterator(rawMapCopy, topLeftIndex, size); !areaIterator.isPastEnd(); ++areaIterator) {
    // Check if fusion for this cell has already been done earlier.
    if (fusedMap_.isValid(*areaIterator)) {
      continue;
    }

    if (!rawMapCopy.isValid(*areaIterator)) {
      // This is an empty cell (hole in the map).
      // TODO(max):
      continue;
    }

    // Get size of error ellipse.
    const float& sigmaXsquare = rawMapCopy.at("horizontal_variance_x", *areaIterator);
    const float& sigmaYsquare = rawMapCopy.at("horizontal_variance_y", *areaIterator);
    const float& sigmaXYsquare = rawMapCopy.at("horizontal_variance_xy", *areaIterator);

    Eigen::Matrix2d covarianceMatrix;
    covarianceMatrix << sigmaXsquare, sigmaXYsquare, sigmaXYsquare, sigmaYsquare;
    // 95.45% confidence ellipse which is 2.486-sigma for 2 dof problem.
    // http://www.reid.ai/2012/09/chi-squared-distribution-table-with.html
    const double uncertaintyFactor = 2.486;  // sqrt(6.18)
    Eigen::EigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Array2d eigenvalues(solver.eigenvalues().real().cwiseAbs());

    Eigen::Array2d::Index maxEigenvalueIndex{0};
    eigenvalues.maxCoeff(&maxEigenvalueIndex);
    Eigen::Array2d::Index minEigenvalueIndex{0};
    maxEigenvalueIndex == Eigen::Array2d::Index(0) ? minEigenvalueIndex = 1 : minEigenvalueIndex = 0;
    const grid_map::Length ellipseLength =
        2.0 * uncertaintyFactor * grid_map::Length(eigenvalues(maxEigenvalueIndex), eigenvalues(minEigenvalueIndex)).sqrt() +
        ellipseExtension;
    const double ellipseRotation(
        atan2(solver.eigenvectors().col(maxEigenvalueIndex).real()(1), solver.eigenvectors().col(maxEigenvalueIndex).real()(0)));

    // Requested length and position (center) of submap in map.
    grid_map::Position requestedSubmapPosition;
    rawMapCopy.getPosition(*areaIterator, requestedSubmapPosition);
    grid_map::EllipseIterator ellipseIterator(rawMapCopy, requestedSubmapPosition, ellipseLength, ellipseRotation);

    // Prepare data fusion.
    Eigen::ArrayXf means;
    Eigen::ArrayXf weights;
    const unsigned int maxNumberOfCellsToFuse = ellipseIterator.getSubmapSize().prod();
    means.resize(maxNumberOfCellsToFuse);
    weights.resize(maxNumberOfCellsToFuse);
    WeightedEmpiricalCumulativeDistributionFunction<float> lowerBoundDistribution;
    WeightedEmpiricalCumulativeDistributionFunction<float> upperBoundDistribution;

    float maxStandardDeviation = sqrt(eigenvalues(maxEigenvalueIndex));
    float minStandardDeviation = sqrt(eigenvalues(minEigenvalueIndex));
    Eigen::Rotation2Dd rotationMatrix(ellipseRotation);
    std::string maxEigenvalueLayer;
    std::string minEigenvalueLayer;
    if (maxEigenvalueIndex == 0) {
      maxEigenvalueLayer = "horizontal_variance_x";
      minEigenvalueLayer = "horizontal_variance_y";
    } else {
      maxEigenvalueLayer = "horizontal_variance_y";
      minEigenvalueLayer = "horizontal_variance_x";
    }

    // For each cell in error ellipse.
    size_t i = 0;
    for (; !ellipseIterator.isPastEnd(); ++ellipseIterator) {
      if (!rawMapCopy.isValid(*ellipseIterator)) {
        // Empty cell in submap (cannot be center cell because we checked above).
        continue;
      }

      means[i] = rawMapCopy.at("elevation", *ellipseIterator);

      // Compute weight from probability.
      grid_map::Position absolutePosition;
      rawMapCopy.getPosition(*ellipseIterator, absolutePosition);
      Eigen::Vector2d distanceToCenter = (rotationMatrix * (absolutePosition - requestedSubmapPosition)).cwiseAbs();

      float probability1 = cumulativeDistributionFunction(distanceToCenter.x() + halfResolution, 0.0, maxStandardDeviation) -
                           cumulativeDistributionFunction(distanceToCenter.x() - halfResolution, 0.0, maxStandardDeviation);
      float probability2 = cumulativeDistributionFunction(distanceToCenter.y() + halfResolution, 0.0, minStandardDeviation) -
                           cumulativeDistributionFunction(distanceToCenter.y() - halfResolution, 0.0, minStandardDeviation);

      const float weight = std::max(minimalWeight, probability1 * probability2);
      weights[i] = weight;
      const float standardDeviation = sqrt(rawMapCopy.at("variance", *ellipseIterator));
      lowerBoundDistribution.add(means[i] - 2.0 * standardDeviation, weight);
      upperBoundDistribution.add(means[i] + 2.0 * standardDeviation, weight);

      i++;
    }

    if (i == 0) {
      // Nothing to fuse.
      fusedMap_.at("elevation", *areaIterator) = rawMapCopy.at("elevation", *areaIterator);
      fusedMap_.at("lower_bound", *areaIterator) =
          rawMapCopy.at("elevation", *areaIterator) - 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("upper_bound", *areaIterator) =
          rawMapCopy.at("elevation", *areaIterator) + 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
      continue;
    }

    // Fuse.
    means.conservativeResize(i);
    weights.conservativeResize(i);

    float mean = (weights * means).sum() / weights.sum();

    if (!std::isfinite(mean)) {
      drake::log()->error(
          "Something went wrong when fusing the map: Mean = {}", mean
      );
      continue;
    }

    // Add to fused map.
    fusedMap_.at("elevation", *areaIterator) = mean;
    lowerBoundDistribution.compute();
    upperBoundDistribution.compute();
    fusedMap_.at("lower_bound", *areaIterator) = lowerBoundDistribution.quantile(0.01);  // TODO(max):
    fusedMap_.at("upper_bound", *areaIterator) = upperBoundDistribution.quantile(0.99);  // TODO(max):
    // TODO(max): Add fusion of colors.
    fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
  }

  fusedMap_.setTimestamp(rawMapCopy.getTimestamp());

  const auto duration = std::chrono::high_resolution_clock::now() - methodStartTime;
  drake::log()->debug("Elevation map has been fused in {} s.", duration.count());

  return true;
}

bool ElevationMap::clear() {
  // Lock raw and fused map object in different scopes to prevent deadlock.
  {
//    boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
    rawMap_.clearAll();
    rawMap_.resetTimestamp();
    rawMap_.get("dynamic_time").setZero();
  }
  {
//    boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
    fusedMap_.clearAll();
    fusedMap_.resetTimestamp();
  }
  return true;
}

void ElevationMap::visibilityCleanup(double updatedTime) {
  const Parameters parameters{parameters_};
  // Get current time to compute calculation time.
  const auto methodStartTime = std::chrono::high_resolution_clock::now();
  const double timeSinceInitialization = updatedTime - initialTime_;

  // Copy raw elevation map data for safe multi-threading.
//  boost::recursive_mutex::scoped_lock scopedLockForVisibilityCleanupData(visibilityCleanupMapMutex_);
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  visibilityCleanupMap_ = rawMap_;
  rawMap_.clear("lowest_scan_point");
  rawMap_.clear("sensor_x_at_lowest_scan");
  rawMap_.clear("sensor_y_at_lowest_scan");
  rawMap_.clear("sensor_z_at_lowest_scan");
//  scopedLockForRawData.unlock();
  visibilityCleanupMap_.add("max_height");

  // Create max. height layer with ray tracing.
  for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
    if (!visibilityCleanupMap_.isValid(*iterator)) {
      continue;
    }
    const auto& lowestScanPoint = visibilityCleanupMap_.at("lowest_scan_point", *iterator);
    const auto& sensorXatLowestScan = visibilityCleanupMap_.at("sensor_x_at_lowest_scan", *iterator);
    const auto& sensorYatLowestScan = visibilityCleanupMap_.at("sensor_y_at_lowest_scan", *iterator);
    const auto& sensorZatLowestScan = visibilityCleanupMap_.at("sensor_z_at_lowest_scan", *iterator);
    if (std::isnan(lowestScanPoint)) {
      continue;
    }
    grid_map::Index indexAtSensor;
    if (!visibilityCleanupMap_.getIndex(grid_map::Position(sensorXatLowestScan, sensorYatLowestScan), indexAtSensor)) {
      continue;
    }
    grid_map::Position point;
    visibilityCleanupMap_.getPosition(*iterator, point);
    float pointDiffX = point.x() - sensorXatLowestScan;
    float pointDiffY = point.y() - sensorYatLowestScan;
    float distanceToPoint = sqrt(pointDiffX * pointDiffX + pointDiffY * pointDiffY);
    if (distanceToPoint > 0.0) {
      for (grid_map::LineIterator iterator(visibilityCleanupMap_, indexAtSensor, *iterator); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position cellPosition;
        visibilityCleanupMap_.getPosition(*iterator, cellPosition);
        const float cellDiffX = cellPosition.x() - sensorXatLowestScan;
        const float cellDiffY = cellPosition.y() - sensorYatLowestScan;
        const float distanceToCell = distanceToPoint - sqrt(cellDiffX * cellDiffX + cellDiffY * cellDiffY);
        const float maxHeightPoint = lowestScanPoint + (sensorZatLowestScan - lowestScanPoint) / distanceToPoint * distanceToCell;
        auto& cellMaxHeight = visibilityCleanupMap_.at("max_height", *iterator);
        if (std::isnan(cellMaxHeight) || cellMaxHeight > maxHeightPoint) {
          cellMaxHeight = maxHeightPoint;
        }
      }
    }
  }

  // Vector of indices that will be removed.
  std::vector<grid_map::Position> cellPositionsToRemove;
  for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
    if (!visibilityCleanupMap_.isValid(*iterator)) {
      continue;
    }
    const auto& time = visibilityCleanupMap_.at("time", *iterator);
    if (timeSinceInitialization - time > parameters.scanningDuration_) {
      // Only remove cells that have not been updated during the last scan duration.
      // This prevents a.o. removal of overhanging objects.
      const auto& elevation = visibilityCleanupMap_.at("elevation", *iterator);
      const auto& variance = visibilityCleanupMap_.at("variance", *iterator);
      const auto& maxHeight = visibilityCleanupMap_.at("max_height", *iterator);
      if (!std::isnan(maxHeight) && elevation - 3.0 * sqrt(variance) > maxHeight) {
        grid_map::Position position;
        visibilityCleanupMap_.getPosition(*iterator, position);
        cellPositionsToRemove.push_back(position);
      }
    }
  }

  // Remove points in current raw map.
//  scopedLockForRawData.lock();
  for (const auto& cellPosition : cellPositionsToRemove) {
    grid_map::Index index;
    if (!rawMap_.getIndex(cellPosition, index)) {
      continue;
    }
    if (rawMap_.isValid(index)) {
      rawMap_.at("elevation", index) = NAN;
      rawMap_.at("dynamic_time", index) = 0.0f;
    }
  }
//  scopedLockForRawData.unlock();

  const auto duration = std::chrono::high_resolution_clock::now() - methodStartTime;
  drake::log()->debug(
      "Visibility cleanup has been performed in {} s ({} points).",
      duration.count(), static_cast<int>(cellPositionsToRemove.size())
  );
  if (duration.count() > parameters.visibilityCleanupDuration_) {
    drake::log()->warn(
        "Visibility cleanup duration is too high (current duration is {}).",
        duration.count()
    );
  }
}

void ElevationMap::move(const Eigen::Vector2d& position) {
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  std::vector<grid_map::BufferRegion> newRegions;

  if (rawMap_.move(position, newRegions)) {
    drake::log()->debug("Elevation map has been moved to position ({}, {}).",
                        rawMap_.getPosition().x(), rawMap_.getPosition().y());

    // The "dynamic_time" layer is meant to be interpreted as integer values, therefore nan:s need to be zeroed.
    grid_map::Matrix& dynTime{rawMap_.get("dynamic_time")};
    dynTime = dynTime.array().isNaN().select(grid_map::Matrix::Scalar(0.0f), dynTime.array());

    if (hasUnderlyingMap_) {
      rawMap_.addDataFrom(underlyingMap_, false, false, true);
    }
  }
}

const grid_map::GridMap& ElevationMap::getRawGridMap() const {
  return rawMap_;
}

void ElevationMap::setRawGridMap(const grid_map::GridMap& map) {
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_ = map;
}

const grid_map::GridMap& ElevationMap::getFusedGridMap() const {
  return fusedMap_;
}

void ElevationMap::setFusedGridMap(const grid_map::GridMap& map) {
//  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  fusedMap_ = map;
}

double ElevationMap::getTimeOfLastUpdate() {
  return rawMap_.getTimestamp() * 1e-9;
}

double ElevationMap::getTimeOfLastFusion() {
//  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return fusedMap_.getTimestamp() * 1e-9;
}

const drake::math::RigidTransformd& ElevationMap::getPose() {
  return pose_;
}

bool ElevationMap::getPosition3dInRobotParentFrame(
    const Eigen::Array2i& index, Eigen::Vector3d& position) {
  Eigen::Vector3d positionInGridFrame;
  if (!rawMap_.getPosition3("elevation", index, positionInGridFrame)) {
    return false;
  }
  position = pose_ * (positionInGridFrame);
  return true;
}

//boost::recursive_mutex& ElevationMap::getFusedDataMutex() {
//  return fusedMapMutex_;
//}
//
//boost::recursive_mutex& ElevationMap::getRawDataMutex() {
//  return rawMapMutex_;
//}

bool ElevationMap::clean() {
  const Parameters parameters{parameters_};
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_.get("variance") =
      rawMap_.get("variance").unaryExpr(VarianceClampOperator<float>(parameters.minVariance_, parameters.maxVariance_));
  rawMap_.get("horizontal_variance_x") =
      rawMap_.get("horizontal_variance_x")
          .unaryExpr(VarianceClampOperator<float>(parameters.minHorizontalVariance_, parameters.maxHorizontalVariance_));
  rawMap_.get("horizontal_variance_y") =
      rawMap_.get("horizontal_variance_y")
          .unaryExpr(VarianceClampOperator<float>(parameters.minHorizontalVariance_, parameters.maxHorizontalVariance_));
  return true;
}

void ElevationMap::resetFusedData() {
//  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  fusedMap_.clearAll();
  fusedMap_.resetTimestamp();
}

void ElevationMap::setFrameId(const std::string& frameId) {
  rawMap_.setFrameId(frameId);
  fusedMap_.setFrameId(frameId);
}

void ElevationMap::setTimestamp(double timestamp) {
  rawMap_.setTimestamp(timestamp * 1e9);
  fusedMap_.setTimestamp(timestamp * 1e9);
}

const std::string& ElevationMap::getFrameId() {
  return rawMap_.getFrameId();
}

void ElevationMap::updateUnderlyingMap(const grid_map::GridMap& underlyingMap) {
  const Parameters parameters{parameters_};
  drake::log()->debug("Updating underlying map.");

  DRAKE_DEMAND(underlyingMap.getFrameId() == rawMap_.getFrameId());
  DRAKE_DEMAND(underlyingMap.exists("elevation"));

  underlyingMap_ = underlyingMap;
  if (!underlyingMap_.exists("variance")) {
    underlyingMap_.add("variance", parameters.minVariance_);
  }
  if (!underlyingMap_.exists("horizontal_variance_x")) {
    underlyingMap_.add("horizontal_variance_x",
                       parameters.minHorizontalVariance_);
  }
  if (!underlyingMap_.exists("horizontal_variance_y")) {
    underlyingMap_.add("horizontal_variance_y",
                       parameters.minHorizontalVariance_);
  }
  if (!underlyingMap_.exists("color")) {
    underlyingMap_.add("color", 0.0);
  }
  underlyingMap_.setBasicLayers(rawMap_.getBasicLayers());
  hasUnderlyingMap_ = true;
  rawMap_.addDataFrom(underlyingMap_, false, false, true);
}

void ElevationMap::setRawSubmapHeight(const grid_map::Position& initPosition,
                                      float mapHeight,
                                      float variance,
                                      double lengthInXSubmap,
                                      double lengthInYSubmap) {
  // Set a submap area (lengthInYSubmap, lengthInXSubmap) with a constant
  // height (mapHeight) and variance.
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  // Calculate submap iterator start index.
  const grid_map::Position topLeftPosition(initPosition(0) + lengthInXSubmap / 2, initPosition(1) + lengthInYSubmap / 2);
  grid_map::Index submapTopLeftIndex;
  rawMap_.getIndex(topLeftPosition, submapTopLeftIndex);

  // Calculate submap area.
  const double resolution = rawMap_.getResolution();
  const int lengthInXSubmapI = static_cast<int>(lengthInXSubmap / resolution);
  const int lengthInYSubmapI = static_cast<int>(lengthInYSubmap / resolution);
  const Eigen::Array2i submapBufferSize(lengthInYSubmapI, lengthInXSubmapI);

  // Iterate through submap and fill height values.
  grid_map::Matrix& elevationData = rawMap_["elevation"];
  grid_map::Matrix& varianceData = rawMap_["variance"];
  for (grid_map::SubmapIterator iterator(rawMap_, submapTopLeftIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    elevationData(index(0), index(1)) = mapHeight;
    varianceData(index(0), index(1)) = variance;
  }
}

void ElevationMap::shift_map_z(double delta_z) {
  const auto& l = rawMap_.getLength();
  auto& elevation = rawMap_["elevation"];
  elevation += (delta_z * grid_map::Matrix::Ones(elevation.rows(), elevation.cols()));
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation) {
  return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
}

}  // namespace elevation_mapping
