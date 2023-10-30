/*
 * ElevationMap.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Drake rigid transform
#include "drake/math/rigid_transform.h"

// Boost
#include <boost/thread/recursive_mutex.hpp>

// Elevation Mapping
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"

namespace elevation_mapping {

/*!
 * Elevation map stored as grid map handling elevation height, variance, color etc.
 */
class ElevationMap {
 public:
  /*!
   * Constructor.
   */
  explicit ElevationMap(const std::string& parameter_yaml = "");

  /*!
   * Destructor.
   */
  virtual ~ElevationMap();

  /*!
   * Set the geometry of the elevation map. Clears all the data.
   * @param length the side lengths in x, and y-direction of the elevation map [m].
   * @param resolution the cell size in [m/cell].
   * @param position the 2d position of the elevation map in the elevation map frame [m].
   * @return true if successful.
   */
  void setGeometry(const grid_map::Length& length,
                   const double& resolution,
                   const grid_map::Position& position);

  /*!
   * Add new measurements to the elevation map.
   * @param pointCloud the point cloud data.
   * @param pointCloudVariances the corresponding variances of the point cloud data.
   * @param timeStamp the time of the input point cloud.
   * @param transformationSensorToMap
   * @return true if successful.
   */
  bool add(PointCloudType::Ptr pointCloud,
           Eigen::VectorXf& pointCloudVariances,
           double timeStamp,
           const drake::math::RigidTransformd& transformationSensorToMap);

  /*!
   * Update the elevation map with variance update data.
   * @param varianceUpdate the variance update in vertical direction.
   * @param horizontalVarianceUpdateX the variance update in horizontal x-direction.
   * @param horizontalVarianceUpdateY the variance update in horizontal y-direction.
   * @param horizontalVarianceUpdateXY the correlated variance update in horizontal xy-direction.
   * @param time the time of the update.
   * @return true if successful.
   */
  bool update(const grid_map::Matrix& varianceUpdate,
              const grid_map::Matrix& horizontalVarianceUpdateX,
              const grid_map::Matrix& horizontalVarianceUpdateY,
              const grid_map::Matrix& horizontalVarianceUpdateXY, double time);

  /*!
   * Triggers the fusion of the entire elevation map.
   * @return true if successful.
   */
  bool fuseAll();

  /*!
   * Fuses the elevation map for a certain rectangular area.
   * @param position the center position of the area to fuse.
   * @param length the sides lengths of the area to fuse.
   * @return true if successful.
   */
  bool fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length);

  /*!
   * Clears all data of the elevation map (data and time).
   * @return true if successful.
   */
  bool clear();

  /*!
   * Removes parts of the map based on visibility criterion with ray tracing.
   * @param transformationSensorToMap
   * @param updatedTime
   */
  void visibilityCleanup(double updatedTime);

  /*!
   * Move the grid map w.r.t. to the grid map frame.
   * @param position the new location of the elevation map in the map frame.
   */
  void move(const Eigen::Vector2d& position);

  /*!
   * Gets a reference to the raw grid map.
   * @return the raw grid map.
   */
  const grid_map::GridMap& getRawGridMap() const;

  /*!
   * Sets a raw grid map.
   * @param map The input raw grid map to set.
   */
  void setRawGridMap(const grid_map::GridMap& map);

  /*!
   * Gets a reference to the fused grid map.
   * @return the fused grid map.
   */
  const grid_map::GridMap& getFusedGridMap() const;

  /*!
   * Sets a fused grid map.
   * @param map The input fused grid map to set.
   */
  void setFusedGridMap(const grid_map::GridMap& map);

  /*!
   * Gets the time of last map update.
   * @return time of the last map update.
   */
  double getTimeOfLastUpdate();

  /*!
   * Gets the time of last map fusion.
   * @return time of the last map fusion.
   */
  double getTimeOfLastFusion();

  /*!
   * Get the pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
   * @return pose of the elevation map frame w.r.t. the parent frame of the robot.
   */
  const drake::math::RigidTransformd& getPose();

  /*!
   * Gets the position of a raw data point (x, y of cell position & height of cell value) in
   * the parent frame of the robot.
   * @param index the index of the requested cell.
   * @param position the position of the data point in the parent frame of the robot.
   * @return true if successful, false if no valid data available.
   */
  bool getPosition3dInRobotParentFrame(const Eigen::Array2i& index,
                                       Eigen::Vector3d& position);


  /*
   * Since we are using ElevationMap within the drake systems framework,
   * we will keep its default copy constructor and let the systems framework
   * handle thread safety.
   *
   *!
   * Gets the fused data mutex.
   * @return reference to the fused data mutex.

    boost::recursive_mutex& getFusedDataMutex();
    boost::recursive_mutex& getRawDataMutex();
    */

  /*!
   * Set the frame id.
   * @param frameId the frame id.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frame id.
   * @return the frameId.
   */
  const std::string& getFrameId();

  /*!
   * Set the timestamp of the raw and fused elevation map.
   * @param timestmap to set.
   */
  void setTimestamp(double time);

  /*!
   * If the raw elevation map has subscribers.
   * @return true if number of subscribers bigger then 0.
   */
  bool hasRawMapSubscribers() const;

  /*!
   * If the fused elevation map has subscribers.
   * @return true if number of subscribers bigger then 0.
   */
  bool hasFusedMapSubscribers() const;

  /*!
   * Method for the updates of the underlying map.
   * Updates the internal underlying map.
   * @param underlyingMap the underlying map.
   */
  void updateUnderlyingMap(const grid_map::GridMap& underlyingMap);

  /*!
   * Method to set the height value around the center of the robot, can be used for initialization.
   * @param initPosition Position to calculate inner rectangle.
   * @param mapHeight The height that gets set uniformly.
   * @param lengthInXSubmap Length of the submap in X direction.
   * @param lengthInYSubmap Length of the submap in Y direction.
   */
  void setRawSubmapHeight(const grid_map::Position& initPosition,
                          float mapHeight,
                          float variance,
                          double lengthInXSubmap,
                          double lengthInYSubmap);

  friend class ElevationMapping;

 private:

  /*!
   * Loads parameters from a yaml and assigns them to the member params struct
   */
  bool loadParams(const std::string& parameter_yaml);

  /*!
   * Fuses a region of the map.
   * @param topLeftIndex the top left index of the region.
   * @param size the size (in number of cells) of the region.
   * @return true if successful.
   */
  bool fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size);

  /*!
   * Cleans the elevation map data to stay within the specified bounds.
   * @return true if successful.
   */
  bool clean();

  /*!
   * Resets the fused map data.
   * @return true if successful.
   */
  void resetFusedData();

  /*!
   * Cumulative distribution function.
   * @param x the argument value.
   * @param mean the mean of the distribution.
   * @param standardDeviation the standardDeviation of the distribution.
   * @return the function value.
   */
  static float cumulativeDistributionFunction(float x, float mean,
                                              float standardDeviation);

  //! Raw elevation map as grid map.
  grid_map::GridMap rawMap_;

  //! Fused elevation map as grid map.
  grid_map::GridMap fusedMap_;

  //! Visibility cleanup debug map.
  grid_map::GridMap visibilityCleanupMap_;

  //! Underlying map, used for ground truth maps, multi-robot mapping etc.
  grid_map::GridMap underlyingMap_;

  //! True if underlying map has been set, false otherwise.
  bool hasUnderlyingMap_;

  //! Pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
  drake::math::RigidTransformd pose_;

  /*
   * Since we are using ElevationMap within the drake systems framework,
   * we will keep its default copy constructor and let the systems framework
   * handle thread safety.
   *
  //! Mutex lock for fused map.
  boost::recursive_mutex fusedMapMutex_;

  //! Mutex lock for raw map.
  boost::recursive_mutex rawMapMutex_;

  //! Mutex lock for visibility cleanup map.
  boost::recursive_mutex visibilityCleanupMapMutex_;
  */

  //! Initial time
  double initialTime_ = -1;

  //! Parameters. Are set through the ElevationMapping class.
  struct Parameters {
    double minVariance_{0.000009};
    double maxVariance_{0.0009};
    double mahalanobisDistanceThreshold_{2.5};
    double multiHeightNoise_{0.000009};
    double minHorizontalVariance_{0.0001};
    double maxHorizontalVariance_{0.05};
    std::string underlyingMapTopic_;
    bool enableVisibilityCleanup_{true};
    bool enableContinuousCleanup_{false};
    double visibilityCleanupDuration_{0.0};
    double scanningDuration_{1.0};
    double increaseHeightAlpha_{1.0};
  };
  Parameters parameters_;
};

}  // namespace elevation_mapping
