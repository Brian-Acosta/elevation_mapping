/*
 * SensorProcessorBase.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Eigen
#include <Eigen/Core>

// STL
#include <memory>
#include <string>
#include <unordered_map>

// Elevation Mapping
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"

// drake
#include "drake/math/rigid_transform.h"

namespace elevation_mapping {

/*!
 * Generic Sensor processor base class. Provides functionalities
 * common to all sensors and defines the interface for specialized
 * sensor processor classes.
 * Cleans the point cloud, transforms it to a desired frame, and
 * computes the measurement variances based on a sensor model in
 * the desired frame.
 */
class SensorProcessorBase {
 public:
  using Ptr = std::unique_ptr<SensorProcessorBase>;
  friend class ElevationMapping;
  friend class Input;

  struct GeneralParameters {
    std::string robotBaseFrameId_;
    std::string mapFrameId_;

    explicit GeneralParameters(
        std::string robotBaseFrameId = "robot", std::string mapFrameId = "map")
        : robotBaseFrameId_(std::move(robotBaseFrameId)), mapFrameId_(std::move(mapFrameId)) {}
  };

  /*!
   * Constructor.
   * @param generalConfig General parameters that the sensor processor must know in order to work. // TODO (magnus) improve documentation.
   */
  SensorProcessorBase(const GeneralParameters& generalConfig);

  /*!
   * Destructor.
   */
  virtual ~SensorProcessorBase();

  /*!
   * Processes the point cloud.
   * @param[in] pointCloudInput the input point cloud.
   * @param[in] targetFrame the frame to which the point cloud should be transformed. // TODO Update.
   * @param[out] pointCloudOutput the processed point cloud.
   * @param[out] variances the measurement variances expressed in the target frame.
   * @return true if successful.
   */
  bool process(PointCloudType::ConstPtr pointCloudInput,
               const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
               PointCloudType::Ptr pointCloudMapFrame,
               Eigen::VectorXf& variances,
               const drake::math::RigidTransformd& sensorPoseInBaseFrame,
               const drake::math::RigidTransformd& robotBaseFramePoseInMapFrame);

  /*!
   * Checks if a valid tf transformation was received since startup.
   * @return True if there was one valid tf transformation.
   */
  bool isTfAvailableInBuffer() const { return firstTfAvailable_; }

 protected:

  /*!
 * \brief Gets a skew-symmetric matrix from a (column) vector
 * \param   vec 3x1-matrix (column vector)
 * \return skew   3x3-matrix
 */
  template<typename PrimType_>
  inline static Eigen::Matrix<PrimType_, 3, 3> getSkewMatrixFromVector(const Eigen::Matrix<PrimType_, 3, 1>& vec) {
    Eigen::Matrix<PrimType_, 3, 3> mat;
    mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return mat;
  }

  /*!
   * Update the transformations for a given time stamp.
   * @param timeStamp the time stamp for the transformation.
   * @return true if successful.
   */
  bool updateTransformations(
      const drake::math::RigidTransformd& sensorPoseInBaseFrame,
      const drake::math::RigidTransformd& robotBaseFramePoseInMapFrame);

  /*!
   * Reads and verifies the parameters.
   * @return true if successful.
   */
  virtual bool readParameters(const std::string& yaml_filename);

  /*!
   * Filters the point cloud regardless of the sensor type. Removes NaN values.
   * Optionally, applies voxelGridFilter to reduce number of points in
   * the point cloud.
   * @param pointCloud the point cloud to clean.
   * @return true if successful.
   */
  bool filterPointCloud(PointCloudType::Ptr pointCloud);

  /*!
   * Sensor specific point cloud cleaning.
   * @param pointCloud the point cloud to clean.
   * @return true if successful.
   */
  virtual bool filterPointCloudSensorType(PointCloudType::Ptr pointCloud);

  /*!
   * Computes the elevation map height variances for each point in a point cloud with the
   * sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
  virtual bool computeVariances(PointCloudType::ConstPtr pointCloud,
                                const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                Eigen::VectorXf& variances) = 0;

  /*!
   * Transforms the point cloud the a target frame.
   * @param[in] pointCloud the point cloud to be transformed.
   * @param[out] pointCloudTransformed the resulting point cloud after transformation.
   * @param[in] targetFrame the desired target frame.
   * @return true if successful.
   */
  bool transformPointCloud(PointCloudType::ConstPtr pointCloud,
                           PointCloudType::Ptr pointCloudTransformed,
                           const drake::math::RigidTransformd& transformPointCloudToTargetFrame);

  /*!
   * Removes points with z-coordinate above a limit in map frame.
   * @param[in/out] pointCloud the point cloud to be cropped.
   */
  void removePointsOutsideLimits(PointCloudType::ConstPtr reference,
                                 std::vector<PointCloudType::Ptr>& pointClouds);


  //! Rotation from Base to Sensor frame (R_SB)
  drake::math::RotationMatrixd rotationBaseToSensor_;

  //! Translation from Base to Sensor in Base frame (B_r_BS)
  Eigen::Vector3d translationBaseToSensorInBaseFrame_;

  //! Rotation from (elevation) Map to Base frame (C_BM)
  drake::math::RotationMatrixd rotationMapToBase_;

  //! Translation from Map to Base in Map frame (M_r_MB)
  Eigen::Vector3d translationMapToBaseInMapFrame_;

  //! Transformation from Sensor to Map frame
  Eigen::Affine3d transformationSensorToMap_;

  GeneralParameters generalParameters_;

  struct Parameters {
    //! Ignore points above this height in map frame.
    double ignorePointsUpperThreshold_{std::numeric_limits<double>::infinity()};

    //! Ignore points below this height in map frame.
    double ignorePointsLowerThreshold_{-std::numeric_limits<double>::infinity()};

    //! Use VoxelGrid filter to cleanup pointcloud if true.
    bool applyVoxelGridFilter_{false};

    //! Sensor parameters.
    std::unordered_map<std::string, double> sensorParameters_;
  };
  ThreadSafeDataWrapper<Parameters> parameters_;

  //! TF frame id of the range sensor for the point clouds.
  std::string sensorFrameId_;

  //! Indicates if the requested tf transformation was available.
  bool firstTfAvailable_;
};

} /* namespace elevation_mapping */
