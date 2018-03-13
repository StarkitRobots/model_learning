#pragma once

#include "rhoban_model_learning/modular_model.h"
#include "rhoban_model_learning/input_reader.h"

#include "Model/HumanoidModel.hpp"
#include "Model/NamesModel.h"

namespace rhoban_model_learning
{

/// This class is used to represent the correction of the model based on vision
/// informations. It can be used to learn static offset in various locations.
///
/// Inputs are composed of:
/// - Position of all the DOF and the IMU
/// - Expected position of the data in the left_foot referential
///
/// Observations are positions of the object in the image (x,y)
class VisionCorrectionModel : public ModularModel {
public:

  class VisionInput : public Input {
  public:
    VisionInput();
    VisionInput(const Leph::VectorLabel & data);
    VisionInput(const VisionInput & other);

    virtual std::unique_ptr<Input> clone() const override;

    /// Contains all DOF + IMU + pixel position
    Leph::VectorLabel data;
  };

  class VisionInputReader : public InputReader {
  public:
    VisionInputReader();

    /// Throw an error if:
    /// - none of `nb_training_tags` and `nb_validation_tags` is provided
    /// - The sum of `nb_training_tags` and `nb_validation_tags` is higher
    ///   than the number of valid tags
    virtual DataSet extractSamples(const std::string & file_path,
                                   std::default_random_engine * engine) const override;

    virtual std::string getClassName() const override;
    Json::Value toJson() const override;
    void fromJson(const Json::Value & v, const std::string & dir_name) override;
  private:
    /// The maximal distance to image center relative to image size in [0,1]
    double max_coordinates;
    /// Number of tags used for the training set
    /// If value is 0, the number of tags is deduced from nb_validation_tags
    int nb_training_tags;
    /// Number of tags used for validation. By choosing to use separate training
    /// set and validation set based on tagId, we ensure that the validation set
    /// is really different from the training set
    /// If value is 0, the number of tags is deduced from nb_training_tags
    int nb_validation_tags;
    /// The maximal number of samples per tag allowed in training_set
    int max_samples_per_tag;
    /// If a tag is represented less than min_samples_per_tag, it is ignored
    int min_samples_per_tag;
    /// Is the reading process printing summary of reading?
    bool verbose;
  };


  VisionCorrectionModel();
  VisionCorrectionModel(const VisionCorrectionModel & other);

  /// Return the [roll,yaw,pitch] offsets of camera using [rad]
  Eigen::Vector3d getCameraOffsetsRad() const;
  /// Return the [roll,yaw,pitch] offsets of the IMU using [rad]
  Eigen::Vector3d getImuOffsetsRad() const;
  /// Return the [roll,yaw,pitch] offsets of neck using [rad]
  Eigen::Vector3d getNeckOffsetsRad() const;
  /// Return the parameters of the camera using Leph format
  Leph::CameraParameters getCameraParameters() const;

  virtual Eigen::VectorXd getGlobalParameters() const override;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const override;
  virtual void setGlobalParameters(const Eigen::VectorXd & new_params) override;
  virtual std::vector<std::string> getGlobalParametersNames() const override;
  virtual Eigen::VectorXi getObservationsCircularity() const override;

  virtual Eigen::VectorXd
  predictObservation(const Input & input,
                     std::default_random_engine * engine) const;

  /// Density
  virtual double computeLogLikelihood(const Sample & sample,
                                      std::default_random_engine * engine) const override;

  virtual std::unique_ptr<Model> clone() const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const;

  /// Convert from leph coordinates ([-1,1]^2) to img coordinates ([0,width]*[0,height])
  Eigen::Vector2d leph2Img(const Eigen::Vector2d & leph_coordinates) const;
  /// Convert from img coordinates ([0,width]*[0,height]) to leph coordinates ([-1 1]^2)
  Eigen::Vector2d img2Leph(const Eigen::Vector2d & img_coordinates) const;

private:
  /// The observation standard deviation in pixels
  double px_stddev;

  /// Offset at camera fixation (roll, pitch, yaw) [deg]
  Eigen::Vector3d cam_offset;
  /// Offset between IMU orientation and trunk orientation (roll, pitch, yaw) [deg]
  Eigen::Vector3d imu_offset;
  /// Offset in neck (roll, pitch, yaw) [deg]
  Eigen::Vector3d neck_offset;

  /// Width aperture angle for camera [deg]
  double camera_pan;

  /// Height aperture angle for camera [deg]
  double camera_tilt;

  /// Width of the image in pixels
  int img_width;
  /// Height of the image in pixels
  int img_height;

  /// Space for tuning px_stddev [min,max]
  Eigen::Vector2d px_stddev_space;

  /// Maximal angle error [deg]
  double max_angle_error;

  /// Allowed space for tuning the camera pan [deg]
  Eigen::Vector2d camera_pan_space;
  /// Allowed space for tuning the camera tilt [deg]
  Eigen::Vector2d camera_tilt_space;
};

}
