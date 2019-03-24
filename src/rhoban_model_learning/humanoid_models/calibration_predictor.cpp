#include "rhoban_model_learning/humanoid_models/calibration_predictor.h"

#include "rhoban_model_learning/humanoid_models/calibration_input.h"
#include "rhoban_model_learning/humanoid_models/calibration_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include "Model/HumanoidFixedModel.hpp"
#include "Model/NamesModel.h"
#include "Types/MatrixLabel.hpp"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
Eigen::VectorXd CalibrationPredictor::predictObservation(const Input& raw_input, const Model& raw_model,
                                                         std::default_random_engine* engine) const
{
  // Can generate bad_cast error
  const CalibrationInput& input = dynamic_cast<const CalibrationInput&>(raw_input);
  const CalibrationModel& calibration_model = dynamic_cast<const CalibrationModel&>(raw_model);
  // TODO: improve/understand what is hidden here (content imported from
  // appCameraModelLearning)
  // Importing geometry data from a default model
  Leph::HumanoidModel tmpModel(Leph::SigmabanModel, "left_foot_tip", false);
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  geometryData = tmpModel.getGeometryData();
  geometryName = tmpModel.getGeometryName();
  // Modification of geometry data
  geometryData.block(geometryName.at("camera"), 0, 1, 3) += calibration_model.getCameraOffsetsRad().transpose();
  geometryData.block(geometryName.at("head_yaw"), 0, 1, 3) += calibration_model.getNeckOffsetsRad().transpose();
  // Initialize a fixed model
  Leph::HumanoidFixedModel model(Leph::SigmabanModel, Eigen::MatrixXd(), {}, geometryData, geometryName);

  // Disable caching optimization
  model.get().setAutoUpdate(true);
  // Assign DOF state
  model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
  for (const std::string& name : Leph::NamesDOF)
  {
    model.get().setDOF(name, input.data(name));
  }
  // Assign trunk orientation from IMU
  double imuPitch = input.data("imu_pitch");
  double imuRoll = input.data("imu_roll");
  Eigen::Vector3d imu_offset_rad = M_PI / 180 * calibration_model.getImuOffsetsRad();
  Eigen::Matrix3d imuMatrix =
      Eigen::AngleAxisd(imu_offset_rad(2), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(imuPitch + imu_offset_rad(1), Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(imuRoll + imu_offset_rad(0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  model.setOrientation(imuMatrix);
  // Assign the left foot to origin
  model.get().setDOF("base_x", 0.0);
  model.get().setDOF("base_y", 0.0);
  model.get().setDOF("base_z", 0.0);
  model.get().setDOF("base_yaw", 0.0);
  // Re-enable model caching
  model.get().setAutoUpdate(false);
  model.get().updateDOFPosition();
  // Getting the position of the seenPoint in the camera
  Eigen::Vector3d seen_point(input.data("ground_x"), input.data("ground_y"), input.data("ground_z"));
  Eigen::Vector2d pixel;
  const Leph::CameraModel camera_model = calibration_model.getCameraModel();
  bool success = model.get().cameraWorldToPixel(camera_model, seen_point, pixel);

  // Now we have some tolerance as long as the point is not behind the camera
  if (!success && pixel(0) == 0 && pixel(1) == 0)
  {
    std::ostringstream oss;
    Eigen::Vector2d pos(input.data("pixel_x"), input.data("pixel_y"));
    Eigen::Vector3d viewVectorInWorld = model.get().cameraPixelToViewVector(camera_model, camera_model.getCenter());
    Eigen::Vector3d groundAtCenter;
    bool isSuccess = model.get().cameraViewVectorToWorld(viewVectorInWorld, groundAtCenter, input.data("ground_z"));
    if (!isSuccess)
    {
      oss << "CalibrationPredictor::predictObservation: failed cameraWorldToPixel AND "
          << " failed cameraViewVectorToPixel" << std::endl;
    }
    oss << "CalibrationPredictor::predictObservation: failed cameraWorldToPixel:" << std::endl
        << " tag_id: " << input.data("tag_id") << std::endl
        << " point: " << seen_point.transpose() << std::endl
        << " measured pos in image:" << pos.transpose() << std::endl
        << " viewVecInWorld: " << viewVectorInWorld.transpose() << std::endl
        << " groundAtCenter: " << groundAtCenter.transpose() << std::endl;
    throw std::logic_error(oss.str());
  }
  // Add noise if required
  if (engine != nullptr)
  {
    std::normal_distribution<double> observation_noise(0, calibration_model.getPxStddev());
    pixel(0) += observation_noise(*engine);
    pixel(1) += observation_noise(*engine);
  }
  return pixel;
}

double CalibrationPredictor::computeLogLikelihood(const Sample& sample, const Model& model,
                                                  std::default_random_engine* engine) const
{
  const CalibrationModel& calibration_model = dynamic_cast<const CalibrationModel&>(model);

  (void)engine;
  Eigen::Vector2d prediction = predictObservation(sample.getInput(), model, nullptr);
  Eigen::Vector2d observation = sample.getObservation();

  double px_stddev = calibration_model.getPxStddev();
  double px_var = px_stddev * px_stddev;
  Eigen::MatrixXd covar(2, 2);
  covar << px_var, 0, 0, px_var;
  rhoban_random::MultivariateGaussian expected_distribution(prediction, covar);

  return expected_distribution.getLogLikelihood(observation);
}

Eigen::VectorXi CalibrationPredictor::getObservationsCircularity() const
{
  return Eigen::VectorXi::Zero(2);
}

std::string CalibrationPredictor::getClassName() const
{
  return "CalibrationPredictor";
}

}  // namespace rhoban_model_learning
