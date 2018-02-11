#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"

#include "rhoban_random/multivariate_gaussian.h"

#include "Model/HumanoidFixedModel.hpp"

namespace rhoban_model_learning
{

VisionCorrectionModel::VisionInput::VisionInput() {}
VisionCorrectionModel::VisionInput::VisionInput(const Leph::VectorLabel & data_)
  : data(data_)
{}
VisionCorrectionModel::VisionInput::VisionInput(const VisionInput & other)
  : data(other.data)
{}


std::unique_ptr<Input> VisionCorrectionModel::VisionInput::clone() const {
  return std::unique_ptr<Input>(new VisionInput(*this));
}

VisionCorrectionModel::VisionCorrectionModel() {
  // TODO read a value instead of having a constant value
  camera_parameters.widthAperture = 67 * M_PI / 180.0;
  camera_parameters.heightAperture = 52.47 * M_PI / 180.0;
}


Eigen::VectorXd VisionCorrectionModel::getParameters() const  {
  // TODO: implement several options
  Eigen::VectorXd params(10);
  params(0) = px_stddev;
  params.segment(1,3) = cam_offset;
  params.segment(4,3) = imu_offset;
  params.segment(7,3) = neck_offset;
  return params;
}

void VisionCorrectionModel::setParameters(const Eigen::VectorXd & new_params)  {
  if (new_params.rows() != 10) {
    throw std::logic_error("VisionCorrectionModel::setParameters: unexpected size for new_params"
                           + std::to_string(new_params.rows()));
  }
  px_stddev = new_params(0);
  cam_offset  = new_params.segment(1,3);
  imu_offset  = new_params.segment(1,3);
  neck_offset = new_params.segment(1,3);
}

std::vector<std::string> VisionCorrectionModel::getParametersNames() const  {
  std::vector<std::string> names(10);
  names[0] = "px_stddev";
  std::vector<std::string> transformations = {"roll", "pitch", "yaw"};
  for (int i = 0; i < 3; i++) {
    names[i+1] = "cam_offset_" + transformations[i];
    names[i+4] = "imu_offset_" + transformations[i];
    names[i+7] = "neck_offset_" + transformations[i];
  }
  return names;
}

Eigen::VectorXi VisionCorrectionModel::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(10);
}

Eigen::VectorXd
VisionCorrectionModel::predictObservation(const Input & raw_input,
                                          std::default_random_engine * engine) const {
  // Can generate bad_cast error
  const VisionInput & input = dynamic_cast<const VisionInput &>(raw_input);
  // TODO: improve/understand what is hidden here (content imported from
  // appCameraModelLearning)
  // Importing geometry data from a default model
  Leph::HumanoidModel tmpModel(Leph::SigmabanModel, "left_foot_tip", false);
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  geometryData = tmpModel.getGeometryData();
  geometryName = tmpModel.getGeometryName();
  // Modification of geometry data
  geometryData.row(geometryName.at("camera")) += cam_offset;
  geometryData.row(geometryName.at("head_yaw")) += neck_offset;
  // Initialize a fixed model 
  Leph::HumanoidFixedModel model(Leph::SigmabanModel, Eigen::MatrixXd(), {},
                                 geometryData, geometryName);

  // Disable caching optimization
  model.get().setAutoUpdate(true);
  // Assign DOF state
  model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
  for (const std::string &name : Leph::NamesDOF) {
    model.get().setDOF(name, input.data(name));
  }
  // Assign trunk orientation from IMU
  double imuPitch = input.data("imu_pitch");
  double imuRoll = input.data("imu_roll");
  Eigen::Matrix3d imuMatrix =
    Eigen::AngleAxisd(imu_offset(2), Eigen::Vector3d::UnitZ()).toRotationMatrix()
    * Eigen::AngleAxisd(imuPitch + imu_offset(1), Eigen::Vector3d::UnitY()).toRotationMatrix()
    * Eigen::AngleAxisd(imuRoll + imu_offset(0), Eigen::Vector3d::UnitX()).toRotationMatrix();
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
  Eigen::Vector3d seen_point(input.data("ground_x"),
                             input.data("ground_y"),
                             input.data("ground_z"));
  Eigen::Vector2d pixel;
  bool success = model.get().cameraWorldToPixel(camera_parameters,
                                                seen_point, pixel);
  if (!success) {
    std::ostringstream oss;
    oss << "VisionCorrectionModel::predictObservation: failed cameraWorldToPixel: "
        << seen_point.transpose();
    throw std::logic_error(oss.str());
  }
  // Add noise if required
  if (engine != nullptr) {
    std::normal_distribution<double> observation_noise(0, px_stddev);
    pixel(0) += observation_noise(*engine);
    pixel(1) += observation_noise(*engine);
  }
  return pixel;
}

double VisionCorrectionModel::computeLogLikelihood(const Sample & sample,
                                                   std::default_random_engine * engine) {
  (void) engine;
  Eigen::VectorXd prediction = predictObservation(sample.getInput(), nullptr);
  Eigen::MatrixXd covar(2,2);
  covar << px_stddev, 0, 0, px_stddev;
  rhoban_random::MultivariateGaussian expected_distribution(prediction, covar);
  return expected_distribution.getLogLikelihood(sample.getObservation());
}

std::unique_ptr<Model> VisionCorrectionModel::clone() const {
  return std::unique_ptr<Model>(new VisionCorrectionModel(*this));
}

Json::Value VisionCorrectionModel::toJson() const  {
  Json::Value v;
  v["px_stddev"] = px_stddev;
  v["cam_offset"] = rhoban_utils::vector2Json(cam_offset);
  v["imu_offset"] = rhoban_utils::vector2Json(imu_offset);
  v["neck_offset"] = rhoban_utils::vector2Json(neck_offset);
  v["cam_aperture_width"] = camera_parameters.widthAperture;
  v["cam_aperture_height"] = camera_parameters.heightAperture;
  return v;
}

void VisionCorrectionModel::fromJson(const Json::Value & v,
                                     const std::string & dir_name) {
  (void) dir_name;
  rhoban_utils::tryRead(v,"px_stddev", &px_stddev);
  rhoban_utils::tryRead(v,"cam_offset", &cam_offset);
  rhoban_utils::tryRead(v,"imu_offset", &imu_offset);
  rhoban_utils::tryRead(v,"neck_offset", &neck_offset);
  rhoban_utils::tryRead(v, "cam_aperture_width", &camera_parameters.widthAperture);
  rhoban_utils::tryRead(v, "cam_aperture_height", &camera_parameters.heightAperture);
}

std::string VisionCorrectionModel::getClassName() const {
  return "VisionCorrectionModel";
}


}
