#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include "Model/HumanoidFixedModel.hpp"
#include "Types/MatrixLabel.hpp"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{

typedef VisionCorrectionModel VCM;

static int nb_parameters =
  1 + // px_stddev
  3 + // camera geometric deformation
  3 + // imu geometric deformation
  3 + // neck geometric deformation
  2 + // FocalX, FocalY
  2 + // CenterX, CenterY
  5   // Distortion coefficients
  ;

VCM::VisionInput::VisionInput() {}
VCM::VisionInput::VisionInput(const Leph::VectorLabel & data_)
  : data(data_)
{}
VCM::VisionInput::VisionInput(const VisionInput & other)
  : data(other.data)
{}


std::unique_ptr<Input> VCM::VisionInput::clone() const {
  return std::unique_ptr<Input>(new VisionInput(*this));
}

VCM::VisionInputReader::VisionInputReader()
  : x_coord_range(0, 10000), y_coord_range(0, 10000),
    nb_training_tags(0), nb_validation_tags(0),
    max_samples_per_tag(1), min_samples_per_tag(1),
    verbose(true)
{
}

DataSet VCM::VisionInputReader::extractSamples(const std::string & file_path,
                                               std::default_random_engine * engine) const
{
  Leph::MatrixLabel logs;
  logs.load(file_path);
  // First: Read all entries and conserve only valid ones
  std::map<int,SampleVector> samples_by_id;
  for (size_t i = 0; i < logs.size(); i++) {
    const Leph::VectorLabel & entry = logs[i];
    int tag_id = entry("tag_id");
    Eigen::Vector2d observation(entry("pixel_x"), entry("pixel_y"));
    std::unique_ptr<Input> input(new VisionInput(entry));
    std::unique_ptr<Sample> sample(new Sample(std::move(input), observation));

    double x = observation(0);
    double y = observation(1);

    if (x >= x_coord_range(0) && x <= x_coord_range(1) &&
        y >= y_coord_range(0) && y <= y_coord_range(1)) {
      samples_by_id[tag_id].push_back(std::move(sample));
    }
  }
  // Get valid indices
  if (verbose) {
    std::cout << "Filtering tag indices" << std::endl;
  }
  std::vector<int> tags_indices;
  for (const auto & pair : samples_by_id) {
    int tag_id = pair.first;
    int nb_samples = pair.second.size();
    if (nb_samples >= min_samples_per_tag) {
      tags_indices.push_back(pair.first);
    } else if (verbose) {
      std::cout << "\tIgnoring tag " << tag_id << " because it has only "
                << nb_samples << " valid samples" << std::endl;
    }
  }
  // Choose which tags will be used for training and validation
  size_t training_size = nb_training_tags;
  size_t validation_size = nb_validation_tags;
  if (training_size == 0 && validation_size == 0) { 
    throw std::runtime_error(DEBUG_INFO + "No size specified for either training or validation");
  } else if (training_size + validation_size > tags_indices.size()) {
    throw std::runtime_error(DEBUG_INFO + "Not enough tags available: ("
                             + std::to_string(validation_size) + "+"
                             + std::to_string(training_size) + ">"
                             + std::to_string(tags_indices.size()) + ")");
  }
  if (validation_size == 0) {
    validation_size = tags_indices.size() - training_size;
  }
  if (training_size == 0) {
    training_size = tags_indices.size() - validation_size;
  }
    

  std::vector<size_t> set_sizes =
    {training_size, validation_size, tags_indices.size() - (training_size+validation_size)};
  std::vector<std::vector<size_t>> separated_indices;
  separated_indices = rhoban_random::splitIndices(tags_indices.size() - 1, set_sizes, engine);
  // Fill data set
  DataSet data;
  if (verbose) std::cout << "Training indices" << std::endl;
  for (size_t idx : separated_indices[0]) {
    // Limited number of samples for training_set
    int tag_id = tags_indices[idx];
    const SampleVector & tag_samples = samples_by_id[tag_id];
    int nb_samples = tag_samples.size();
    std::vector<size_t> samples_indices =
      rhoban_random::getUpToKDistinctFromN(max_samples_per_tag, nb_samples, engine);
    for (size_t sample_idx : samples_indices) {
      data.training_set.push_back(tag_samples[sample_idx]->clone());
    }
    if (verbose) {
      std::cout << "\t" << tag_id << ": " << samples_indices.size() << " samples" << std::endl;
    }
  }
  if (verbose) std::cout << "Validation indices" << std::endl;
  for (size_t idx : separated_indices[1]) {
    // Limited number of samples for training_set
    int tag_id = tags_indices[idx];
    const SampleVector & tag_samples = samples_by_id[tag_id];
    for (const auto & sample : tag_samples) {
      data.validation_set.push_back(sample->clone());
    }
    if (verbose) {
      std::cout << "\t" << tag_id << ": " << tag_samples.size() << " samples" << std::endl;
    }
  }

  if (verbose) {
    std::cout << "Training Set size: " << data.training_set.size() << std::endl;
    std::cout << "Validation Set size: " << data.validation_set.size() << std::endl;
  }
  
  return data;
}

std::string VCM::VisionInputReader::getClassName() const {
  return "VisionInputReader";
}

Json::Value VCM::VisionInputReader::toJson() const {
  Json::Value  v;
  v["x_coord_range"] = rhoban_utils::vector2Json(x_coord_range);
  v["y_coord_range"] = rhoban_utils::vector2Json(y_coord_range);
  v["nb_training_tags"   ] = nb_training_tags   ;
  v["nb_validation_tags" ] = nb_validation_tags ;
  v["max_samples_per_tag"] = max_samples_per_tag;
  v["min_samples_per_tag"] = min_samples_per_tag;
  v["verbose"            ] = verbose            ;
  return v;
}
void VCM::VisionInputReader::fromJson(const Json::Value & v, 
                                      const std::string & dir_name) {
  (void) dir_name;
  rhoban_utils::tryReadEigen(v, "x_coord_range", &x_coord_range);
  rhoban_utils::tryReadEigen(v, "y_coord_range", &y_coord_range);
  rhoban_utils::tryRead(v, "nb_training_tags"   , &nb_training_tags   );
  rhoban_utils::tryRead(v, "nb_validation_tags" , &nb_validation_tags );
  rhoban_utils::tryRead(v, "max_samples_per_tag", &max_samples_per_tag);
  rhoban_utils::tryRead(v, "min_samples_per_tag", &min_samples_per_tag);
  rhoban_utils::tryRead(v, "verbose"            , &verbose            );
}


VCM::VisionCorrectionModel() :
  ModularModel(nb_parameters),
  cam_offset (Eigen::Vector3d::Zero()),
  imu_offset (Eigen::Vector3d::Zero()),
  neck_offset(Eigen::Vector3d::Zero()),
  camera_model(),
  px_stddev_space(0.01, 50),
  max_angle_error(5),
  focal_length_space(0,500),
  center_max_error(20),
  max_distortion(30)
{
  px_stddev = (px_stddev_space(0) + px_stddev_space(1)) / 2;
}

VCM::VisionCorrectionModel(const VisionCorrectionModel & other)
  : ModularModel(other), px_stddev(other.px_stddev), cam_offset(other.cam_offset),
    imu_offset(other.imu_offset), neck_offset(other.neck_offset),
    camera_model(other.camera_model),
    px_stddev_space(other.px_stddev_space),
    max_angle_error(other.max_angle_error),
    focal_length_space(other.focal_length_space),
    center_max_error(other.center_max_error),
    max_distortion(other.max_distortion)
{
}

double VCM::getPxStddev() const {
  return px_stddev;
}

Eigen::Vector3d VCM::getCameraOffsetsRad() const {
  return M_PI/ 180 * cam_offset;
}

Eigen::Vector3d VCM::getImuOffsetsRad() const {
  return M_PI/ 180 * imu_offset;
}

Eigen::Vector3d VCM::getNeckOffsetsRad() const {
  return M_PI/ 180 * neck_offset;
}

const Leph::CameraModel & VCM::getCameraModel() const {
  return camera_model;
}


Eigen::VectorXd VCM::getGlobalParameters() const  {
  Eigen::VectorXd params(nb_parameters);
  int i = 0;
  params(i++) = px_stddev;
  params.segment(i,3) = cam_offset; i+=3;
  params.segment(i,3) = imu_offset; i+=3;
  params.segment(i,3) = neck_offset; i+=3;
  params(i++) = camera_model.getFocalX();
  params(i++) = camera_model.getFocalY();
  params(i++) = camera_model.getCenterX();
  params(i++) = camera_model.getCenterY();
  params.segment(i,5) = camera_model.getDistortionCoeffsAsEigen();
  return params;
}

Eigen::MatrixXd VCM::getGlobalParametersSpace() const  {
  Eigen::MatrixXd space(nb_parameters,2);
  space.row(0) = px_stddev_space.transpose();
  for (int row = 1; row < 10; row++) {
    space(row,0) = -max_angle_error;
    space(row,1) = max_angle_error;
  }
  int i=10;
  space.row(i++) = focal_length_space;
  space.row(i++) = focal_length_space;
  space.row(i++) = Eigen::Vector2d(camera_model.getCenterX() - center_max_error,
                                   camera_model.getCenterX() + center_max_error);
  space.row(i++) = Eigen::Vector2d(camera_model.getCenterY() - center_max_error,
                                   camera_model.getCenterY() + center_max_error);
  Eigen::Vector2d distortion_basis(-max_distortion, max_distortion);
  double halfImgDiag = camera_model.getImgDiag() / 2;
  space.row(i++) = distortion_basis / std::pow(halfImgDiag,2);//k_1 grows with r^2
  space.row(i++) = distortion_basis / std::pow(halfImgDiag,4);//k_2 grows with r^4
  space.row(i++) = distortion_basis / std::pow(halfImgDiag,2);//p_1 grows with r^2
  space.row(i++) = distortion_basis / std::pow(halfImgDiag,2);//p_2 grows with r^4
  space.row(i++) = distortion_basis / std::pow(halfImgDiag,6);//k_3 grows with r^6
  return space;
}

void VCM::setGlobalParameters(const Eigen::VectorXd & new_params)  {
  if (new_params.rows() != nb_parameters) {
    throw std::logic_error("VCM::setParameters: unexpected size for new_params"
                           + std::to_string(new_params.rows()));
  }
  px_stddev = new_params(0);
  cam_offset  = new_params.segment(1,3);
  imu_offset  = new_params.segment(4,3);
  neck_offset = new_params.segment(7,3);
  int i = 10;
  camera_model.setCenter(new_params.segment(i,2)); i+= 2;
  camera_model.setFocal(new_params.segment(i,2)); i+= 2;
  camera_model.setDistortion(new_params.segment(i,5)); i+= 5;
}

std::vector<std::string> VCM::getGlobalParametersNames() const  {
  std::vector<std::string> names(nb_parameters);
  names[0] = "px_stddev";
  std::vector<std::string> transformations = {"roll", "pitch", "yaw"};
  for (int i = 0; i < 3; i++) {
    names[i+1] = "cam_offset_" + transformations[i];
    names[i+4] = "imu_offset_" + transformations[i];
    names[i+7] = "neck_offset_" + transformations[i];
  }
  int i = 10;
  names[i++] = "focal_x";
  names[i++] = "focal_y";
  names[i++] = "center_x";
  names[i++] = "center_y";
  names[i++] = "k1";
  names[i++] = "k2";
  names[i++] = "p1";
  names[i++] = "p2";
  names[i++] = "k3";
  return names;
}

Eigen::VectorXi VCM::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(nb_parameters);
}

Eigen::VectorXd VCM::predictObservation(const Input & raw_input,
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
  geometryData.block(geometryName.at("camera"),0,1,3) += getCameraOffsetsRad().transpose();
  geometryData.block(geometryName.at("head_yaw"),0,1,3) += getNeckOffsetsRad().transpose();
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
  Eigen::Vector3d imu_offset_rad = M_PI/ 180 * imu_offset;
  Eigen::Matrix3d imuMatrix =
    Eigen::AngleAxisd(imu_offset_rad(2), Eigen::Vector3d::UnitZ()).toRotationMatrix()
    * Eigen::AngleAxisd(imuPitch + imu_offset_rad(1), Eigen::Vector3d::UnitY()).toRotationMatrix()
    * Eigen::AngleAxisd(imuRoll + imu_offset_rad(0), Eigen::Vector3d::UnitX()).toRotationMatrix();
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
  bool success = model.get().cameraWorldToPixel(camera_model,
                                                seen_point, pixel);

  if (!success) {
    std::ostringstream oss;
    Eigen::Vector2d pos(input.data("pixel_x"), input.data("pixel_y"));
    Eigen::Vector3d viewVectorInWorld =
      model.get().cameraPixelToViewVector(camera_model, Eigen::Vector2d(0,0));
    Eigen::Vector3d groundAtCenter;
    bool isSuccess = model.get().cameraViewVectorToWorld(
      viewVectorInWorld, groundAtCenter, input.data("ground_z"));
    if (!isSuccess) { 
      oss << "VCM::predictObservation: failed cameraWorldToPixel AND "
          << " failed cameraViewVectorToPixel"<< std::endl;
    }
    oss << "VCM::predictObservation: failed cameraWorldToPixel:" << std::endl
        << " tag_id: " << input.data("tag_id") << std::endl
        << " point: " << seen_point.transpose() << std::endl
        << " measured pos in image:" << pos.transpose() << std::endl
        << " viewVecInWorld: " << viewVectorInWorld.transpose() << std::endl
        << " groundAtCenter: " << groundAtCenter.transpose() << std::endl;
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

double VCM::computeLogLikelihood(const Sample & sample,
                                 std::default_random_engine * engine) const {
  (void) engine;
  Eigen::Vector2d prediction = predictObservation(sample.getInput(), nullptr);
  Eigen::Vector2d observation = sample.getObservation();

  double px_var = px_stddev * px_stddev;
  Eigen::MatrixXd covar(2,2);
  covar << px_var, 0, 0, px_var;
  rhoban_random::MultivariateGaussian expected_distribution(prediction, covar);
  return expected_distribution.getLogLikelihood(observation);
}

std::unique_ptr<Model> VCM::clone() const {
  return std::unique_ptr<Model>(new VCM(*this));
}

Json::Value VCM::toJson() const  {
  Json::Value v = ModularModel::toJson();
  v["px_stddev"] = px_stddev;
  v["px_stddev_space"] = rhoban_utils::vector2Json<2>(px_stddev_space);
  v["cam_offset" ] = rhoban_utils::vector2Json<3>(cam_offset );
  v["imu_offset" ] = rhoban_utils::vector2Json<3>(imu_offset );
  v["neck_offset"] = rhoban_utils::vector2Json<3>(neck_offset);
  v["camera_model"] = camera_model.toJson();
  v["max_angle_error"] = max_angle_error;
  v["focal_length_space"]  = rhoban_utils::vector2Json<2>(focal_length_space);
  v["center_max_error"] = center_max_error;
  v["max_distortion"] = max_distortion;
  return v;
}

void VCM::fromJson(const Json::Value & v, const std::string & dir_name) {
  ModularModel::fromJson(v, dir_name);
  rhoban_utils::tryRead(v,"px_stddev", &px_stddev);
  rhoban_utils::tryReadEigen(v,"px_stddev_space", &px_stddev_space);
  rhoban_utils::tryReadEigen(v,"cam_offset", &cam_offset);
  rhoban_utils::tryReadEigen(v,"imu_offset", &imu_offset);
  rhoban_utils::tryReadEigen(v,"neck_offset", &neck_offset);
  camera_model.read(v, "camera_model", dir_name);
  rhoban_utils::tryRead(v, "max_angle_error", &max_angle_error);
  rhoban_utils::tryReadEigen(v,"focal_length_space", &focal_length_space);
  rhoban_utils::tryRead(v, "center_max_error", &center_max_error);
  rhoban_utils::tryRead(v, "max_distortion", &max_distortion);
}

std::string VCM::getClassName() const {
  return "VisionCorrectionModel";
}

}
