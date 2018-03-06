#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include "Model/HumanoidFixedModel.hpp"
#include "Types/MatrixLabel.hpp"

namespace rhoban_model_learning
{

typedef VisionCorrectionModel VCM;

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
  : max_coordinates(0.7), nb_validation_tags(3), max_samples_per_tag(30)
{
}

DataSet VCM::VisionInputReader::extractSamples(const std::string & file_path,
                                               std::default_random_engine * engine) const
{
  Leph::MatrixLabel logs;
  logs.load(file_path);
  // First: Read all entries
  std::map<int,SampleVector> samples_by_id;
  for (size_t i = 0; i < logs.size(); i++) {
    const Leph::VectorLabel & entry = logs[i];
    int tag_id = entry("tag_id");
    Eigen::Vector2d observation(entry("pixel_x"), entry("pixel_y"));
    std::unique_ptr<Input> input(new VisionInput(entry));
    std::unique_ptr<Sample> sample(new Sample(std::move(input), observation));

    if (std::fabs(observation(0)) < max_coordinates &&
        std::fabs(observation(1)) < max_coordinates) {
      samples_by_id[tag_id].push_back(std::move(sample));
    }
  }
  // Get used indices
  std::vector<int> tags_indices;
  for (const auto & pair : samples_by_id) {
    tags_indices.push_back(pair.first);
  }
  // Choose which tags will be used for training and validation
  std::vector<size_t> set_sizes =
    {(size_t)(tags_indices.size() - nb_validation_tags), (size_t)nb_validation_tags};
  std::vector<std::vector<size_t>> separated_indices;
  separated_indices = rhoban_random::splitIndices(tags_indices.size() - 1, set_sizes, engine);
  // Fill data set
  DataSet data;
  std::cout << "Training indices" << std::endl;
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
    std::cout << "\t" << tag_id << ": " << samples_indices.size() << " samples" << std::endl;
  }
  std::cout << "Validation indices" << std::endl;
  for (size_t idx : separated_indices[1]) {
    // Limited number of samples for training_set
    int tag_id = tags_indices[idx];
    const SampleVector & tag_samples = samples_by_id[tag_id];
    for (const auto & sample : tag_samples) {
      data.validation_set.push_back(sample->clone());
    }
    std::cout << "\t" << tag_id << ": " << tag_samples.size() << " samples" << std::endl;
  }

  std::cout << "Training Set size: " << data.training_set.size() << std::endl;
  std::cout << "Validation Set size: " << data.validation_set.size() << std::endl;

  return data;
}

std::string VCM::VisionInputReader::getClassName() const {
  return "VisionInputReader";
}

Json::Value VCM::VisionInputReader::toJson() const {
  Json::Value  v;
  v["max_coordinates"    ] = max_coordinates    ;
  v["nb_validation_tags" ] = nb_validation_tags ;
  v["max_samples_per_tag"] = max_samples_per_tag;
  return v;
}
void VCM::VisionInputReader::fromJson(const Json::Value & v, 
                                      const std::string & dir_name) {
  (void) dir_name;
  rhoban_utils::tryRead(v, "max_coordinates"    , &max_coordinates    );
  rhoban_utils::tryRead(v, "nb_validation_tags" , &nb_validation_tags );
  rhoban_utils::tryRead(v, "max_samples_per_tag", &max_samples_per_tag);
}


VCM::VisionCorrectionModel() :
  ModularModel(10),
  cam_offset (Eigen::Vector3d::Zero()),
  imu_offset (Eigen::Vector3d::Zero()),
  neck_offset(Eigen::Vector3d::Zero()),
  img_width(640), img_height(480),
  px_stddev_space(0.01, 50),
  max_angle_error(5)
{
  // TODO read a value instead of having a constant value
  camera_parameters.widthAperture = 67 * M_PI / 180.0;
  camera_parameters.heightAperture = 52.47 * M_PI / 180.0;
  px_stddev = (px_stddev_space(0) + px_stddev_space(1)) / 2;
}

VCM::VisionCorrectionModel(const VisionCorrectionModel & other)
  : ModularModel(other), px_stddev(other.px_stddev), cam_offset(other.cam_offset),
    imu_offset(other.imu_offset), neck_offset(other.neck_offset),
    camera_parameters(other.camera_parameters),
    img_width(other.img_width),
    img_height(other.img_height),
    px_stddev_space(other.px_stddev_space),
    max_angle_error(other.max_angle_error)
{
}


Eigen::VectorXd VCM::getGlobalParameters() const  {
  Eigen::VectorXd params(10);
  params(0) = px_stddev;
  params.segment(1,3) = cam_offset;
  params.segment(4,3) = imu_offset;
  params.segment(7,3) = neck_offset;
  return params;
}

Eigen::MatrixXd VCM::getGlobalParametersSpace() const  {
  Eigen::MatrixXd space(10,2);
  space.row(0) = px_stddev_space.transpose();
  for (int row = 1; row < 10; row++) {
    space(row,0) = -max_angle_error;
    space(row,1) = max_angle_error;
  }
  return space;
}

void VCM::setGlobalParameters(const Eigen::VectorXd & new_params)  {
  if (new_params.rows() != 10) {
    throw std::logic_error("VCM::setParameters: unexpected size for new_params"
                           + std::to_string(new_params.rows()));
  }
  px_stddev = new_params(0);
  cam_offset  = new_params.segment(1,3);
  imu_offset  = new_params.segment(4,3);
  neck_offset = new_params.segment(7,3);
}

std::vector<std::string> VCM::getGlobalParametersNames() const  {
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

Eigen::VectorXi VCM::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(10);
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
  geometryData.block(geometryName.at("camera"),0,1,3) += M_PI/ 180 * cam_offset.transpose();
  geometryData.block(geometryName.at("head_yaw"),0,1,3) += M_PI/ 180 * neck_offset.transpose();
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
  bool success = model.get().cameraWorldToPixel(camera_parameters,
                                                seen_point, pixel);

  if (!success) {
    std::ostringstream oss;
    Eigen::Vector2d pos(input.data("pixel_x"), input.data("pixel_y"));
    Eigen::Vector3d viewVectorInWorld =
      model.get().cameraPixelToViewVector(camera_parameters, Eigen::Vector2d(0,0));
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
        << " groundAtCenter: " << groundAtCenter.transpose() << std::endl
        << " camParams: " << camera_parameters.widthAperture << ", " << camera_parameters.heightAperture;
    throw std::logic_error(oss.str());
  }
  // Add noise if required
  if (engine != nullptr) {
    std::normal_distribution<double> observation_noise(0, px_stddev);
    pixel = leph2Img(pixel);
    pixel(0) += observation_noise(*engine);
    pixel(1) += observation_noise(*engine);
    pixel = img2Leph(pixel);
  }
  return pixel;
}

double VCM::computeLogLikelihood(const Sample & sample,
                                 std::default_random_engine * engine) const {
  (void) engine;
  Eigen::Vector2d prediction_leph = predictObservation(sample.getInput(), nullptr);
  Eigen::Vector2d prediction_img = leph2Img(prediction_leph);
  Eigen::Vector2d observation_leph = sample.getObservation();
  Eigen::Vector2d observation_img = leph2Img(observation_leph);

  double px_var = px_stddev * px_stddev;
  Eigen::MatrixXd covar(2,2);
  covar << px_var, 0, 0, px_var;
  rhoban_random::MultivariateGaussian expected_distribution(prediction_img, covar);
  return expected_distribution.getLogLikelihood(observation_img);
}

std::unique_ptr<Model> VCM::clone() const {
  return std::unique_ptr<Model>(new VCM(*this));
}

Json::Value VCM::toJson() const  {
  Json::Value v = ModularModel::toJson();
  v["px_stddev"] = px_stddev;
  v["cam_offset" ] = rhoban_utils::vector2Json<3>(cam_offset );
  v["imu_offset" ] = rhoban_utils::vector2Json<3>(imu_offset );
  v["neck_offset"] = rhoban_utils::vector2Json<3>(neck_offset);
  v["cam_aperture_width"] = camera_parameters.widthAperture;
  v["cam_aperture_height"] = camera_parameters.heightAperture;
  v["img_width"] = img_width;
  v["img_height"] = img_height;
  return v;
}

void VCM::fromJson(const Json::Value & v, const std::string & dir_name) {
  ModularModel::fromJson(v, dir_name);
  rhoban_utils::tryRead(v,"px_stddev", &px_stddev);
  rhoban_utils::tryReadEigen(v,"cam_offset", &cam_offset);
  rhoban_utils::tryReadEigen(v,"imu_offset", &imu_offset);
  rhoban_utils::tryReadEigen(v,"neck_offset", &neck_offset);
  rhoban_utils::tryRead(v, "cam_aperture_width", &camera_parameters.widthAperture);
  rhoban_utils::tryRead(v, "cam_aperture_height", &camera_parameters.heightAperture);
  rhoban_utils::tryRead(v, "img_width", &img_width);
  rhoban_utils::tryRead(v, "img_height", &img_height);
}

std::string VCM::getClassName() const {
  return "VCM";
}

Eigen::Vector2d VCM::leph2Img(const Eigen::Vector2d & leph_px) const {
  Eigen::Vector2d img_px;
  img_px(0) = ((leph_px(0) + 1) / 2) * img_width;
  img_px(1) = ((leph_px(1) + 1) / 2) * img_height;
  return img_px;
}

Eigen::Vector2d VCM::img2Leph(const Eigen::Vector2d & img_px)  const{
  Eigen::Vector2d leph_px;
  leph_px(0) = 2 * (img_px(0) / img_width) - 1;
  leph_px(1) = 2 * (img_px(1) / img_height) - 1;
  return leph_px;
}


}
