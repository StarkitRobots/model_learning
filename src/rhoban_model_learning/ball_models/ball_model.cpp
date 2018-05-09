#include "rhoban_model_learning/ball_models/ball_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include "Model/HumanoidFixedModel.hpp"
#include "Types/MatrixLabel.hpp"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{

typedef BallPredictionModel BPM;

BPM::BallTrajectoriesInput::BallTrajectoriesInput() {}
BPM::BallTrajectoriesInput::BallTrajectoriesInput(const Leph::VectorLabel & data_)
  : data(data_)
{}
BPM::BallTrajectoriesInput::BallTrajectoriesInput(const BallTrajectoriesInput & other)
  : data(other.data)
{}


std::unique_ptr<Input> BPM::BallTrajectoriesInput::clone() const {
  return std::unique_ptr<Input>(new BallTrajectoriesInput(*this));
}

BPM::BallTrajectoriesInputReader::BallTrajectoriesInputReader()
  : nb_input_points(2),
    nb_training_trajectories(0)
    nb_validation_trajectories(0),
    max_samples_per_trajectories(0),
    min_samples_per_trajectories(0),
    max_dt_observation(0),
    min_dt_observation(0),
    max_dt_prediction(0),
    min_dt_prediction(0),
    verbose(true)
{
}

DataSet BPM::BallTrajectoriesInputReader::extractSamples(const std::string & file_path,
                                               std::default_random_engine * engine) const
{
  Leph::MatrixLabel logs;
  logs.load(file_path);
  // First: Read all entries and conserve only valid ones
  std::map<int,SampleVector> samples_by_trajectory;
  for (size_t i = 0; i < logs.size(); i++) {
    const Leph::VectorLabel & entry = logs[i];
    int trajectory_id = entry("trajectory_id");
    Eigen::VectorXd observation(
        entry("xR"),
        entry("yR"),
        entry("thetaR"),
        entry("time"),
        entry("xB"),
        entry("yB")
        );
    std::unique_ptr<Input> input(new BallTrajectoriesInput(entry));
    std::unique_ptr<Sample> sample(new Sample(std::move(input), observation));

    samples_by_trajectory[trajectory_id].push_back(std::move(sample));
  }
  // Get valid indices
  if (verbose) {
    std::cout << "Filtering trajectories" << std::endl;
  }
  std::vector<int> trajectories_id;
  for (const auto & pair : samples_by_trajectory) {
    int trajectories_id = pair.first;
    int nb_samples = pair.second.size();
    if (nb_samples >= min_samples_per_trajectories) {
      trajectories_id.push_back(pair.first);
    } else if (verbose) {
      std::cout << "\tIgnoring trajectory " << trajectory_id << " because it has only "
                << nb_samples << " valid samples" << std::endl;
    }
  }
  // Choose which trajectories will be used for training and validation
  size_t training_size = nb_training_trajectoriess;
  size_t validation_size = nb_validation_trajectories;
  if (training_size == 0 && validation_size == 0) {
    throw std::runtime_error(DEBUG_INFO + "No size specified for either training or validation");
  } else if (training_size + validation_size > trajectories_id.size()) {
    throw std::runtime_error(DEBUG_INFO + "Not enough trajectories available: ("
                             + std::to_string(validation_size) + "+"
                             + std::to_string(training_size) + ">"
                             + std::to_string(trajectories_id.size()) + ")");
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

std::string BPM::BallTrajectoriesInputReader::getClassName() const {
  return "BallTrajectoriesInputReader";
}

Json::Value BPM::BallTrajectoriesInputReader::toJson() const {
  Json::Value  v;
  v["nb_input_points"    ] = nb_input_points    ;
  v["nb_training_trajectories"   ] = nb_training_trajectories   ;
  v["nb_validation_trajectories" ] = nb_validation_trajectories ;
  v["max_samples_per_trajectories"] = max_samples_per_trajectories;
  v["min_samples_per_trajectories"] = min_samples_per_trajectories;
  v["max_dt_observation"] = max_dt_observation;
  v["min_dt_observation"] = min_dt_observation;
  v["max_dt_prediction"] = max_dt_prediction;
  v["min_dt_prediction"] = min_dt_prediction;
  v["verbose"] = verbose;
  return v;
}
void BPM::BallTrajectoriesInputReader::fromJson(const Json::Value & v, 
                                      const std::string & dir_name) {
  (void) dir_name;
  rhoban_utils::tryRead(v, "nb_input_points", &nb_input_points);
  rhoban_utils::tryRead(v, "nb_training_trajectories", &nb_training_trajectories);
  rhoban_utils::tryRead(v, "nb_validation_trajectories", &nb_validation_trajectories );
  rhoban_utils::tryRead(v, "max_samples_per_trajectories", &max_samples_per_trajectories);
  rhoban_utils::tryRead(v, "min_samples_per_trajectories", &min_samples_per_trajectories);
  rhoban_utils::tryRead(v, "max_dt_observation", &max_dt_observation);
  rhoban_utils::tryRead(v, "min_dt_observation", &min_dt_observation);
  rhoban_utils::tryRead(v, "max_dt_prediction", &max_dt_prediction);
  rhoban_utils::tryRead(v, "min_dt_prediction", &min_dt_prediction);
  rhoban_utils::tryRead(v, "verbose", &verbose);
}

BPM::BallPredictionModel() :
//  ModularModel(12),
//  cam_offset (Eigen::Vector3d::Zero()),
//  imu_offset (Eigen::Vector3d::Zero()),
//  neck_offset(Eigen::Vector3d::Zero()),
//  camera_pan(67),
//  camera_tilt(52.5),
//  img_width(640), img_height(480),
//  px_stddev_space(0.01, 50),
//  max_angle_error(5),
//  camera_pan_space(60,70),
//  camera_tilt_space(48,55)
{
//  px_stddev = (px_stddev_space(0) + px_stddev_space(1)) / 2;
}

BPM::BallPredictionModel(const Model & other)
//  : ModularModel(other), px_stddev(other.px_stddev), cam_offset(other.cam_offset),
//    imu_offset(other.imu_offset), neck_offset(other.neck_offset),
//    camera_pan(other.camera_pan), camera_tilt(other.camera_tilt),
//    img_width(other.img_width),
//    img_height(other.img_height),
//    px_stddev_space(other.px_stddev_space),
//    max_angle_error(other.max_angle_error),
//    camera_pan_space(other.camera_pan_space),
//    camera_tilt_space(other.camera_tilt_space)
{
}

Eigen::MatrixXd BPM::getGlobalParametersSpace() const  {
  Eigen::MatrixXd space(12,2);
//  space.row(0) = px_stddev_space.transpose();
//  for (int row = 1; row < 10; row++) {
//    space(row,0) = -max_angle_error;
//    space(row,1) = max_angle_error;
//  }
//  space.row(10) = camera_pan_space.transpose();
//  space.row(11) = camera_tilt_space.transpose();
  return space;
}

void BPM::setGlobalParameters(const Eigen::VectorXd & new_params)  {
//  if (new_params.rows() != 12) {
//    throw std::logic_error("BPM::setParameters: unexpected size for new_params"
//                           + std::to_string(new_params.rows()));
//  }
//  px_stddev = new_params(0);
//  cam_offset  = new_params.segment(1,3);
//  imu_offset  = new_params.segment(4,3);
//  neck_offset = new_params.segment(7,3);
//  camera_pan = new_params(10);
//  camera_tilt = new_params(11);
}

std::vector<std::string> BPM::getGlobalParametersNames() const  {
  std::vector<std::string> names(12);
  names[0] = "px_stddev";
  std::vector<std::string> transformations = {"roll", "pitch", "yaw"};
  for (int i = 0; i < 3; i++) {
    names[i+1] = "cam_offset_" + transformations[i];
    names[i+4] = "imu_offset_" + transformations[i];
    names[i+7] = "neck_offset_" + transformations[i];
  }
  names[10] = "camera_pan";
  names[11] = "camera_tilt";
  return names;
}

Eigen::VectorXi BPM::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(12);
}

Eigen::VectorXd BPM::predictObservation(const Input & raw_input,
                                        std::default_random_engine * engine) const {
  // Can generate bad_cast error
  const BallTrajectoriesInput & input = dynamic_cast<const BallTrajectoriesInput &>(raw_input);
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
  Leph::CameraParameters camera_parameters;
  camera_parameters.widthAperture = camera_pan * M_PI / 180;
  camera_parameters.heightAperture = camera_tilt * M_PI / 180;
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
      oss << "BPM::predictObservation: failed cameraWorldToPixel AND "
          << " failed cameraViewVectorToPixel"<< std::endl;
    }
    oss << "BPM::predictObservation: failed cameraWorldToPixel:" << std::endl
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

double BPM::computeLogLikelihood(const Sample & sample,
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

std::unique_ptr<Model> BPM::clone() const {
  return std::unique_ptr<Model>(new BPM(*this));
}

Json::Value VCM::toJson() const  {
  Json::Value v = ModularModel::toJson();
  v["px_stddev"] = px_stddev;
  v["px_stddev_space"] = rhoban_utils::vector2Json<2>(px_stddev_space);
  v["cam_offset" ] = rhoban_utils::vector2Json<3>(cam_offset );
  v["imu_offset" ] = rhoban_utils::vector2Json<3>(imu_offset );
  v["neck_offset"] = rhoban_utils::vector2Json<3>(neck_offset);
  v["camera_pan"] = camera_pan;
  v["camera_tilt"] = camera_tilt;
  v["img_width"] = img_width;
  v["img_height"] = img_height;
  v["max_angle_error"] = max_angle_error;
  v["camera_pan_space"]  = rhoban_utils::vector2Json<2>(camera_pan_space );
  v["camera_tilt_space"] = rhoban_utils::vector2Json<2>(camera_tilt_space);
  return v;
}

void VCM::fromJson(const Json::Value & v, const std::string & dir_name) {
  ModularModel::fromJson(v, dir_name);
  rhoban_utils::tryRead(v,"px_stddev", &px_stddev);
  rhoban_utils::tryReadEigen(v,"px_stddev_space", &px_stddev_space);
  rhoban_utils::tryReadEigen(v,"cam_offset", &cam_offset);
  rhoban_utils::tryReadEigen(v,"imu_offset", &imu_offset);
  rhoban_utils::tryReadEigen(v,"neck_offset", &neck_offset);
  rhoban_utils::tryRead(v, "camera_pan", &camera_pan);
  rhoban_utils::tryRead(v, "camera_tilt", &camera_tilt);
  rhoban_utils::tryRead(v, "img_width", &img_width);
  rhoban_utils::tryRead(v, "img_height", &img_height);
  rhoban_utils::tryRead(v, "max_angle_error", &max_angle_error);
  rhoban_utils::tryReadEigen(v,"camera_pan_space", &camera_pan_space);
  rhoban_utils::tryReadEigen(v,"camera_tilt_space", &camera_tilt_space);
}

std::string VCM::getClassName() const {
  return "Model";
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
