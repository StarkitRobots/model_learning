#include "rhoban_model_learning/humanoid_models/poses_optimization_model.h"

#include "rhoban_model_learning/humanoid_models/camera_model.h"
#include "rhoban_model_learning/humanoid_models/multi_poses_model.h"
#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"

#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{

typedef PosesOptimizationModel POM;

POM::PosesOptimizationModel() :
  CompositeModel()
{
  models["noise" ] = std::unique_ptr<Model>(new VisionNoiseModel);
  models["camera"] = std::unique_ptr<Model>(new CameraModel);
  models["poses" ] = std::unique_ptr<Model>(new MultiPosesModel);
}

POM::PosesOptimizationModel(const PosesOptimizationModel & other)
  : CompositeModel(other)
{
}

double POM::getPxStddev() const {
  return static_cast<const VisionNoiseModel &>(*models.at("noise")).px_stddev;
}

const Leph::CameraModel & POM::getCameraModel() const {
  return static_cast<const CameraModel &>(*models.at("camera")).model;
}

const PoseModel & POM::getPose(int idx) const {
  return static_cast<const MultiPosesModel &>(*models.at("poses")).getPose(idx);
}

std::unique_ptr<Model> POM::clone() const {
  return std::unique_ptr<Model>(new POM(*this));
}
void POM::fromJson(const Json::Value & v, const std::string & dir_name) {
  CompositeModel::fromJson(v, dir_name);
  // Checking that content has been appropriately set
  try{
    dynamic_cast<const CameraModel &>(*models.at("camera"));
  } catch (const std::bad_cast & e) {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'camera'");
  }
  try{
    dynamic_cast<const VisionNoiseModel &>(*models.at("noise"));
  } catch (const std::bad_cast & e) {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'noise'");
  }
  try{
    dynamic_cast<const MultiPosesModel &>(*models.at("poses"));
  } catch (const std::bad_cast & e) {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'poses'");
  }
}

std::string POM::getClassName() const {
  return "PosesOptimizationModel";
}

}
