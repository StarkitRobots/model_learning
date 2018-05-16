#include "rhoban_model_learning/ball_models/ball_physical_model.h"

namespace rhoban_model_learning
{

int BallPhysicalModel::nb_parameters = 3

BallPhysicalModel::BallPhysicalModel()
  : ModularModel(nb_parameters),
    blade_grass_direction(0), max_integration_step(0.1)
{
}

BallPhysicalModel::~BallPhysicalModel() {}

Eigen::VectorXd BallPhysicalModel::GetGlobalParameters() const {
  Eigen::VectorXd params(0);
  throw std::logic_error(DEBUG_INFO + " not implemented");
}

Eigen::MatrixXd BallPhysicalModel::getGlobalParametersSpace() const {
  Eigen::MatrixXd limits(2,2);

  int dim = 0;
  limits(dim,0) = -180; limits(dim,1) = 180; dim++;
  limits(dim,0) = 0.0001; limits(dim,1) = 1.0; dim++;
  throw std::logic_error(DEBUG_INFO + " not implemented");
}
void BallPhysicalModel::setGlobalParameters(const Eigen::VectorXd & new_params) {
  if (new_params.rows() != nb_parameters) {
    throw std::logic_error(DEBUG_INFO + " invalid number of parameters, "
                           + new_params.rows() + " received, "
                           + nb_parameters + " expected");
  }
  // TODO: set parameters
}

std::vector<std::string> BallPhysicalModel::getGlobalParametersNames() const {
  // TODO: to be completed
  return { "blade_grass_direction", "max_integration_step"};
}

Json::Value BallPhysicalModel::toJson() const {
  Json::Value v;
  v["blade_grass_direction"] = blade_grass_direction.getSignedValue();
  v["max_integration_step" ] = max_integration_step;
  return v;
}

void BallPhysicalModel::fromJson(const Json::Value & v, const std::string & dir_name) {
  double bgd_deg = blade_grass_direction.getSignedValue();
  rhoban_utils::tryRead(v, "blade_grass_direction",  &bdg_deg             );
  rhoban_utils::tryRead(v, "max_integration_step" ,  &max_integration_step);
  blade_grass_direction = Angle(bdg_deg);
}

std::string BallPhysicalModel::getClassName() const {
  return "BallPhysicalModel";
}

}
