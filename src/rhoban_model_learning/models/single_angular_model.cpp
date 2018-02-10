#include "rhoban_model_learning/models/single_angular_model.h"

namespace rhoban_model_learning
{

SimpleAngularModel::SimpleAngularModel() : SimpleAngularModel(0,0)
{
}

SimpleAngularModel::SimpleAngularModel(double observation_stddev_,
                                       double step_stddev_) :
  observation_stddev(observation_stddev_),
  step_stddev(step_stddev_)
{
}

Eigen::VectorXd SimpleAngularModel::getParameters() const
{
  return Eigen::Vector2d(observation_stddev, step_stddev);
}

void SimpleAngularModel::setParameters(const Eigen::VectorXd & new_params) const
{
  if (new_params.rows() != 2) {
    throw std::logic_error("Invalid size for params of SimpleAngularModel: expecting 2 and got "
                           + to_str(new_params.rows()));
  }
  observation_stddev = new_params(0);
  step_stddev = new_params(1);
}

std::vector<std::string> SimpleAngularModel::getParametersNames() const
{
  return {"observation_stddev","step_stddev"};
}

Eigen::VectorXi SimpleAngularModel::getObservationsCircularity() const
{
  return Eigen::VectorXi(1);
}

Eigen::VectorXd SingleAngularModel::predictObservation(const Input & input,
                                                       std::default_random_engine * engine) const
{
  const SimpleAngularModelInput & casted_input =
    dynamic_cast<const SimpleAngularModelInput &>(input);
  // Creating distributions
  std::normal_distribution<double> obs_distrib(0,observation_stddev);
  std::normal_distribution<double> step_distrib(0,step_stddev);
  //
  double obs = 0;
  obs += obs_distrib(*engine);
  for (int i = 0; i < casted_input.nb_steps; i++) {
    obs += step_distrib(*engine);
  }
  obs += obs_distrib(*engine);
}
