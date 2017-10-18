#include "rosban_model_learning/models/simple_angular_model.h"
#include "rosban_model_learning/models/simple_angular_model_input.h"

#include "rosban_random/multivariate_gaussian.h"

namespace rosban_model_learning
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

void SimpleAngularModel::setParameters(const Eigen::VectorXd & new_params)
{
  if (new_params.rows() != 2) {
    std::ostringstream oss;
    oss << "Invalid size for params of SimpleAngularModel: expecting 2 and got ";
    oss << new_params.rows();
    throw std::logic_error(oss.str());
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

Eigen::VectorXd SimpleAngularModel::predictObservation(const Input & input,
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

  Eigen::VectorXd result(1);
  result(0) = obs;
  return result;
}

double SimpleAngularModel::computeLogLikelihood(const Sample & sample,
                                                std::default_random_engine * engine) const
{
  bool use_analytic_model = false;
  if (use_analytic_model) {
    const SimpleAngularModelInput & casted_input =
      dynamic_cast<const SimpleAngularModelInput &>(sample.getInput());
    double obs_var = 2 * pow(observation_stddev,2);
    double step_var = casted_input.nb_steps * pow(step_stddev,2);
    Eigen::VectorXd mu(1);
    Eigen::MatrixXd covar(1,1);
    mu(0) = 0;
    covar(0,0) = obs_var + step_var;
    rosban_random::MultivariateGaussian distrib(mu, covar);
    return distrib.getLogLikelihood(sample.getObservation());
  } else {
    return Model::computeLogLikelihood(sample,engine);
  }
}

std::unique_ptr<Model> SimpleAngularModel::clone() const
{
  return std::unique_ptr<Model>(new SimpleAngularModel(observation_stddev, step_stddev));
}

}
