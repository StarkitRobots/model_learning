#include "starkit_model_learning/ball_models/ball_physical_model.h"

#include "starkit_utils/util.h"

#include <iostream>

using namespace starkit_utils;

namespace starkit_model_learning
{
int BallPhysicalModel::nb_parameters = 8;

BallPhysicalModel::BallPhysicalModel()
  : PositionPredictor(nb_parameters)
  , base_dry(0.0)
  , base_visc(0.05)
  , opp_dry(0.0)
  , opp_visc(0.0)
  , lat_dry(0.0)
  , lat_visc(0.0)
  , blade_grass_direction(0)
  , max_integration_step(0.1)
  , min_speed(0.02)
{
}

BallPhysicalModel::~BallPhysicalModel()
{
}

Eigen::VectorXd BallPhysicalModel::predictObservation(const starkit_model_learning::Input& raw_input,
                                                      std::default_random_engine* engine) const
{
  (void)engine;
  if (max_integration_step <= 0)
  {
    throw std::logic_error(DEBUG_INFO +
                           "Invalid value for max_integration_step: " + std::to_string(max_integration_step));
  }
  try
  {
    const Input& input = dynamic_cast<const Input&>(raw_input);
    Eigen::Vector2d ball_pos = input.ball_pos;
    Eigen::Vector2d ball_speed = input.ball_speed;
    Eigen::Vector2d grass_dir(cos(blade_grass_direction), sin(blade_grass_direction));
    double time_to_prediction = input.prediction_duration;
    while (time_to_prediction > 0 && ball_speed.norm() > min_speed)
    {
      Eigen::Vector2d ball_dir = ball_speed.normalized();
      Eigen::Vector2d ball_dir_perp(ball_dir(1), -ball_dir(0));

      double dt = std::min(time_to_prediction, max_integration_step);
      Eigen::Vector2d ball_acc(0.0, 0.0);
      // Applying friction part independent of grass
      ball_acc -= base_dry * ball_dir;
      ball_acc -= base_visc * ball_speed.norm() * ball_dir;
      // TODO: apply grass friction (opp+lat)
      double dot_product = ball_dir.dot(grass_dir);
      double cross_product = std::fabs(ball_dir(0) * grass_dir(1) - ball_dir(1) * grass_dir(0));
      // Applying grass friction
      ball_acc -= dot_product * opp_dry * ball_dir;
      ball_acc -= dot_product * opp_visc * ball_speed.norm() * ball_dir;
      ball_acc += cross_product * lat_dry * grass_dir;
      ball_acc += cross_product * lat_visc * ball_speed.norm() * grass_dir;
      // Updating status values
      ball_pos += ball_speed * dt;
      Eigen::Vector2d new_ball_speed = ball_speed + ball_acc;
      if (ball_acc.norm() > ball_speed.norm())
      {
        new_ball_speed = Eigen::Vector2d(0.0, 0.0);
      }
      ball_pos += (ball_speed + new_ball_speed) / 2 * dt;
      ball_speed = new_ball_speed;
      time_to_prediction -= dt;
    }
    Eigen::Vector4d result;
    result.segment(0, 2) = ball_pos;
    result.segment(2, 2) = ball_speed;
    return result;
  }
  catch (const std::bad_cast& exc)
  {
    throw std::logic_error(DEBUG_INFO + "Invalid type for input");
  }
}

Eigen::VectorXd BallPhysicalModel::getGlobalParameters() const
{
  Eigen::VectorXd params(nb_parameters);
  int dim = 0;
  params(dim++) = base_dry;
  params(dim++) = base_visc;
  params(dim++) = opp_dry;
  params(dim++) = opp_visc;
  params(dim++) = lat_dry;
  params(dim++) = lat_visc;
  params(dim++) = blade_grass_direction.getSignedValue();
  params(dim++) = max_integration_step;
  params(dim++) = min_speed;
  return params;
}

Eigen::MatrixXd BallPhysicalModel::getGlobalParametersSpace() const
{
  Eigen::MatrixXd limits(nb_parameters, 2);
  int dim = 0;
  limits(dim, 0) = 0;
  limits(dim, 1) = 1;
  dim++;  // base_dry
  limits(dim, 0) = 0;
  limits(dim, 1) = 0.5;
  dim++;  // base_visc
  limits(dim, 0) = 0;
  limits(dim, 1) = 1;
  dim++;  // opp_dry
  limits(dim, 0) = 0;
  limits(dim, 1) = 0.5;
  dim++;  // opp_visc
  limits(dim, 0) = 0;
  limits(dim, 1) = 1;
  dim++;  // lat_dry
  limits(dim, 0) = 0;
  limits(dim, 1) = 0.5;
  dim++;  // lat_visc
  limits(dim, 0) = -180;
  limits(dim, 1) = 180;
  dim++;
  limits(dim, 0) = 0.0001;
  limits(dim, 1) = 1.0;
  dim++;
  limits(dim, 0) = 0.01;
  limits(dim, 1) = 0.05;
  dim++;  // min_speed
  return limits;
}
void BallPhysicalModel::setGlobalParameters(const Eigen::VectorXd& new_params)
{
  if (new_params.rows() != nb_parameters)
  {
    throw std::logic_error(DEBUG_INFO + " invalid number of parameters, " + std::to_string(new_params.rows()) +
                           " received, " + std::to_string(nb_parameters) + " expected");
  }
  // TODO: set parameters
  int dim = 0;
  base_dry = new_params(dim++);
  base_visc = new_params(dim++);
  opp_dry = new_params(dim++);
  opp_visc = new_params(dim++);
  lat_dry = new_params(dim++);
  lat_visc = new_params(dim++);
  blade_grass_direction = Angle(new_params(dim++));
  max_integration_step = new_params(dim++);
  min_speed = new_params(dim++);
}

std::vector<std::string> BallPhysicalModel::getGlobalParametersNames() const
{
  return { "base_dry",
           "base_visc",
           "opp_dry",
           "opp_visc",
           "lat_dry",
           "lat_visc",
           "blade_grass_direction",
           "max_integration_step",
           "min_speed" };
}

Json::Value BallPhysicalModel::toJson() const
{
  Json::Value v = Model::toJson();
  v["base_dry"] = base_dry;
  v["base_visc"] = base_visc;
  v["opp_dry"] = opp_dry;
  v["opp_visc"] = opp_visc;
  v["lat_dry"] = lat_dry;
  v["lat_visc"] = lat_visc;
  v["blade_grass_direction"] = blade_grass_direction.getSignedValue();
  v["max_integration_step"] = max_integration_step;
  v["min_speed"] = min_speed;
  return v;
}

void BallPhysicalModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Model::fromJson(v, dir_name);
  double bgd_deg = blade_grass_direction.getSignedValue();
  starkit_utils::tryRead(v, "base_dry", &base_dry);
  starkit_utils::tryRead(v, "base_visc", &base_visc);
  starkit_utils::tryRead(v, "opp_dry", &opp_dry);
  starkit_utils::tryRead(v, "opp_visc", &opp_visc);
  starkit_utils::tryRead(v, "lat_dry", &lat_dry);
  starkit_utils::tryRead(v, "lat_visc", &lat_visc);
  starkit_utils::tryRead(v, "blade_grass_direction", &bgd_deg);
  starkit_utils::tryRead(v, "max_integration_step", &max_integration_step);
  starkit_utils::tryRead(v, "min_speed", &min_speed);
  blade_grass_direction = Angle(bgd_deg);
}

std::string BallPhysicalModel::getClassName() const
{
  return "BallPhysicalModel";
}

}  // namespace starkit_model_learning
