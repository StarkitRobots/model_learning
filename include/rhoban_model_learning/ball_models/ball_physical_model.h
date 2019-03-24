#pragma once

#include "rhoban_model_learning/ball_models/position_predictor.h"

#include "rhoban_utils/angle.h"

namespace rhoban_model_learning
{
class BallPhysicalModel : public PositionPredictor
{
public:
  BallPhysicalModel();
  virtual ~BallPhysicalModel();

  virtual Eigen::VectorXd predictObservation(const rhoban_model_learning::Input& input,
                                             std::default_random_engine* engine) const override;

  virtual Eigen::VectorXd getGlobalParameters() const;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const;
  virtual void setGlobalParameters(const Eigen::VectorXd& new_params);
  virtual std::vector<std::string> getGlobalParametersNames() const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const;

private:
  /// The number of parameters of the full physical model
  static int nb_parameters;

  /// Dry friction independent of grass [m/s^2]
  double base_dry;

  /// Viscous friction independent of grass
  double base_visc;

  /// Dry friction opposite to grass [m/s^2]
  double opp_dry;

  /// Viscous friction independent of grass
  double opp_visc;

  /// Dry friction lateral with grass [m/s^2]
  double lat_dry;

  /// Viscous friction independent of grass
  double lat_visc;

  /// The direction toward which is pointing the blade of the grass [deg]
  ///
  /// Dir = 0 means that the highest friction of the ball occurs when the ball
  /// is rolling in direction (-1,0)
  rhoban_utils::Angle blade_grass_direction;

  /// Maximal duration of an integration step [s]
  double max_integration_step;

  /// Min speed from which we suppose that the ball doesn't move
  double min_speed;
};

}  // namespace rhoban_model_learning
