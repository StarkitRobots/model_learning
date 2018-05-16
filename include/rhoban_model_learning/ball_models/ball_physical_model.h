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


  virtual Eigen::VectorXd getGlobalParameters() const;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const;
  virtual void setGlobalParameters(const Eigen::VectorXd & new_params);
  virtual std::vector<std::string> getGlobalParametersNames() const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const;

private:
  /// The number of parameters of the full physical model
  static int nb_parameters;

  /// Physical coefficients of the grass:
  /// TODO: to be listed

  /// The direction toward which is pointing the blade of the grass [deg]
  ///
  /// Dir = 0 means that the highest friction of the ball occurs when the ball
  /// is rolling in direction (-1,0)
  rhoban_utils::Angle blade_grass_direction;

  /// Maximal duration of an integration step [s]
  double max_integration_step;
};

}
