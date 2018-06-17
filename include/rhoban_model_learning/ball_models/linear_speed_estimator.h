#pragma once

#include "rhoban_model_learning/ball_models/speed_estimator.h"

namespace rhoban_model_learning
{

class LinearSpeedEstimator : public SpeedEstimator {
public:
  LinearSpeedEstimator();
  virtual ~LinearSpeedEstimator();

  virtual Eigen::VectorXd
  predictObservation(const rhoban_model_learning::Input & input,
                     std::default_random_engine * engine) const override;

  virtual Eigen::VectorXd getGlobalParameters() const override;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const override;
  virtual void setGlobalParameters(const Eigen::VectorXd & new_params) override;
  virtual std::vector<std::string> getGlobalParametersNames() const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const;

private:
  /// Duration of the window used to estimate ball speed
  double window_duration;

  /// Stddev on speed
  double noise;
  
  double max_ball_speed;
};

}
