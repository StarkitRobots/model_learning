#include "rhoban_model_learning/model.h"

namespace rhoban_model_learning
{
/// A simple noise model through pixel standard deviation
class VisionNoiseModel : public Model
{
public:
  VisionNoiseModel();
  VisionNoiseModel(const VisionNoiseModel& other);

  int getParametersSize() const override;
  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;
  std::string getClassName() const override;

  std::unique_ptr<Model> clone() const override;

  double px_stddev;
};

}  // namespace rhoban_model_learning
