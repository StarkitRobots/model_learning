#include "rhoban_model_learning/model.h"

namespace rhoban_model_learning
{
/// Simple class with an affine model and a measurement noise
class LinearModel : public Model
{
public:
  LinearModel();
  LinearModel(int dim);
  LinearModel(const LinearModel& other);

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;

  std::unique_ptr<Model> clone() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const override;

  /// Coefficients of the affine function
  Eigen::VectorXd coeffs;

  /// Value at origin
  double bias;

  /// Measurement noise
  double std_dev;
};

}  // namespace rhoban_model_learning
