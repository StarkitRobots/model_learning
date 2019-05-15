#include "starkit_model_learning/tools.h"

namespace starkit_model_learning
{
Eigen::VectorXd extractSubset(const Eigen::VectorXd& vector, const std::vector<int>& used_indices)
{
  Eigen::VectorXd result(used_indices.size());
  int i = 0;
  for (int idx : used_indices)
  {
    result(i++) = vector(idx);
  }
  return result;
}

Eigen::VectorXd extractSubset(const Eigen::VectorXd& vector, const std::set<int>& used_indices)
{
  Eigen::VectorXd result(used_indices.size());
  int i = 0;
  for (int idx : used_indices)
  {
    result(i++) = vector(idx);
  }
  return result;
}

}  // namespace starkit_model_learning
